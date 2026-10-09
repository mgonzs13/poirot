# Copyright 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import threading
import time
from dataclasses import dataclass

from poirot.utils.sysfs_reader import SysfsReader

RAPL_PATH = "/sys/class/powercap/intel-rapl"
RATE_WINDOW_US = 250_000
MIN_INTERVAL_US = 20_000.0


@dataclass
class EnergySnapshot:
    """Point-in-time CPU energy and system busy-time reading."""

    energy_uj: float = 0.0
    busy_us: float = 0.0
    valid: bool = False


class RateWindow:
    """Lazy windowed estimator of energy per system busy microsecond."""

    def __init__(self) -> None:
        self.rate_uj_per_busy_us = 0.0
        self.start_energy_uj = 0.0
        self.start_busy_us = 0.0
        self.start_time_us = 0

    def update(
        self,
        now_us: int,
        energy_uj: float,
        busy_us: float,
        window_us: int,
    ) -> None:
        """Refresh the rate when the window has been open for window_us."""
        if self.start_time_us == 0:
            self.start_time_us = now_us
            self.start_energy_uj = energy_uj
            self.start_busy_us = busy_us
            return

        if now_us - self.start_time_us < window_us:
            return

        delta_energy_uj = energy_uj - self.start_energy_uj
        delta_busy_us = busy_us - self.start_busy_us
        if delta_energy_uj > 0.0 and delta_busy_us > 0.0:
            self.rate_uj_per_busy_us = delta_energy_uj / delta_busy_us

        self.start_time_us = now_us
        self.start_energy_uj = energy_uj
        self.start_busy_us = busy_us


class RaplMonitor:
    """
    Class for monitoring CPU energy consumption via RAPL.
    """

    def __init__(self) -> None:
        """
        Constructor.
        """
        # Mutex for thread-safe RAPL readings
        self._rapl_mutex = threading.Lock()
        # RAPL package sysfs path (empty string means RAPL is not available)
        self._rapl_package_path: str = ""
        # Accumulated energy in microjoules
        self._accumulated_energy_uj = 0.0
        # Last RAPL energy value (for delta calculation)
        self._last_rapl_energy_uj = 0.0
        # Maximum RAPL energy value before wrap-around
        self._rapl_max_energy_uj = 0.0
        # Lazy fallback rate window
        self._rate_window = RateWindow()

        self._load_rapl_package_path()
        self._initialize_rapl_max_energy()

    @staticmethod
    def attribute_energy(
        delta_energy_uj: float,
        delta_busy_us: float,
        thread_cpu_delta_us: float,
        rate_uj_per_busy_us: float,
    ) -> float:
        """Attribute a measured package-energy delta to a thread."""
        if thread_cpu_delta_us <= 0.0:
            return 0.0

        if delta_energy_uj > 0.0 and delta_busy_us >= MIN_INTERVAL_US:
            share = thread_cpu_delta_us / delta_busy_us
            share = max(0.0, min(1.0, share))
            return delta_energy_uj * share

        if rate_uj_per_busy_us > 0.0:
            return rate_uj_per_busy_us * thread_cpu_delta_us

        return 0.0

    @staticmethod
    def parse_proc_stat_busy_us(line: str, clk_tck: float) -> float:
        """Parse the aggregate `cpu` line of /proc/stat into busy time."""
        parts = line.split()
        if not parts or parts[0] != "cpu" or len(parts) < 9:
            return 0.0

        try:
            values = [float(value) for value in parts[1:9]]
        except ValueError:
            return 0.0

        if clk_tck <= 0:
            clk_tck = 100.0

        busy_jiffies = (
            values[0] + values[1] + values[2] + values[5] + values[6] + values[7]
        )
        return busy_jiffies * 1_000_000.0 / clk_tck

    def _load_rapl_package_path(self) -> None:
        """Load RAPL package path."""
        with self._rapl_mutex:
            if not os.path.exists(RAPL_PATH):
                return

            for entry in os.listdir(RAPL_PATH):
                domain_path = os.path.join(RAPL_PATH, entry)
                name_path = os.path.join(domain_path, "name")

                if os.path.exists(name_path):
                    domain_name = SysfsReader.read_string(name_path)

                    if "package" in domain_name.lower():
                        self._rapl_package_path = domain_path
                        break

    def _initialize_rapl_max_energy(self) -> None:
        """Initialize RAPL max energy range for wraparound detection."""
        with self._rapl_mutex:
            if not self._rapl_package_path:
                self._rapl_max_energy_uj = 0.0
                return

            max_range_path = f"{self._rapl_package_path}/max_energy_range_uj"

            if os.path.exists(max_range_path):
                max_val = SysfsReader.read_double(max_range_path)
                if max_val > 0:
                    self._rapl_max_energy_uj = max_val
                    return

            self._rapl_max_energy_uj = 0.0

    def _read_energy_uj_locked(self) -> float:
        """Read accumulated RAPL energy; caller must hold _rapl_mutex."""
        if not self._rapl_package_path:
            return -1.0

        energy_path = f"{self._rapl_package_path}/energy_uj"
        if not os.path.exists(energy_path):
            return -1.0

        current = SysfsReader.read_double(energy_path)
        if current <= 0:
            return -1.0

        # If we have a previous measurement, accumulate delta (handle wraparound)
        if self._last_rapl_energy_uj > 0.0:
            if current < self._last_rapl_energy_uj:
                # Counter wrapped around
                if self._rapl_max_energy_uj > 0.0:
                    self._accumulated_energy_uj += (
                        self._rapl_max_energy_uj - self._last_rapl_energy_uj + current
                    )
            else:
                # Normal case
                self._accumulated_energy_uj += current - self._last_rapl_energy_uj

        self._last_rapl_energy_uj = current
        return self._accumulated_energy_uj

    def read_energy_uj(self) -> float:
        """
        Read accumulated CPU energy consumption in microjoules.

        Returns:
            Accumulated energy consumption in microjoules, or -1.0 when
            RAPL is unavailable.
        """
        with self._rapl_mutex:
            return self._read_energy_uj_locked()

    def _read_system_busy_us_locked(self) -> float:
        """Read system busy time from /proc/stat; caller holds _rapl_mutex."""
        try:
            with open("/proc/stat", "r") as file:
                line = file.readline()
            clk_tck = float(os.sysconf("SC_CLK_TCK"))
        except (OSError, ValueError):
            return 0.0

        return self.parse_proc_stat_busy_us(line, clk_tck)

    def read_snapshot(self) -> EnergySnapshot:
        """
        Read a combined CPU energy and system busy-time snapshot.

        Returns:
            Snapshot; `valid` is False when RAPL is unreadable.
        """
        with self._rapl_mutex:
            energy_uj = self._read_energy_uj_locked()

            snapshot = EnergySnapshot(
                energy_uj=max(0.0, energy_uj),
                busy_us=self._read_system_busy_us_locked(),
                valid=energy_uj >= 0.0,
            )

            if snapshot.valid:
                now_us = time.monotonic_ns() // 1_000
                self._rate_window.update(
                    now_us,
                    snapshot.energy_uj,
                    snapshot.busy_us,
                    RATE_WINDOW_US,
                )

            return snapshot

    def calculate_call_energy_uj(
        self,
        start: EnergySnapshot,
        end: EnergySnapshot,
        thread_cpu_delta_us: float,
    ) -> float:
        """
        Calculate the CPU energy attributed to a profiled call.

        Args:
            start: Snapshot taken at function start.
            end: Snapshot taken at function end.
            thread_cpu_delta_us: Thread CPU time consumed by the call.

        Returns:
            Attributed energy in microjoules (never negative).
        """
        if not start.valid or not end.valid:
            return 0.0

        delta_energy_uj = end.energy_uj - start.energy_uj
        delta_busy_us = end.busy_us - start.busy_us

        with self._rapl_mutex:
            rate_uj_per_busy_us = self._rate_window.rate_uj_per_busy_us

        return self.attribute_energy(
            delta_energy_uj,
            delta_busy_us,
            thread_cpu_delta_us,
            rate_uj_per_busy_us,
        )
