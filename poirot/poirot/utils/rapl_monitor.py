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
import time
import threading

from poirot.utils.sysfs_reader import SysfsReader

RAPL_PATH = "/sys/class/powercap/intel-rapl"


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
        # Wall-clock time of the first successful RAPL read
        self._start_time: float = 0.0

        self._load_rapl_package_path()
        self._initialize_rapl_max_energy()

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

    def read_energy_uj(self) -> float:
        """
        Read accumulated CPU energy consumption in microjoules.

        Returns:
            Accumulated energy consumption in microjoules.
        """
        with self._rapl_mutex:
            if not self._rapl_package_path:
                return 0.0

            energy_path = f"{self._rapl_package_path}/energy_uj"
            if not os.path.exists(energy_path):
                return 0.0

            current = SysfsReader.read_double(energy_path)
            if current <= 0:
                return 0.0

            # Record wall-clock time on the very first successful RAPL read.
            # This is used to compute the long-running average power.
            if self._start_time == 0.0:
                self._start_time = time.monotonic()

            # If we have a previous measurement, accumulate delta (handle wraparound)
            if self._last_rapl_energy_uj > 0.0:
                if current < self._last_rapl_energy_uj:
                    # Counter wrapped around
                    if self._rapl_max_energy_uj > 0.0:
                        delta = (
                            self._rapl_max_energy_uj - self._last_rapl_energy_uj + current
                        )
                        self._accumulated_energy_uj += delta
                    else:
                        self._accumulated_energy_uj += current - self._last_rapl_energy_uj

                else:
                    # Normal case
                    delta = current - self._last_rapl_energy_uj
                    self._accumulated_energy_uj += delta

            # Update last reading
            self._last_rapl_energy_uj = current
            return self._accumulated_energy_uj

    def get_average_power_uj_per_us(self) -> float:
        """
        Get the long-running average RAPL power in uJ/us (= Watts).

        Computes average power as total accumulated energy divided by total
        elapsed wall time since the first RAPL read. Using this stable baseline
        instead of a per-window RAPL delta guarantees that parent function
        energy is always >= child function energy, because thread CPU time is a
        cumulative monotonic counter that includes all nested calls.

        Returns:
            Average power in uJ/us, or 0.0 if unavailable.
        """
        with self._rapl_mutex:
            if self._start_time == 0.0 or self._accumulated_energy_uj <= 0.0:
                return 0.0
            elapsed_us = (time.monotonic() - self._start_time) * 1_000_000.0
            if elapsed_us <= 0.0:
                return 0.0
            return self._accumulated_energy_uj / elapsed_us

    def calculate_thread_energy_uj(
        self,
        thread_cpu_delta_us: float,
    ) -> float:
        """
        Calculate thread energy consumption using average-power attribution.

        Energy = avg_power x thread_cpu_time

        The long-running average RAPL power is multiplied by the thread's own
        CPU time delta (CLOCK_THREAD_CPUTIME_ID). Because that clock is a
        cumulative monotonic counter, a parent function's delta always
        encompasses any nested child's delta, so energy_parent >= energy_child
        is guaranteed by definition — no child-tracking or clamping is needed.

        Args:
            thread_cpu_delta_us: Thread CPU time delta in microseconds.

        Returns:
            Thread energy consumption in microjoules.
        """
        if thread_cpu_delta_us <= 0.0:
            return 0.0

        avg_power_uj_per_us = self.get_average_power_uj_per_us()
        if avg_power_uj_per_us <= 0.0:
            return 0.0

        return avg_power_uj_per_us * thread_cpu_delta_us
