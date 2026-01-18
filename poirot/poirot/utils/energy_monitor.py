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

from poirot.utils.hwmon_scanner import HwmonScanner
from poirot.utils.sysfs_reader import SysfsReader


class EnergyMonitor:
    """
    Class for monitoring CPU energy consumption.

    Provides methods for reading energy from RAPL, hwmon, or estimating
    from power measurements. Maintains state for tracking accumulated energy.
    """

    def __init__(self, hwmon_scanner: HwmonScanner) -> None:
        """
        Constructor.

        Args:
            hwmon_scanner: Reference to HwmonScanner for energy readings.
        """
        self._hwmon_scanner = hwmon_scanner

        # Mutex for thread-safe energy readings
        self._energy_mutex = threading.Lock()
        # Accumulated energy in microjoules
        self._accumulated_energy_uj = 0.0
        # Last energy read time point (0 means uninitialized)
        self._last_energy_read_time = 0.0
        # Last RAPL energy value (for delta calculation)
        self._last_rapl_energy_uj = 0.0
        # Last hwmon energy value (for delta calculation)
        self._last_hwmon_energy_uj = 0.0
        # Maximum RAPL energy value before wrap-around
        self._rapl_max_energy_uj = 0.0

        # CPU TDP in watts for estimation
        self._cpu_tdp_watts = 0.0
        # Idle power factor for estimation (typical: 0.10-0.20)
        self._idle_power_factor = 0.15

        self.initialize_rapl_max_energy()

    def set_cpu_tdp_watts(self, tdp: float) -> None:
        """
        Set the CPU TDP in watts for estimation.

        Args:
            tdp: TDP value in watts.
        """
        self._cpu_tdp_watts = tdp

    def set_idle_power_factor(self, factor: float) -> None:
        """
        Set the idle power factor for estimation.

        Args:
            factor: Idle power factor (0.0 to 1.0).
        """
        self._idle_power_factor = factor

    def initialize_rapl_max_energy(self) -> None:
        """Initialize RAPL max energy range for wraparound detection."""

        # Try Intel RAPL first
        intel_max_range_path = "/sys/class/powercap/intel-rapl:0/max_energy_range_uj"
        if os.path.exists(intel_max_range_path):
            max_val = SysfsReader.read_double(intel_max_range_path)
            if max_val > 0:
                self._rapl_max_energy_uj = max_val
                return

        # Try AMD RAPL if Intel not available
        amd_max_range_path = "/sys/class/powercap/amd-rapl:0/max_energy_range_uj"
        if os.path.exists(amd_max_range_path):
            max_val = SysfsReader.read_double(amd_max_range_path)
            if max_val > 0:
                self._rapl_max_energy_uj = max_val
                return

        # Fallback: if neither found, rapl_max_energy_uj_ remains 0
        self._rapl_max_energy_uj = 0.0

    def read_energy_uj(self, cpu_percent: float = 0.0) -> float:
        """
        Read accumulated CPU energy consumption in microjoules.

        Args:
            cpu_percent: Current CPU utilization percentage for estimation.

        Returns:
            Accumulated energy consumption in microjoules.
        """
        with self._energy_mutex:
            # Helper to read RAPL-like counters with wraparound handling
            def _read_rapl_energy(path: str) -> float:
                if not os.path.exists(path):
                    return -1.0

                current = SysfsReader.read_double(path)
                if current <= 0:
                    return -1.0

                # If we have a previous measurement, accumulate delta (handle wraparound)
                if self._last_rapl_energy_uj > 0.0:
                    if current < self._last_rapl_energy_uj:
                        # Counter wrapped around
                        if self._rapl_max_energy_uj > 0.0:
                            delta = (
                                self._rapl_max_energy_uj
                                - self._last_rapl_energy_uj
                                + current
                            )
                            self._accumulated_energy_uj += delta
                    else:
                        self._accumulated_energy_uj += current - self._last_rapl_energy_uj

                # Update last reading
                self._last_rapl_energy_uj = current
                return self._accumulated_energy_uj

            # 1) Try Intel RAPL
            energy = _read_rapl_energy("/sys/class/powercap/intel-rapl:0/energy_uj")
            if energy >= 0.0:
                return energy

            # 2) Try AMD RAPL
            energy = _read_rapl_energy("/sys/class/powercap/amd-rapl:0/energy_uj")
            if energy >= 0.0:
                return energy

            # 3) Try hwmon energy path
            energy_path = self._hwmon_scanner.get_energy_path()
            if energy_path and os.path.exists(energy_path):
                current_hwmon = SysfsReader.read_double(energy_path)
                if current_hwmon > 0.0:
                    if self._last_hwmon_energy_uj > 0.0:
                        if current_hwmon >= self._last_hwmon_energy_uj:
                            # Normal case: accumulate delta
                            self._accumulated_energy_uj += (
                                current_hwmon - self._last_hwmon_energy_uj
                            )
                        # else: wraparound detected; ignore this reading

                    self._last_hwmon_energy_uj = current_hwmon
                    return self._accumulated_energy_uj

            # 4) Fallback: estimate from power measurements using elapsed time
            now = time.time()

            # If last read time is uninitialized, set it and return current accumulated
            if self._last_energy_read_time <= 0.0:
                self._last_energy_read_time = now
                return self._accumulated_energy_uj

            elapsed_s = now - self._last_energy_read_time
            # Update last read time now
            self._last_energy_read_time = now

            if elapsed_s <= 0.0:
                return self._accumulated_energy_uj

            elapsed_us = elapsed_s * 1_000_000.0

            # Try to read direct power from hwmon scanner
            power_w = 0.0
            try:
                power_w = self._hwmon_scanner.read_power_w()
            except Exception:
                power_w = 0.0

            # If no direct power measurement, estimate from CPU TDP and usage
            if power_w <= 0.0 and self._cpu_tdp_watts > 0.0:
                base_power = self._cpu_tdp_watts * self._idle_power_factor
                dynamic_power = (
                    self._cpu_tdp_watts
                    * (1.0 - self._idle_power_factor)
                    * (cpu_percent / 100.0)
                )
                power_w = base_power + dynamic_power

            if power_w > 0.0:
                # W * us = uJ
                energy_delta_uj = power_w * elapsed_us
                self._accumulated_energy_uj += energy_delta_uj

            return self._accumulated_energy_uj

    def _estimate_power_from_cpu(self, cpu_percent: float) -> float:
        """
        Estimate power consumption from CPU percentage.

        Args:
            cpu_percent: CPU utilization percentage.

        Returns:
            Estimated power in watts.
        """
        # Simple linear model: idle_power + (tdp - idle_power) * (cpu_percent / 100)
        idle_power = self._cpu_tdp_watts * self._idle_power_factor
        active_power = self._cpu_tdp_watts - idle_power
        return idle_power + active_power * (cpu_percent / 100.0)

    def calculate_thread_energy_uj(
        self,
        thread_cpu_delta_us: float,
        process_cpu_delta_us: float,
        system_cpu_delta_us: float,
        total_energy_delta_uj: float,
    ) -> float:
        """
        Calculate thread energy consumption using hierarchical attribution.

        This method calculates the energy attributed to a specific thread
        based on its CPU time usage relative to the process and system.

        Args:
            thread_cpu_delta_us: Thread CPU time delta in microseconds.
            process_cpu_delta_us: Process CPU time delta in microseconds.
            system_cpu_delta_us: System CPU time delta in microseconds.
            total_energy_delta_uj: Total energy delta in microjoules.

        Returns:
            Thread energy consumption in microjoules.
        """
        if system_cpu_delta_us <= 0 or total_energy_delta_uj <= 0:
            return 0.0

        # Calculate thread's share of process energy
        if process_cpu_delta_us > 0:
            thread_share_of_process = thread_cpu_delta_us / process_cpu_delta_us
        else:
            thread_share_of_process = 1.0

        # Calculate process's share of system energy
        process_share_of_system = process_cpu_delta_us / system_cpu_delta_us

        # Thread energy = total * process_share * thread_share
        return total_energy_delta_uj * process_share_of_system * thread_share_of_process
