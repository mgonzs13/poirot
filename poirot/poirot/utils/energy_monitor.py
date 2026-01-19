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
from enum import IntEnum
from typing import Union

from poirot.utils.hwmon_scanner import HwmonScanner
from poirot.utils.sysfs_reader import SysfsReader


class EnergyType(IntEnum):
    """Energy detection type constants."""

    ENERGY_TYPE_RAPL = 1
    ENERGY_TYPE_HWMON = 2
    ENERGY_TYPE_HWMON_ESTIMATED = 3
    ENERGY_TYPE_ESTIMATED = 4


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
        # Last RAPL energy value (for delta calculation)
        self._last_rapl_energy_uj = 0.0
        # Last hwmon energy value (for delta calculation)
        self._last_hwmon_energy_uj = 0.0
        # Maximum RAPL energy value before wrap-around
        self._rapl_max_energy_uj = 0.0

        # CPU TDP in watts for estimation
        self._cpu_tdp_watts = 0.0

        self.initialize_rapl_max_energy()

    def set_cpu_tdp_watts(self, tdp: float) -> None:
        """
        Set the CPU TDP in watts for estimation.

        Args:
            tdp: TDP value in watts.
        """
        self._cpu_tdp_watts = tdp

    def initialize_rapl_max_energy(self) -> None:
        """Initialize RAPL max energy range for wraparound detection."""

        intel_max_range_path = "/sys/class/powercap/intel-rapl:0/max_energy_range_uj"
        if os.path.exists(intel_max_range_path):
            max_val = SysfsReader.read_double(intel_max_range_path)
            if max_val > 0:
                self._rapl_max_energy_uj = max_val
                return

        # Fallback: if neither found, rapl_max_energy_uj_ remains 0
        self._rapl_max_energy_uj = 0.0

    def read_energy_uj(self, elapsed_us: float = 0.0) -> Union[float, EnergyType]:
        """
        Read accumulated CPU energy consumption in microjoules.

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
                return energy, EnergyType.ENERGY_TYPE_RAPL

            # 2) Try hwmon energy path
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
                    return self._accumulated_energy_uj, EnergyType.ENERGY_TYPE_HWMON

            # 3) Fallback: estimate from power measurements using elapsed time
            power_w = 0.0
            energy_type = EnergyType.ENERGY_TYPE_HWMON_ESTIMATED
            try:
                power_w = self._hwmon_scanner.read_power_w()
                energy_type = EnergyType.ENERGY_TYPE_HWMON_ESTIMATED
            except Exception:
                power_w = 0.0

            # If no direct power measurement, estimate from CPU TDP and usage
            if power_w <= 0.0 and self._cpu_tdp_watts > 0.0:
                energy_type = EnergyType.ENERGY_TYPE_ESTIMATED
                power_w = self._cpu_tdp_watts

            if power_w > 0.0:
                # W * us = uJ
                energy_delta_uj = power_w * elapsed_us
                self._accumulated_energy_uj += energy_delta_uj

            return self._accumulated_energy_uj, energy_type

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
        # If no energy delta or no thread CPU time, return 0
        if total_energy_delta_uj <= 0.0 or thread_cpu_delta_us <= 0.0:
            return 0.0

        # Calculate thread's share of system energy using hierarchical attribution
        thread_share = 0.0

        if system_cpu_delta_us > 0.0 and process_cpu_delta_us > 0.0:
            # Hierarchical attribution:
            # thread_share = (thread_cpu / process_cpu) * (process_cpu / system_cpu)
            #              = thread_cpu / system_cpu
            # But we use the hierarchical form to be more accurate when there are
            # multiple processes and threads
            thread_process_share = thread_cpu_delta_us / process_cpu_delta_us
            process_system_share = process_cpu_delta_us / system_cpu_delta_us
            thread_share = thread_process_share * process_system_share
        elif process_cpu_delta_us > 0.0:
            # Fallback: attribute based on thread's share of process time
            thread_share = thread_cpu_delta_us / process_cpu_delta_us
        else:
            # Last resort: assume thread gets all the energy
            thread_share = 1.0

        # Clamp share to valid range [0, 1]
        thread_share = max(0.0, min(1.0, thread_share))

        return total_energy_delta_uj * thread_share
