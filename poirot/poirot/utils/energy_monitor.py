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

    INTEL_RAPL_ENERGY_PATH = "/sys/class/powercap/intel-rapl/intel-rapl:0/energy_uj"
    INTEL_RAPL_MAX_ENERGY_PATH = (
        "/sys/class/powercap/intel-rapl/intel-rapl:0/max_energy_range_uj"
    )

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
        # Last energy read time point
        self._last_energy_read_time = time.time()
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
        self._rapl_max_energy_uj = SysfsReader.read_double(
            self.INTEL_RAPL_MAX_ENERGY_PATH
        )
        if self._rapl_max_energy_uj <= 0:
            # Default to a large value (about 65536 Joules)
            self._rapl_max_energy_uj = 65536.0 * 1_000_000.0

        # Initialize last energy reading
        self._last_rapl_energy_uj = SysfsReader.read_double(self.INTEL_RAPL_ENERGY_PATH)

    def read_energy_uj(self, cpu_percent: float = 0.0) -> float:
        """
        Read accumulated CPU energy consumption in microjoules.

        Args:
            cpu_percent: Current CPU utilization percentage for estimation.

        Returns:
            Accumulated energy consumption in microjoules.
        """
        with self._energy_mutex:
            current_time = time.time()
            elapsed_s = current_time - self._last_energy_read_time

            if elapsed_s <= 0:
                return self._accumulated_energy_uj

            # Try RAPL first
            current_rapl_uj = SysfsReader.read_double(self.INTEL_RAPL_ENERGY_PATH)
            if current_rapl_uj > 0:
                # Handle wraparound
                if current_rapl_uj < self._last_rapl_energy_uj:
                    delta_uj = (
                        self._rapl_max_energy_uj
                        - self._last_rapl_energy_uj
                        + current_rapl_uj
                    )
                else:
                    delta_uj = current_rapl_uj - self._last_rapl_energy_uj

                self._accumulated_energy_uj += delta_uj
                self._last_rapl_energy_uj = current_rapl_uj
            else:
                # Try hwmon energy path
                energy_path = self._hwmon_scanner.get_energy_path()
                if energy_path:
                    current_energy_uj = SysfsReader.read_double(energy_path)
                    if current_energy_uj > 0:
                        self._accumulated_energy_uj = current_energy_uj
                else:
                    # Estimate from power and CPU usage
                    estimated_power_w = self._estimate_power_from_cpu(cpu_percent)
                    energy_delta_uj = estimated_power_w * elapsed_s * 1_000_000.0
                    self._accumulated_energy_uj += energy_delta_uj

            self._last_energy_read_time = current_time
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
