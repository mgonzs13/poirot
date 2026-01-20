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
        # RAPL package domain
        self._rapl_package = None
        # Accumulated energy in microjoules
        self._accumulated_energy_uj = 0.0
        # Last RAPL energy value (for delta calculation)
        self._last_rapl_energy_uj = 0.0
        # Maximum RAPL energy value before wrap-around
        self._rapl_max_energy_uj = 0.0

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
            energy_path = f"{self._rapl_package_path}/energy_uj"
            if not os.path.exists(energy_path):
                return 0.0

            current = SysfsReader.read_double(energy_path)
            if current <= 0:
                return 0.0

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
