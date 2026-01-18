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
from typing import Callable

from poirot.utils.sysfs_reader import SysfsReader


class HwmonScanner:
    """
    Class for scanning and caching hwmon device paths.

    Provides methods for iterating over hwmon devices and caching
    paths for energy and power readings.
    """

    HWMON_BASE_PATH = "/sys/class/hwmon"

    def __init__(self) -> None:
        """Default constructor."""
        # Cached hwmon paths
        self._cached_energy_path: str = ""
        # Cached hwmon power path
        self._cached_power_path: str = ""
        # Flag indicating if paths have been searched
        self._paths_searched: bool = False

        self.search_paths()

    def iterate_devices(self, callback: Callable[[str, str], bool]) -> None:
        """
        Iterate over hwmon devices.

        Args:
            callback: Function called for each device (base_path, name).
                     Return True to stop iteration.
        """
        if not os.path.isdir(self.HWMON_BASE_PATH):
            return

        try:
            for entry in os.listdir(self.HWMON_BASE_PATH):
                # skip '.' and '..' and hidden entries
                if entry.startswith("."):
                    continue

                hwmon_path = os.path.join(self.HWMON_BASE_PATH, entry)
                if not os.path.isdir(hwmon_path):
                    continue

                name = SysfsReader.read_string(os.path.join(hwmon_path, "name"))

                if callback(hwmon_path, name):
                    break
        except OSError:
            # Ignore filesystem access errors
            pass

    def search_paths(self) -> None:
        """Search for and cache hwmon paths for energy and power readings."""
        if self._paths_searched:
            return

        # Mark as searched early to avoid reentrancy or repeated work
        self._paths_searched = True

        def find_paths(base_path: str, name: str) -> bool:
            # determine if this driver likely provides energy/power
            is_energy_driver = (
                name == "zenpower" or name == "amd_energy" or (name and "energy" in name)
            )
            is_power_driver = (
                name == "zenpower"
                or name == "amd_energy"
                or name == "coretemp"
                or name == "k10temp"
                or (name and "power" in name)
            )

            # Search for energyN_input (1..10)
            if is_energy_driver and not self._cached_energy_path:
                for i in range(1, 11):
                    energy_path = os.path.join(base_path, f"energy{i}_input")
                    try:
                        val = SysfsReader.read_long(energy_path)
                        if val >= 0:
                            self._cached_energy_path = energy_path
                            break
                    except Exception:
                        # fallback to existence check
                        if os.path.exists(energy_path):
                            self._cached_energy_path = energy_path
                            break

            # Search for powerN_input (1..10)
            if is_power_driver and not self._cached_power_path:
                for i in range(1, 11):
                    power_path = os.path.join(base_path, f"power{i}_input")
                    # prefer read_long if available, otherwise existence
                    try:
                        # If file can be read as integer, accept it
                        _ = SysfsReader.read_long(power_path)
                        self._cached_power_path = power_path
                        break
                    except Exception:
                        if os.path.exists(power_path):
                            self._cached_power_path = power_path
                            break

            # stop when both cached
            return bool(self._cached_energy_path and self._cached_power_path)

        self.iterate_devices(find_paths)

    def read_power_w(self) -> float:
        """
        Read power from cached hwmon path.

        Returns:
            Power in watts or 0.0 if not available.
        """
        if not self._cached_power_path:
            return 0.0

        # hwmon power readings are expressed in microwatts (uW)
        try:
            power_uw = SysfsReader.read_long(self._cached_power_path)
        except Exception:
            # if read_long isn't available or fails, return 0
            return 0.0

        return (power_uw / 1_000_000.0) if power_uw > 0 else 0.0

    def get_energy_path(self) -> str:
        """
        Get cached energy path.

        Returns:
            Cached energy path or empty string.
        """
        return self._cached_energy_path

    def get_power_path(self) -> str:
        """
        Get cached power path.

        Returns:
            Cached power path or empty string.
        """
        return self._cached_power_path

    def is_searched(self) -> bool:
        """
        Check if paths have been searched.

        Returns:
            True if search has been performed.
        """
        return self._paths_searched
