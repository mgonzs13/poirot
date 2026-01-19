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
from enum import IntEnum
from typing import Tuple

from poirot.utils.hwmon_scanner import HwmonScanner
from poirot.utils.sysfs_reader import SysfsReader

# Fallback minimum TDP in watts if system detection fails
FALLBACK_MIN_TDP_WATTS = 15.0
# Fallback maximum TDP in watts if system detection fails
FALLBACK_MAX_TDP_WATTS = 400.0
# Fallback idle power factor if measurement fails
FALLBACK_IDLE_POWER_FACTOR = 0.15
# Fallback watts per core per GHz if calculation fails
FALLBACK_WATTS_PER_GHZ = 4.0
# Fallback minimum watts per GHz for validation
FALLBACK_MIN_WATTS_PER_GHZ = 2.0
# Fallback maximum watts per GHz for validation
FALLBACK_MAX_WATTS_PER_GHZ = 20.0
# Fallback power per core per GHz if all measurements fail
FALLBACK_POWER_PER_CORE_PER_GHZ = 10.0
# Fallback watts per core if all measurements fail
FALLBACK_WATTS_PER_CORE = 12.0


class TdpType(IntEnum):
    """CPU TDP detection type constants."""

    RAPL_TDP_TYPE = 1
    HWMON_TDP_TYPE = 2
    THERMAL_POWER_TDP_TYPE = 3


class PowerEstimator:
    """
    Class for estimating power consumption and TDP values.

    Provides methods for reading RAPL power limits, battery power,
    and estimating various power-related metrics.
    """

    INTEL_RAPL_PATH = "/sys/class/powercap/intel-rapl/intel-rapl:0"

    def __init__(self, hwmon_scanner: HwmonScanner) -> None:
        """
        Constructor.

        Args:
            hwmon_scanner: Reference to HwmonScanner for power readings.
        """
        self._hwmon_scanner = hwmon_scanner

    def read_tdp_watts(self) -> Tuple[float, TdpType]:
        """
        Read CPU TDP in watts.

        Returns:
            Tuple of (TDP in watts, TdpType).
        """
        tdp_watts = 0.0
        tdp_type = TdpType.RAPL_TDP_TYPE

        # Try RAPL
        if tdp_watts == 0:
            tdp_watts = self.read_rapl_power_limit_w()
            tdp_type = TdpType.RAPL_TDP_TYPE

        # Try hwmon TDP
        if tdp_watts == 0:
            tdp_watts = self.read_hwmon_tdp_watts()
            tdp_type = TdpType.HWMON_TDP_TYPE

        # Try thermal power
        if tdp_watts == 0:
            tdp_watts = self.read_thermal_tdp_watts()
            tdp_type = TdpType.THERMAL_POWER_TDP_TYPE

        return tdp_watts, tdp_type

    def read_rapl_power_limit_w(self) -> float:
        """
        Read RAPL power limit in watts.

        Returns:
            Power limit in watts or 0.0 if not available.
        """

        # Try both constraints
        intel_rapl_paths = [
            f"{self.INTEL_RAPL_PATH}/constraint_0_power_limit_uw",
            f"{self.INTEL_RAPL_PATH}/constraint_1_power_limit_uw",
        ]

        for path in intel_rapl_paths:
            power_uw = SysfsReader.read_long(path)
            if power_uw > 0:
                return power_uw / 1_000_000.0

        return 0.0

    def read_hwmon_tdp_watts(self) -> float:
        """
        Read hwmon TDP from power limit files.

        Returns:
            TDP in watts or 0.0 if not available.
        """
        # Prefer hwmon scanner reported power
        return self._hwmon_scanner.read_power_w()

    def read_thermal_tdp_watts(self) -> float:
        """
        Read thermal zone power budget TDP.

        Returns:
            TDP in watts or 0.0 if not available.
        """
        tdp_watts = 0.0

        thermal_base = "/sys/class/thermal"
        try:
            for entry in os.listdir(thermal_base):

                if not entry.startswith("cooling_device"):
                    continue

                base = os.path.join(thermal_base, entry)
                type_str = SysfsReader.read_string(os.path.join(base, "type"))

                if ("Processor" in type_str) or ("processor" in type_str):
                    max_pstate = SysfsReader.read_long(os.path.join(base, "max_state"))
                    if max_pstate > 0:
                        # Try sustainable power from thermal zone
                        power_mw = SysfsReader.read_long(
                            f"{thermal_base}/thermal_zone0/sustainable_power"
                        )
                        if power_mw > 0:
                            tdp_watts = power_mw / 1000.0

                        # Try cooling device power
                        if tdp_watts == 0.0:
                            power = SysfsReader.read_long(os.path.join(base, "power"))
                            if power > 0:
                                tdp_watts = power / 1000.0

        except OSError:
            pass

        return tdp_watts
