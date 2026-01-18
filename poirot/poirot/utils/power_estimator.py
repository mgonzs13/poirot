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

    INTEL_RAPL_TDP_TYPE = 1
    AMD_RAPL_TDP_TYPE = 2
    HWMON_RAPL_TDP_TYPE = 3
    THERMAL_POWER_TDP_TYPE = 4
    CPU_CORES_FREQUENCY_TYPE = 5
    CPU_CORES_TYPE = 6


class PowerEstimator:
    """
    Class for estimating power consumption and TDP values.

    Provides methods for reading RAPL power limits, battery power,
    and estimating various power-related metrics.
    """

    INTEL_RAPL_PATH = "/sys/class/powercap/intel-rapl/intel-rapl:0"
    AMD_RAPL_PATH = "/sys/class/powercap/amd-rapl/amd-rapl:0"

    def __init__(self, hwmon_scanner: HwmonScanner) -> None:
        """
        Constructor.

        Args:
            hwmon_scanner: Reference to HwmonScanner for power readings.
        """
        self._hwmon_scanner = hwmon_scanner
        self._cpu_cores = os.cpu_count() or 1
        self._cpu_tdp_watts = 0.0

    def set_cpu_cores(self, cores: int) -> None:
        """
        Set the number of CPU cores.

        Args:
            cores: Number of CPU cores.
        """
        self._cpu_cores = cores

    def set_cpu_tdp_watts(self, tdp: float) -> None:
        """
        Set the CPU TDP in watts.

        Args:
            tdp: TDP value in watts.
        """
        self._cpu_tdp_watts = tdp

    def rapl_available(self) -> bool:
        """
        Check if RAPL is available.

        Returns:
            True if RAPL is available, False otherwise.
        """
        # Check for energy_uj presence (more reliable)
        intel_energy = os.path.join(self.INTEL_RAPL_PATH, "energy_uj")
        amd_energy = os.path.join(self.AMD_RAPL_PATH, "energy_uj")
        return os.path.exists(intel_energy) or os.path.exists(amd_energy)

    def read_tdp_watts(self) -> Tuple[float, TdpType]:
        """
        Read CPU TDP in watts.

        Returns:
            Tuple of (TDP in watts, TdpType).
        """
        # Try Intel RAPL
        tdp = self.read_intel_rapl_power_limit_w()
        if tdp > 0:
            return tdp, TdpType.INTEL_RAPL_TDP_TYPE

        # Try AMD RAPL
        tdp = self.read_amd_rapl_power_limit_w()
        if tdp > 0:
            return tdp, TdpType.AMD_RAPL_TDP_TYPE

        # Try hwmon RAPL
        tdp = self.read_hwmon_tdp_watts()
        if tdp > 0:
            return tdp, TdpType.HWMON_RAPL_TDP_TYPE

        # Try thermal power
        tdp = self.read_thermal_tdp_watts()
        if tdp > 0:
            return tdp, TdpType.THERMAL_POWER_TDP_TYPE

        # Estimate from CPU frequency
        tdp = self.estimate_frequency_tdp_watts()
        if tdp > 0:
            return tdp, TdpType.CPU_CORES_FREQUENCY_TYPE

        # Estimate from cores
        tdp = self.estimate_cores_tdp_watts()
        return tdp, TdpType.CPU_CORES_TYPE

    def read_rapl_power_limit_w(self) -> float:
        """
        Read RAPL power limit in watts.

        Returns:
            Power limit in watts or 0.0 if not available.
        """
        result = self.read_intel_rapl_power_limit_w()
        if result > 0:
            return result
        return self.read_amd_rapl_power_limit_w()

    def read_intel_rapl_power_limit_w(self) -> float:
        """
        Read Intel RAPL power limit in watts.

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

    def read_amd_rapl_power_limit_w(self) -> float:
        """
        Read AMD RAPL power limit in watts.

        Returns:
            Power limit in watts or 0.0 if not available.
        """
        amd_rapl_paths = [
            f"{self.AMD_RAPL_PATH}/constraint_0_power_limit_uw",
        ]

        for path in amd_rapl_paths:
            power_uw = SysfsReader.read_long(path)
            if power_uw > 0:
                return power_uw / 1_000_000.0

        return 0.0

    def read_battery_power_w(self) -> float:
        """
        Read battery power in watts.

        Returns:
            Battery power in watts or -1.0 if not available.
        """
        power_supply_path = "/sys/class/power_supply"

        if not os.path.isdir(power_supply_path):
            return -1.0

        try:
            for entry in os.listdir(power_supply_path):
                supply_path = os.path.join(power_supply_path, entry)

                # Prefer entries named like BAT*
                if entry.startswith("BAT"):
                    power_path = os.path.join(supply_path, "power_now")
                    power_uw = SysfsReader.read_long(power_path)
                    if power_uw > 0:
                        return power_uw / 1_000_000.0

                    voltage_path = os.path.join(supply_path, "voltage_now")
                    current_path = os.path.join(supply_path, "current_now")
                    voltage_uv = SysfsReader.read_long(voltage_path)
                    current_ua = SysfsReader.read_long(current_path)
                    if voltage_uv > 0 and current_ua > 0:
                        return (voltage_uv * current_ua) / 1_000_000_000_000.0

                # Fallback: check type
                type_path = os.path.join(supply_path, "type")
                supply_type = SysfsReader.read_string(type_path)
                if supply_type == "Battery":
                    power_path = os.path.join(supply_path, "power_now")
                    power_uw = SysfsReader.read_long(power_path)
                    if power_uw > 0:
                        return power_uw / 1_000_000.0

                    voltage_path = os.path.join(supply_path, "voltage_now")
                    current_path = os.path.join(supply_path, "current_now")
                    voltage_uv = SysfsReader.read_long(voltage_path)
                    current_ua = SysfsReader.read_long(current_path)
                    if voltage_uv > 0 and current_ua > 0:
                        return (voltage_uv * current_ua) / 1_000_000_000_000.0
        except OSError:
            pass

        return -1.0

    def read_min_tdp_watts(self) -> float:
        """
        Read minimum TDP in watts.

        Returns:
            Minimum TDP in watts.
        """
        min_power_paths = [
            f"{self.INTEL_RAPL_PATH}/constraint_0_min_power_uw",
            f"{self.INTEL_RAPL_PATH}/constraint_1_min_power_uw",
            f"{self.AMD_RAPL_PATH}/constraint_0_min_power_uw",
        ]

        for path in min_power_paths:
            power_uw = SysfsReader.read_long(path)
            if power_uw > 0:
                return power_uw / 1_000_000.0

        # Estimate from minimum CPU frequency
        min_freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq"
        )
        if min_freq_khz > 0 and self._cpu_cores > 0:
            min_freq_ghz = min_freq_khz / 1_000_000.0
            watts_per_ghz = self.read_watts_per_ghz()
            min_tdp = self._cpu_cores * min_freq_ghz * watts_per_ghz
            if min_tdp > 0:
                return min_tdp

        return FALLBACK_MIN_TDP_WATTS

    def read_max_tdp_watts(self) -> float:
        """
        Read maximum TDP in watts.

        Returns:
            Maximum TDP in watts.
        """
        max_power_paths = [
            f"{self.INTEL_RAPL_PATH}/constraint_0_max_power_uw",
            f"{self.INTEL_RAPL_PATH}/constraint_1_max_power_uw",
            f"{self.AMD_RAPL_PATH}/constraint_0_max_power_uw",
        ]

        for path in max_power_paths:
            power_uw = SysfsReader.read_long(path)
            if power_uw > 0:
                return power_uw / 1_000_000.0

        # Try hwmon power cap
        max_tdp_w = 0.0

        def find_max_power_cap(base_path: str, name: str) -> bool:
            nonlocal max_tdp_w
            if name in ("zenpower", "amd_energy", "coretemp", "k10temp"):
                power_cap_path = os.path.join(base_path, "power1_cap_max")
                power_uw = SysfsReader.read_long(power_cap_path)
                if power_uw > 0:
                    max_tdp_w = power_uw / 1_000_000.0
                    return True
            return False

        self._hwmon_scanner.iterate_devices(find_max_power_cap)
        if max_tdp_w > 0:
            return max_tdp_w

        # Estimate from max CPU frequency
        max_freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
        )
        if max_freq_khz > 0 and self._cpu_cores > 0:
            max_freq_ghz = max_freq_khz / 1_000_000.0
            watts_per_ghz = self.read_watts_per_ghz()
            max_tdp = self._cpu_cores * max_freq_ghz * watts_per_ghz
            if max_tdp > 0:
                return max_tdp * 1.5

        return FALLBACK_MAX_TDP_WATTS

    def read_idle_power_factor(self) -> float:
        """
        Read idle power factor.

        Returns:
            Idle power factor (0.0 to 1.0).
        """
        idle_power_w = 0.0
        max_power_w = self.read_max_tdp_watts()

        # Try an idle constraint if available
        idle_uw = SysfsReader.read_long(
            f"{self.INTEL_RAPL_PATH}/constraint_1_power_limit_uw"
        )
        if idle_uw > 0:
            idle_power_w = idle_uw / 1_000_000.0

        # Try sustainable_power from thermal zone
        if idle_power_w == 0.0:
            sustainable_mw = SysfsReader.read_long(
                "/sys/class/thermal/thermal_zone0/sustainable_power"
            )
            if sustainable_mw > 0:
                idle_power_w = sustainable_mw / 1000.0

        # If still not found, use frequency ratio heuristic
        if idle_power_w == 0.0 or max_power_w <= 0.0:
            min_freq_khz = SysfsReader.read_long(
                "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq"
            )
            max_freq_khz = SysfsReader.read_long(
                "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
            )
            if min_freq_khz > 0 and max_freq_khz > 0:
                freq_ratio = float(min_freq_khz) / float(max_freq_khz)
                return freq_ratio * freq_ratio

        if idle_power_w > 0.0 and max_power_w > 0.0:
            factor = idle_power_w / max_power_w
            return max(0.05, min(0.5, factor))

        return FALLBACK_IDLE_POWER_FACTOR

    def read_watts_per_ghz(self) -> float:
        """
        Read watts per GHz per core.

        Returns:
            Watts per GHz per core.
        """
        # Prefer current hwmon power reading
        current_power_w = self._hwmon_scanner.read_power_w()
        if current_power_w == 0.0:
            # fallback to rapl power limit if hwmon doesn't provide current power
            current_power_w = self.read_rapl_power_limit_w()

        # current frequency
        freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        )
        current_freq_ghz = freq_khz / 1_000_000.0 if freq_khz > 0 else 0.0

        if current_power_w > 0 and current_freq_ghz > 0 and self._cpu_cores > 0:
            return current_power_w / (self._cpu_cores * current_freq_ghz)

        # Fallback: use CPU TDP and max frequency
        if self._cpu_tdp_watts > 0:
            max_freq_khz = SysfsReader.read_long(
                "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
            )
            if max_freq_khz > 0 and self._cpu_cores > 0:
                max_freq_ghz = max_freq_khz / 1_000_000.0
                return self._cpu_tdp_watts / (self._cpu_cores * max_freq_ghz)

        return FALLBACK_WATTS_PER_GHZ

    def read_min_watts_per_ghz(self) -> float:
        """
        Read minimum watts per GHz.

        Returns:
            Minimum watts per GHz.
        """
        min_tdp = self.read_min_tdp_watts()
        max_freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
        )

        if min_tdp > 0 and max_freq_khz > 0 and self._cpu_cores > 0:
            max_freq_ghz = max_freq_khz / 1_000_000.0
            min_watts_per_ghz = min_tdp / (self._cpu_cores * max_freq_ghz)
            if min_watts_per_ghz > 0:
                return min_watts_per_ghz

        watts_per_ghz = self.read_watts_per_ghz()
        idle_factor = self.read_idle_power_factor()
        if watts_per_ghz > 0 and idle_factor > 0:
            return watts_per_ghz * idle_factor

        return FALLBACK_MIN_WATTS_PER_GHZ

    def read_max_watts_per_ghz(self) -> float:
        """
        Read maximum watts per GHz.

        Returns:
            Maximum watts per GHz.
        """
        max_tdp = self.read_max_tdp_watts()
        base_freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/base_frequency"
        )

        if base_freq_khz <= 0:
            base_freq_khz = SysfsReader.read_long(
                "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq"
            )

        if max_tdp > 0 and base_freq_khz > 0 and self._cpu_cores > 0:
            base_freq_ghz = base_freq_khz / 1_000_000.0
            max_watts_per_ghz = max_tdp / (self._cpu_cores * base_freq_ghz)
            if max_watts_per_ghz > 0:
                return max_watts_per_ghz

        watts_per_ghz = self.read_watts_per_ghz()
        if watts_per_ghz > 0:
            return watts_per_ghz * 2.0

        return FALLBACK_MAX_WATTS_PER_GHZ

    def estimate_power_per_core_per_ghz(self) -> float:
        """
        Estimate power per core per GHz.

        Returns:
            Estimated power per core per GHz.
        """
        current_power_w = self._hwmon_scanner.read_power_w()

        if current_power_w == 0.0:
            current_power_w = self.read_rapl_power_limit_w()

        freq_khz = SysfsReader.read_long(
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
        )
        current_freq_ghz = freq_khz / 1_000_000.0 if freq_khz > 0 else 0.0

        if current_power_w > 0 and current_freq_ghz > 0 and self._cpu_cores > 0:
            calculated = current_power_w / (self._cpu_cores * current_freq_ghz)
            min_watts_per_ghz = self.read_min_watts_per_ghz()
            max_watts_per_ghz = self.read_max_watts_per_ghz()
            if calculated >= min_watts_per_ghz and calculated <= max_watts_per_ghz:
                return calculated

        max_speed_mhz = SysfsReader.read_long("/sys/class/dmi/id/processor_max_speed")
        if max_speed_mhz > 0:
            max_freq_ghz = max_speed_mhz / 1000.0
            if self._cpu_tdp_watts > 0 and self._cpu_cores > 0:
                calculated = self._cpu_tdp_watts / (self._cpu_cores * max_freq_ghz)
                return calculated

        return FALLBACK_POWER_PER_CORE_PER_GHZ

    def estimate_watts_per_core(self) -> float:
        """
        Estimate watts per core.

        Returns:
            Estimated watts per core.
        """
        total_power_w = self._hwmon_scanner.read_power_w()

        if total_power_w == 0.0:
            battery_power = self.read_battery_power_w()
            if battery_power > 0:
                total_power_w = battery_power

        if total_power_w == 0.0:
            total_power_w = self.read_rapl_power_limit_w()

        if total_power_w == 0.0:
            power_mw = SysfsReader.read_long(
                "/sys/class/thermal/thermal_zone0/sustainable_power"
            )
            if power_mw > 0:
                total_power_w = power_mw / 1000.0

        if total_power_w == 0.0 and self._cpu_tdp_watts > 0:
            total_power_w = self._cpu_tdp_watts

        if total_power_w > 0 and self._cpu_cores > 0:
            return total_power_w / self._cpu_cores

        return FALLBACK_WATTS_PER_CORE

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
                            "/sys/class/thermal/thermal_zone0/sustainable_power"
                        )
                        if power_mw > 0:
                            tdp_watts = power_mw / 1000.0

                        # Try cooling device power
                        if tdp_watts == 0.0:
                            power = SysfsReader.read_long(os.path.join(base, "power"))
                            if power > 0:
                                tdp_watts = power / 1000.0

                        # Try ACPI max power
                        if tdp_watts == 0.0:
                            power_uw = SysfsReader.read_long(
                                f"{self.INTEL_RAPL_PATH}/constraint_0_max_power_uw"
                            )
                            if power_uw > 0:
                                tdp_watts = power_uw / 1_000_000.0

                        # Derive from CPU frequency
                        if tdp_watts == 0.0 and self._cpu_cores > 0:
                            max_freq_khz = SysfsReader.read_long(
                                "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
                            )
                            if max_freq_khz > 0:
                                max_freq_ghz = max_freq_khz / 1_000_000.0
                                watts_per_ghz = self.read_watts_per_ghz()
                                watts_per_core_at_max = max_freq_ghz * watts_per_ghz
                                tdp_watts = self._cpu_cores * watts_per_core_at_max

                        break
        except OSError:
            pass

        return tdp_watts

    def estimate_frequency_tdp_watts(self) -> float:
        """
        Estimate TDP from CPU frequency and core count.

        Returns:
            TDP in watts or 0.0 if not available.
        """
        path = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
        max_freq_khz = SysfsReader.read_long(path)
        if max_freq_khz > 0:
            max_freq_ghz = max_freq_khz / 1_000_000.0
            return self._cpu_cores * max_freq_ghz * FALLBACK_POWER_PER_CORE_PER_GHZ

        return 0.0

    def estimate_cores_tdp_watts(self) -> float:
        """
        Estimate TDP from core count alone.

        Returns:
            Estimated TDP in watts.
        """
        return self._cpu_cores * FALLBACK_WATTS_PER_CORE
