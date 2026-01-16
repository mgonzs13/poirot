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
import shutil
import time
import threading
import subprocess
from dataclasses import dataclass
from enum import IntEnum, auto

from poirot.utils.sysfs_reader import SysfsReader


# Fallback GPU TDP in watts if detection fails
FALLBACK_GPU_TDP_WATTS = 150.0
# Fallback idle power factor for GPU
FALLBACK_GPU_IDLE_POWER_FACTOR = 0.10


class GpuTdpType(IntEnum):
    """GPU TDP detection type constants."""

    NVIDIA_SMI_TDP_TYPE = 1
    AMD_ROCM_TDP_TYPE = 2
    SYSFS_TDP_TYPE = 3
    ESTIMATED_TDP_TYPE = 4


class GpuVendor(IntEnum):
    """GPU vendor type for optimized metric reading."""

    NONE = 0
    NVIDIA = 1
    AMD = 2
    INTEL = 3


@dataclass
class GpuMetrics:
    """Structure to hold current GPU metrics."""

    utilization_percent: float = 0.0
    mem_used_kb: int = 0
    power_w: float = 0.0
    energy_uj: float = 0.0


@dataclass
class ProcessGpuMetrics:
    """Structure to hold per-process GPU metrics."""

    is_using_gpu: bool = False
    mem_used_kb: int = 0
    estimated_utilization_percent: float = 0.0
    estimated_power_w: float = 0.0


@dataclass
class GpuInfo:
    """Structure to hold static GPU information."""

    model: str = ""
    vendor: str = ""
    index: int = 0
    mem_total_kb: int = 0
    tdp_watts: float = 0.0
    tdp_type: GpuTdpType = GpuTdpType.ESTIMATED_TDP_TYPE
    available: bool = False
    power_monitoring: bool = False


class GpuMonitor:
    """
    Class for monitoring GPU metrics and energy consumption.

    Supports NVIDIA GPUs via nvidia-smi, AMD GPUs via ROCm/sysfs,
    and Intel GPUs via sysfs. Provides methods for reading GPU
    utilization, memory usage, power, and energy.
    """

    def __init__(self) -> None:
        """Default constructor."""
        # GPU information structure
        self._gpu_info = GpuInfo()

        # GPU vendor type for optimized metric reading
        self._gpu_vendor = GpuVendor.NONE

        # Mutex for thread-safe energy readings
        self._energy_mutex = threading.RLock()
        # Accumulated GPU energy in microjoules (system-wide)
        self._accumulated_energy_uj = 0.0
        # Accumulated per-process GPU energy in microjoules
        self._accumulated_process_energy_uj = 0.0
        # Last energy read time point
        self._last_energy_read_time = time.time()
        # Last per-process energy read time point
        self._last_process_energy_read_time = time.time()
        # Idle power factor for estimation
        self._idle_power_factor = FALLBACK_GPU_IDLE_POWER_FACTOR

        # AMD GPU sysfs paths
        self._amd_gpu_busy_path = ""
        self._amd_mem_busy_path = ""
        self._amd_power_path = ""
        self._amd_temp_path = ""
        self._amd_vram_used_path = ""
        self._amd_vram_total_path = ""

        # Intel GPU sysfs paths
        self._intel_freq_path = ""
        self._intel_power_path = ""

    def initialize(self) -> bool:
        """
        Initialize GPU monitoring and detect available GPUs.

        Returns:
            True if GPU was detected and initialized.
        """
        # Try NVIDIA first
        if self._detect_nvidia_gpu():
            return True

        # Try AMD
        if self._detect_amd_gpu():
            return True

        # Try Intel
        if self._detect_intel_gpu():
            return True

        return False

    def is_available(self) -> bool:
        """
        Check if GPU monitoring is available.

        Returns:
            True if GPU is available for monitoring.
        """
        return self._gpu_info.available

    def get_gpu_info(self) -> GpuInfo:
        """
        Get GPU information.

        Returns:
            Const reference to GpuInfo structure.
        """
        return self._gpu_info

    def read_metrics(self) -> GpuMetrics:
        """
        Read current GPU metrics.

        Returns:
            GpuMetrics structure with current values.
        """
        if not self._gpu_info.available:
            return GpuMetrics()

        if self._gpu_vendor == GpuVendor.NVIDIA:
            return self._read_nvidia_metrics()
        elif self._gpu_vendor == GpuVendor.AMD:
            return self._read_amd_metrics()
        elif self._gpu_vendor == GpuVendor.INTEL:
            return self._read_intel_metrics()

        return GpuMetrics()

    def read_process_metrics(self, pid: int = 0) -> ProcessGpuMetrics:
        """
        Read per-process GPU metrics for a specific PID.

        Args:
            pid: Process ID to query (default: current process).

        Returns:
            ProcessGpuMetrics structure with current values.
        """
        if pid == 0:
            pid = os.getpid()

        if not self._gpu_info.available:
            return ProcessGpuMetrics()

        if self._gpu_vendor == GpuVendor.NVIDIA:
            return self._read_nvidia_process_metrics(pid)
        elif self._gpu_vendor == GpuVendor.AMD:
            return self._read_amd_process_metrics(pid)
        elif self._gpu_vendor == GpuVendor.INTEL:
            return self._read_intel_process_metrics(pid)

        return ProcessGpuMetrics()

    def is_process_using_gpu(self, pid: int = 0) -> bool:
        """
        Check if a specific process is using the GPU.

        Args:
            pid: Process ID to check (default: current process).

        Returns:
            True if the process is using GPU compute.
        """
        metrics = self.read_process_metrics(pid)
        return metrics.is_using_gpu

    def read_process_energy_uj(self, pid: int = 0) -> float:
        """
        Read per-process accumulated GPU energy in microjoules.

        This method calculates the energy consumption attributed to a specific
        process based on its GPU memory usage ratio compared to system-wide
        GPU usage. Uses time-based integration of power measurements.

        Args:
            pid: Process ID to query (default: current process).

        Returns:
            Accumulated energy attributed to the process in microjoules.
        """
        if pid == 0:
            pid = os.getpid()

        process_metrics = self.read_process_metrics(pid)
        if not process_metrics.is_using_gpu:
            return 0.0

        with self._energy_mutex:
            current_time = time.time()
            elapsed_s = current_time - self._last_process_energy_read_time

            if elapsed_s <= 0:
                return self._accumulated_process_energy_uj

            # Get current GPU metrics for power calculation
            gpu_metrics = self.read_metrics()

            # Calculate process's share based on memory usage
            share = 0.0
            if self._gpu_info.mem_total_kb > 0:
                share = process_metrics.mem_used_kb / self._gpu_info.mem_total_kb

            # Calculate energy using power and share
            if gpu_metrics.power_w > 0:
                energy_delta_uj = gpu_metrics.power_w * share * elapsed_s * 1_000_000.0
            else:
                # Estimate power from utilization
                estimated_power = self._estimate_energy_uj(
                    0.0, process_metrics.estimated_utilization_percent
                )
                energy_delta_uj = estimated_power * elapsed_s

            self._accumulated_process_energy_uj += energy_delta_uj
            self._last_process_energy_read_time = current_time

            return self._accumulated_process_energy_uj

    def set_idle_power_factor(self, factor: float) -> None:
        """
        Set idle power factor for estimation.

        Args:
            factor: Idle power factor (0.0 to 1.0).
        """
        self._idle_power_factor = factor

    def _detect_nvidia_gpu(self) -> bool:
        """
        Detect NVIDIA GPU using nvidia-smi.

        Returns:
            True if NVIDIA GPU was detected.
        """
        # Check if nvidia-smi is available (like C++ does with 'which')
        if not shutil.which("nvidia-smi"):
            return False

        try:
            # Get GPU name
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=name",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode != 0 or not result.stdout.strip():
                return False

            self._gpu_info.model = result.stdout.strip()
            self._gpu_info.vendor = "NVIDIA"
            self._gpu_info.index = 0
            self._gpu_info.available = True

            # Get total memory
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.total",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0 and result.stdout.strip():
                try:
                    # nvidia-smi reports memory in MiB
                    self._gpu_info.mem_total_kb = int(float(result.stdout.strip()) * 1024)
                except ValueError:
                    self._gpu_info.mem_total_kb = 0

            # Get power limit (TDP)
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=power.limit",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0 and result.stdout.strip():
                power_str = result.stdout.strip()
                if "[N/A]" not in power_str:
                    try:
                        self._gpu_info.tdp_watts = float(power_str)
                        self._gpu_info.tdp_type = GpuTdpType.NVIDIA_SMI_TDP_TYPE
                        self._gpu_info.power_monitoring = True
                    except ValueError:
                        self._gpu_info.tdp_watts = FALLBACK_GPU_TDP_WATTS
                        self._gpu_info.tdp_type = GpuTdpType.ESTIMATED_TDP_TYPE
                else:
                    self._gpu_info.tdp_watts = FALLBACK_GPU_TDP_WATTS
                    self._gpu_info.tdp_type = GpuTdpType.ESTIMATED_TDP_TYPE
            else:
                self._gpu_info.tdp_watts = FALLBACK_GPU_TDP_WATTS
                self._gpu_info.tdp_type = GpuTdpType.ESTIMATED_TDP_TYPE

            # Check if power monitoring works
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=power.draw",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0 and result.stdout.strip():
                if "[N/A]" not in result.stdout:
                    self._gpu_info.power_monitoring = True

            self._gpu_vendor = GpuVendor.NVIDIA
            return True

        except (subprocess.SubprocessError, ValueError, FileNotFoundError):
            pass

        return False

    def _detect_amd_gpu(self) -> bool:
        """
        Detect AMD GPU using ROCm or sysfs.

        Returns:
            True if AMD GPU was detected.
        """
        drm_path = "/sys/class/drm"
        if not os.path.isdir(drm_path):
            return False

        try:
            for entry in os.listdir(drm_path):
                if not entry.startswith("card"):
                    continue

                card_path = os.path.join(drm_path, entry)
                vendor_path = os.path.join(card_path, "device/vendor")
                vendor = SysfsReader.read_string(vendor_path)

                # AMD vendor ID
                if vendor == "0x1002":
                    self._gpu_info.vendor = "AMD"
                    self._gpu_vendor = GpuVendor.AMD

                    # Get model name
                    product_path = os.path.join(card_path, "device/product_name")
                    self._gpu_info.model = SysfsReader.read_string(product_path)
                    if not self._gpu_info.model:
                        self._gpu_info.model = "AMD GPU"

                    # Get memory
                    mem_path = os.path.join(card_path, "device/mem_info_vram_total")
                    mem_bytes = SysfsReader.read_long(mem_path)
                    if mem_bytes > 0:
                        self._gpu_info.mem_total_kb = mem_bytes // 1024

                    # Cache AMD sysfs paths
                    device_path = os.path.join(card_path, "device")
                    self._amd_gpu_busy_path = os.path.join(
                        device_path, "gpu_busy_percent"
                    )
                    self._amd_mem_busy_path = os.path.join(
                        device_path, "mem_busy_percent"
                    )
                    self._amd_power_path = os.path.join(
                        device_path, "hwmon/hwmon0/power1_average"
                    )
                    self._amd_vram_used_path = os.path.join(
                        device_path, "mem_info_vram_used"
                    )
                    self._amd_vram_total_path = mem_path

                    self._gpu_info.tdp_watts = FALLBACK_GPU_TDP_WATTS
                    self._gpu_info.tdp_type = GpuTdpType.SYSFS_TDP_TYPE
                    self._gpu_info.available = True
                    self._gpu_info.power_monitoring = False
                    return True
        except OSError:
            pass

        return False

    def _detect_intel_gpu(self) -> bool:
        """
        Detect Intel GPU using sysfs.

        Returns:
            True if Intel GPU was detected.
        """
        drm_path = "/sys/class/drm"
        if not os.path.isdir(drm_path):
            return False

        try:
            for entry in os.listdir(drm_path):
                if not entry.startswith("card"):
                    continue

                card_path = os.path.join(drm_path, entry)
                vendor_path = os.path.join(card_path, "device/vendor")
                vendor = SysfsReader.read_string(vendor_path)

                # Intel vendor ID
                if vendor == "0x8086":
                    self._gpu_info.vendor = "Intel"
                    self._gpu_info.model = "Intel Integrated Graphics"
                    self._gpu_info.tdp_watts = 25.0  # Typical iGPU TDP
                    self._gpu_info.tdp_type = GpuTdpType.ESTIMATED_TDP_TYPE
                    self._gpu_info.available = True
                    self._gpu_info.power_monitoring = False
                    self._gpu_vendor = GpuVendor.INTEL

                    # Cache Intel sysfs paths
                    self._intel_freq_path = os.path.join(card_path, "gt_cur_freq_mhz")
                    return True
        except OSError:
            pass

        return False

    def _read_nvidia_metrics(self) -> GpuMetrics:
        """
        Read NVIDIA GPU metrics via nvidia-smi.

        Returns:
            GpuMetrics structure with current values.
        """
        metrics = GpuMetrics()

        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,power.draw",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode == 0 and result.stdout.strip():
                parts = result.stdout.strip().split(",")
                if len(parts) >= 3:
                    metrics.utilization_percent = float(parts[0].strip())
                    metrics.mem_used_kb = int(float(parts[1].strip()) * 1024)
                    metrics.power_w = float(parts[2].strip())
        except (subprocess.SubprocessError, ValueError):
            pass

        return metrics

    def _read_amd_metrics(self) -> GpuMetrics:
        """
        Read AMD GPU metrics via sysfs.

        Returns:
            GpuMetrics structure with current values.
        """
        metrics = GpuMetrics()

        if self._amd_gpu_busy_path:
            metrics.utilization_percent = float(
                SysfsReader.read_long(self._amd_gpu_busy_path)
            )

        if self._amd_vram_used_path:
            vram_bytes = SysfsReader.read_long(self._amd_vram_used_path)
            if vram_bytes > 0:
                metrics.mem_used_kb = vram_bytes // 1024

        if self._amd_power_path:
            power_uw = SysfsReader.read_long(self._amd_power_path)
            if power_uw > 0:
                metrics.power_w = power_uw / 1_000_000.0

        return metrics

    def _read_intel_metrics(self) -> GpuMetrics:
        """
        Read Intel GPU metrics via sysfs.

        Returns:
            GpuMetrics structure with current values.
        """
        metrics = GpuMetrics()

        if self._intel_freq_path:
            freq_mhz = SysfsReader.read_long(self._intel_freq_path)
            if freq_mhz > 0:
                # Estimate utilization from frequency (rough approximation)
                max_freq = 1500  # Typical max for Intel iGPU
                metrics.utilization_percent = min(100.0, (freq_mhz / max_freq) * 100.0)

        return metrics

    def _read_nvidia_process_metrics(self, pid: int) -> ProcessGpuMetrics:
        """
        Read NVIDIA per-process GPU metrics via nvidia-smi.

        Args:
            pid: Process ID to query.

        Returns:
            ProcessGpuMetrics structure with current values.
        """
        metrics = ProcessGpuMetrics()

        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-compute-apps=pid,used_memory",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )

            if result.returncode == 0 and result.stdout.strip():
                for line in result.stdout.strip().split("\n"):
                    parts = line.split(",")
                    if len(parts) >= 2:
                        proc_pid = int(parts[0].strip())
                        if proc_pid == pid:
                            metrics.is_using_gpu = True
                            metrics.mem_used_kb = int(float(parts[1].strip()) * 1024)
                            # Estimate utilization based on memory usage
                            if self._gpu_info.mem_total_kb > 0:
                                metrics.estimated_utilization_percent = (
                                    metrics.mem_used_kb
                                    / self._gpu_info.mem_total_kb
                                    * 100.0
                                )
                            break
        except (subprocess.SubprocessError, ValueError):
            pass

        return metrics

    def _read_amd_process_metrics(self, pid: int) -> ProcessGpuMetrics:
        """
        Read AMD per-process GPU metrics via sysfs/fdinfo.

        Args:
            pid: Process ID to query.

        Returns:
            ProcessGpuMetrics structure with current values.
        """
        metrics = ProcessGpuMetrics()

        # Check if process has any GPU file descriptors
        fdinfo_path = f"/proc/{pid}/fdinfo"
        if not os.path.isdir(fdinfo_path):
            return metrics

        try:
            for fd in os.listdir(fdinfo_path):
                fd_path = os.path.join(fdinfo_path, fd)
                content = SysfsReader.read_string(fd_path)
                if "drm-engine-gfx" in content or "amdgpu" in content:
                    metrics.is_using_gpu = True
                    break
        except OSError:
            pass

        return metrics

    def _read_intel_process_metrics(self, pid: int) -> ProcessGpuMetrics:
        """
        Read Intel per-process GPU metrics via sysfs/fdinfo.

        Args:
            pid: Process ID to query.

        Returns:
            ProcessGpuMetrics structure with current values.
        """
        metrics = ProcessGpuMetrics()

        # Check if process has any GPU file descriptors
        fdinfo_path = f"/proc/{pid}/fdinfo"
        if not os.path.isdir(fdinfo_path):
            return metrics

        try:
            for fd in os.listdir(fdinfo_path):
                fd_path = os.path.join(fdinfo_path, fd)
                content = SysfsReader.read_string(fd_path)
                if "drm-engine-render" in content or "i915" in content:
                    metrics.is_using_gpu = True
                    break
        except OSError:
            pass

        return metrics

    def _exec_command(self, cmd: str) -> str:
        """
        Execute a command and return its output.

        Args:
            cmd: Command to execute.

        Returns:
            Command output as string.
        """
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=5,
            )
            return result.stdout.strip()
        except subprocess.SubprocessError:
            return ""

    def _estimate_energy_uj(self, power_w: float, utilization: float) -> float:
        """
        Estimate energy based on power and elapsed time.

        Args:
            power_w: Current power in watts.
            utilization: GPU utilization percentage.

        Returns:
            Energy delta in microjoules.
        """
        if power_w > 0:
            return power_w * 1_000_000.0  # Convert to microjoules per second

        # Estimate from utilization
        idle_power = self._gpu_info.tdp_watts * self._idle_power_factor
        active_power = self._gpu_info.tdp_watts - idle_power
        estimated_power = idle_power + active_power * (utilization / 100.0)
        return estimated_power * 1_000_000.0
