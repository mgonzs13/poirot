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
import re
import shutil
import time
import threading
import subprocess
from dataclasses import dataclass
from enum import IntEnum
from typing import List


class GpuTdpType(IntEnum):
    """GPU TDP detection type constants."""

    NO_TDP_TYPE = 0
    NVIDIA_TDP_TYPE = 1
    AMD_TDP_TYPE = 2


class GpuVendor(IntEnum):
    """GPU vendor type for optimized metric reading."""

    NONE = 0
    NVIDIA = 1
    AMD = 2


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
    tdp_type: GpuTdpType = GpuTdpType.NO_TDP_TYPE
    available: bool = False
    power_monitoring: bool = False


class GpuMonitor:
    """
    Class for monitoring GPU metrics and energy consumption.

    Supports NVIDIA GPUs via nvidia-smi and AMD GPUs via rocm-smi.
    Provides methods for reading GPU
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

        self._initialize()

    def _initialize(self) -> bool:
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
        # Attribute energy based on memory usage ratio
        if pid == 0:
            pid = os.getpid()

        with self._energy_mutex:
            current_time = time.time()
            elapsed_s = current_time - self._last_process_energy_read_time

            # Update timestamp regardless to avoid gaps
            self._last_process_energy_read_time = current_time

            if elapsed_s <= 0:
                return self._accumulated_process_energy_uj

            # Get process-specific metrics and system metrics
            proc_metrics = self.read_process_metrics(pid)
            if not proc_metrics.is_using_gpu:
                return self._accumulated_process_energy_uj

            sys_metrics = self.read_metrics()

            # Calculate memory ratio
            memory_ratio = 0.0
            if sys_metrics.mem_used_kb > 0 and proc_metrics.mem_used_kb > 0:
                try:
                    memory_ratio = float(proc_metrics.mem_used_kb) / float(
                        sys_metrics.mem_used_kb
                    )
                    memory_ratio = max(0.0, min(1.0, memory_ratio))
                except Exception:
                    memory_ratio = 0.0

            # Estimate process power
            process_power_w = 0.0
            if sys_metrics.power_w > 0.0 and memory_ratio > 0.0:
                process_power_w = sys_metrics.power_w * memory_ratio
            elif self._gpu_info.tdp_watts > 0.0:
                utilization_factor = sys_metrics.utilization_percent / 100.0
                process_power_w = self._gpu_info.tdp_watts * utilization_factor

            if process_power_w > 0.0:
                energy_delta_uj = process_power_w * (elapsed_s * 1_000_000.0)
                self._accumulated_process_energy_uj += energy_delta_uj

            return self._accumulated_process_energy_uj

    def _detect_nvidia_gpu(self) -> bool:
        """
        Detect NVIDIA GPU using nvidia-smi.

        Returns:
            True if NVIDIA GPU was detected.
        """
        # Check if nvidia-smi is available
        if not shutil.which("nvidia-smi"):
            return False

        try:
            # Get GPU name
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-gpu=name",
                    "--format=csv,noheader,nounits",
                ]
            )

            if not result:
                return False

            self._gpu_info.model = result
            self._gpu_info.vendor = "NVIDIA"
            self._gpu_info.index = 0
            self._gpu_info.available = True

            # Get total memory
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.total",
                    "--format=csv,noheader,nounits",
                ]
            )

            if result:
                try:
                    # nvidia-smi reports memory in MiB
                    self._gpu_info.mem_total_kb = int(float(result) * 1024)
                except ValueError:
                    self._gpu_info.mem_total_kb = 0

            # Get power limit (TDP)
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-gpu=power.limit",
                    "--format=csv,noheader,nounits",
                ]
            )

            if result:
                power_str = result
                if "[N/A]" not in power_str:
                    try:
                        self._gpu_info.tdp_watts = float(power_str)
                        self._gpu_info.tdp_type = GpuTdpType.NVIDIA_TDP_TYPE
                        self._gpu_info.power_monitoring = True
                    except ValueError:
                        self._gpu_info.tdp_watts = 0.0
                        self._gpu_info.tdp_type = GpuTdpType.NO_TDP_TYPE
                else:
                    self._gpu_info.tdp_watts = 0.0
                    self._gpu_info.tdp_type = GpuTdpType.NO_TDP_TYPE
            else:
                self._gpu_info.tdp_watts = 0.0
                self._gpu_info.tdp_type = GpuTdpType.NO_TDP_TYPE

            # Check if power monitoring works
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-gpu=power.draw",
                    "--format=csv,noheader,nounits",
                ]
            )

            if result:
                if "[N/A]" not in result:
                    self._gpu_info.power_monitoring = True

            self._gpu_vendor = GpuVendor.NVIDIA
            return True

        except (ValueError, FileNotFoundError):
            pass

        return False

    def _detect_amd_gpu(self) -> bool:
        """
        Detect AMD GPU using rocm-smi.

        Returns:
            True if AMD GPU was detected.
        """
        # Check if rocm-smi is available
        if not shutil.which("rocm-smi"):
            return False

        try:
            # Get GPU model
            result = self._exec_command(["rocm-smi", "--showproductname"])
            if not result:
                return False

            # Parse output
            lines = result.split("\n")
            for line in lines:
                if "Card series" in line and ":" in line:
                    parts = line.split(":")
                    if len(parts) > 2:
                        self._gpu_info.model = parts[2].strip()
                        break

            if not self._gpu_info.model:
                self._gpu_info.model = "AMD GPU"

            self._gpu_info.vendor = "AMD"
            self._gpu_info.index = 0
            self._gpu_info.available = True

            # Get total memory
            result = self._exec_command(["rocm-smi", "--showmeminfo", "vram"])

            if result:
                lines = result.split("\n")
                for line in lines:
                    if "VRAM Total Memory" in line and ":" in line:
                        parts = line.split(":")
                        if len(parts) > 2:
                            mem_str = parts[2].strip()
                            mem_mb = int(mem_str)
                            self._gpu_info.mem_total_kb = mem_mb / 1024
                            break

            # Get TDP
            result = self._exec_command(["rocm-smi", "--showmaxpower"])
            if result:
                lines = result.split("\n")
                for line in lines:
                    if "Package Power" in line and ":" in line:
                        parts = line.split(":")
                        if len(parts) > 2:
                            tdp_str = parts[2].strip()
                            self._gpu_info.tdp_watts = float(tdp_str)
                            self._gpu_info.tdp_type = GpuTdpType.AMD_TDP_TYPE
                            break

            # Check power monitoring
            result = self._exec_command(["rocm-smi", "--showpower"])
            if result and "N/A" not in result:
                self._gpu_info.power_monitoring = True

            self._gpu_vendor = GpuVendor.AMD
            return True

        except Exception:
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
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,memory.used,power.draw",
                    "--format=csv,noheader,nounits",
                ]
            )

            if result:
                parts = result.split(",")
                if len(parts) >= 3:
                    metrics.utilization_percent = float(parts[0].strip())
                    metrics.mem_used_kb = int(float(parts[1].strip()) * 1024)
                    metrics.power_w = float(parts[2].strip())
        except ValueError:
            pass

        # Calculate energy (time-integrated)
        metrics.energy_uj = self._estimate_energy_uj(
            metrics.power_w, metrics.utilization_percent
        )

        return metrics

    def _read_amd_metrics(self) -> GpuMetrics:
        """
        Read AMD GPU metrics via rocm-smi.

        Returns:
            GpuMetrics structure with current values.
        """
        metrics = GpuMetrics()

        try:
            # Read utilization
            result = self._exec_command(["rocm-smi", "--showuse"])
            if result:
                lines = result.split("\n")
                for line in lines:
                    if "GPU use" in line and ":" in line:
                        parts = line.split(":")
                        if len(parts) > 2:
                            use_str = parts[2].strip()
                            metrics.utilization_percent = float(use_str)
                            break

            # Read memory used
            result = self._exec_command(["rocm-smi", "--showmemuse"])
            if result:
                lines = result.split("\n")
                for line in lines:
                    if "memory" in line and "use" in line.lower() and ":" in line:
                        parts = line.split(":")
                        if len(parts) > 2:
                            mem_str = parts[2].strip()
                            mem_percent = float(mem_str)
                            metrics.mem_used_kb = (
                                mem_percent / 100.0 * self._gpu_info.mem_total_kb
                            )
                            break

            # Read power
            result = self._exec_command(["rocm-smi", "--showpower"])
            if result:
                lines = result.split("\n")
                for line in lines:
                    if ("Power" in line or "power" in line) and ":" in line:
                        parts = line.split(":")
                        if len(parts) > 2:
                            power_str = parts[2].strip()
                            metrics.power_w = float(power_str)
                            break

        except Exception:
            pass

        # Calculate energy (time-integrated)
        metrics.energy_uj = self._estimate_energy_uj(
            metrics.power_w, metrics.utilization_percent
        )

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
            result = self._exec_command(
                [
                    "nvidia-smi",
                    "--query-compute-apps=pid,used_memory",
                    "--format=csv,noheader,nounits",
                ],
            )

            if result:

                for line in result.split("\n"):

                    parts = line.split(",")
                    if len(parts) >= 2:

                        proc_pid = int(parts[0].strip())
                        if proc_pid == pid:
                            metrics.is_using_gpu = True
                            metrics.mem_used_kb = int(float(parts[1].strip()) * 1024)

                            if self._gpu_info.mem_total_kb > 0:
                                metrics.estimated_utilization_percent = (
                                    metrics.mem_used_kb
                                    / self._gpu_info.mem_total_kb
                                    * 100.0
                                )

                            break
        except ValueError:
            pass

        return metrics

    def _read_amd_process_metrics(self, pid: int) -> ProcessGpuMetrics:
        """
        Read AMD per-process GPU metrics via rocm-smi.

        Args:
            pid: Process ID to query.

        Returns:
            ProcessGpuMetrics structure with current values.
        """
        metrics = ProcessGpuMetrics()

        try:
            result = self._exec_command(["rocm-smi", "--showpids"])
            if result:
                lines = result.split("\n")
                for line in lines:
                    parts = line.split()
                    if len(parts) >= 4:
                        try:
                            proc_pid = int(parts[0].strip())
                            if proc_pid == pid:
                                metrics.is_using_gpu = True
                                mem_str = parts[3].strip()
                                metrics.mem_used_kb = int(mem_str) / 1024

                                if self._gpu_info.mem_total_kb > 0:
                                    metrics.estimated_utilization_percent = (
                                        metrics.mem_used_kb
                                        / self._gpu_info.mem_total_kb
                                        * 100.0
                                    )

                                break
                        except (ValueError, IndexError):
                            continue
        except Exception:
            pass

        return metrics

    def _exec_command(self, cmd: List[str]) -> str:
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
        # Integrate energy over elapsed time and accumulate (microjoules)
        with self._energy_mutex:
            current_time = time.time()
            elapsed_s = current_time - self._last_energy_read_time
            # Update last read time
            self._last_energy_read_time = current_time

            if elapsed_s <= 0:
                return self._accumulated_energy_uj

            # If we have an actual power reading, use it
            if power_w > 0.0:
                energy_delta_uj = power_w * (elapsed_s * 1_000_000.0)
                self._accumulated_energy_uj += energy_delta_uj
            elif self._gpu_info.tdp_watts > 0.0:
                # Estimate based on TDP and utilization
                estimated_power_w = self._gpu_info.tdp_watts * (utilization / 100.0)
                energy_delta_uj = estimated_power_w * (elapsed_s * 1_000_000.0)
                self._accumulated_energy_uj += energy_delta_uj

            return self._accumulated_energy_uj
