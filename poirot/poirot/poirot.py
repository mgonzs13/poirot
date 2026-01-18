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
import sys
import time
import functools
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, TypeVar

from rclpy.node import Node
from rclpy.qos import QoSProfile

from poirot_msgs.msg import (
    CpuInfo,
    FunctionCall,
    FunctionStats,
    GpuInfo,
    ProcessInfo,
    ProfilingData,
    SystemInfo,
)

from poirot.utils.co2_manager import Co2Manager
from poirot.utils.energy_monitor import EnergyMonitor
from poirot.utils.gpu_monitor import GpuMonitor, GpuTdpType
from poirot.utils.hwmon_scanner import HwmonScanner
from poirot.utils.power_estimator import PowerEstimator, TdpType
from poirot.utils.process_metrics import ProcessMetrics
from poirot.utils.string_utils import StringUtils
from poirot.utils.system_info_reader import SystemInfoReader
from poirot.utils.thread_metrics import ThreadMetrics

# Type variable for generic function decoration
F = TypeVar("F", bound=Callable[..., Any])


@dataclass
class ThreadProfilingContext:
    """Context structure to hold per-thread profiling data."""

    function_name: str = ""
    file: str = ""
    line: int = 0
    start_time: float = 0.0
    start_cpu_time_us: int = 0
    start_process_cpu_time_us: int = 0
    start_gpu_utilization_percent: float = 0.0
    start_mem_kb: int = 0
    start_gpu_mem_kb: int = 0
    start_io_read_bytes: int = 0
    start_io_write_bytes: int = 0
    start_context_switches: int = 0
    start_cpu_energy_uj: float = 0.0
    start_gpu_energy_uj: float = 0.0
    thread_id: int = 0


class Poirot:
    """
    Main class for the Poirot profiler.

    Provides methods to start and stop profiling, retrieve statistics,
    and manage system and process information.
    """

    _instance: Optional["Poirot"] = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        """Initialize the Poirot profiler."""
        # System and process info
        self._system_info = SystemInfo()
        self._process_info = ProcessInfo()

        # Verbosity flag
        self._verbose = False

        # Statistics storage
        self._statistics: Dict[str, FunctionStats] = {}
        self._statistics_lock = threading.RLock()

        # Thread contexts
        self._thread_contexts: Dict[int, ThreadProfilingContext] = {}
        self._contexts_lock = threading.Lock()

        # Utility instances
        self._co2_manager = Co2Manager()
        self._hwmon_scanner = HwmonScanner()
        self._power_estimator = PowerEstimator(self._hwmon_scanner)
        self._energy_monitor = EnergyMonitor(self._hwmon_scanner)
        self._gpu_monitor = GpuMonitor()
        self._process_metrics = ProcessMetrics()
        self._thread_metrics = ThreadMetrics()

        # ROS 2 integration
        self._node: Optional[Node] = None
        self._publisher = None

        # Initialize ROS 2 node and publisher
        node_name = f"poirot_{StringUtils.generate_uuid()}_node"
        self._node = Node(node_name, use_global_arguments=False)

        qos = QoSProfile(depth=100)
        self._publisher = self._node.create_publisher(ProfilingData, "poirot/data", qos)

        # Auto-configuration
        self._auto_configure()

    def _auto_configure(self) -> None:
        """Auto-configure the profiler."""

        # Detect system information
        self._detect_system_info()

    def _detect_system_info(self) -> None:
        """Detect system information."""
        system_reader = SystemInfoReader()

        # CPU info
        cpu_info = system_reader.read_cpu_info()
        self._system_info.cpu_info.model = cpu_info.model
        self._system_info.cpu_info.cores = cpu_info.cores

        # Memory info
        mem_info = system_reader.read_memory_info()
        self._system_info.mem_total_kb = mem_info.total_kb

        # OS info
        os_info = system_reader.read_os_info()
        self._system_info.os_name = os_info.name
        self._system_info.os_version = os_info.version
        self._system_info.hostname = os_info.hostname

        # Configure power estimator
        self._power_estimator.set_cpu_cores(self._system_info.cpu_info.cores)

        # RAPL detection
        self._system_info.cpu_info.rapl_available = self._power_estimator.rapl_available()

        # TDP detection
        tdp_watts, tdp_type = self._power_estimator.read_tdp_watts()
        self._system_info.cpu_info.tdp_watts = tdp_watts

        tdp_type_map = {
            TdpType.INTEL_RAPL_TDP_TYPE: CpuInfo.INTEL_RAPL_TDP_TYPE,
            TdpType.AMD_RAPL_TDP_TYPE: CpuInfo.AMD_RAPL_TDP_TYPE,
            TdpType.HWMON_RAPL_TDP_TYPE: CpuInfo.HWMON_RAPL_TDP_TYPE,
            TdpType.THERMAL_POWER_TDP_TYPE: CpuInfo.THERMAL_POWER_TDP_TYPE,
            TdpType.CPU_CORES_FREQUENCY_TYPE: CpuInfo.CPU_CORES_FREQUENCY_TYPE,
            TdpType.CPU_CORES_TYPE: CpuInfo.CPU_CORES_TYPE,
        }
        self._system_info.cpu_info.tdp_watts_type = tdp_type_map.get(
            tdp_type, CpuInfo.CPU_CORES_TYPE
        )

        # Clamp TDP to bounds: min(max(tdp, min_tdp), max_tdp)
        # This ensures tdp is at least min_tdp, but not more than max_tdp
        min_tdp = self._power_estimator.read_min_tdp_watts()
        max_tdp = self._power_estimator.read_max_tdp_watts()
        self._system_info.cpu_info.tdp_watts = min(
            max(self._system_info.cpu_info.tdp_watts, min_tdp), max_tdp
        )

        # Update power estimator and energy monitor
        self._power_estimator.set_cpu_tdp_watts(self._system_info.cpu_info.tdp_watts)
        self._energy_monitor.set_cpu_tdp_watts(self._system_info.cpu_info.tdp_watts)
        self._energy_monitor.set_idle_power_factor(
            self._power_estimator.read_idle_power_factor()
        )

        # GPU detection
        if self._gpu_monitor.is_available():
            gpu_info = self._gpu_monitor.get_gpu_info()
            self._system_info.gpu_info.model = gpu_info.model
            self._system_info.gpu_info.vendor = gpu_info.vendor
            self._system_info.gpu_info.index = gpu_info.index
            self._system_info.gpu_info.mem_total_kb = gpu_info.mem_total_kb
            self._system_info.gpu_info.tdp_watts = gpu_info.tdp_watts

            gpu_tdp_type_map = {
                GpuTdpType.NVIDIA_SMI_TDP_TYPE: GpuInfo.NVIDIA_SMI_TDP_TYPE,
                GpuTdpType.AMD_ROCM_TDP_TYPE: GpuInfo.AMD_ROCM_TDP_TYPE,
                GpuTdpType.SYSFS_TDP_TYPE: GpuInfo.SYSFS_TDP_TYPE,
                GpuTdpType.ESTIMATED_TDP_TYPE: GpuInfo.ESTIMATED_TDP_TYPE,
            }
            self._system_info.gpu_info.tdp_watts_type = gpu_tdp_type_map.get(
                gpu_info.tdp_type, GpuInfo.ESTIMATED_TDP_TYPE
            )

            self._system_info.gpu_info.available = True
            self._system_info.gpu_info.power_monitoring = gpu_info.power_monitoring
        else:
            self._system_info.gpu_info.available = False
            self._system_info.gpu_info.power_monitoring = False

        # CO2 factor
        co2_info = self._co2_manager.get_co2_info()
        self._system_info.co2_info.country_code = co2_info.country
        self._system_info.co2_info.co2_factor_loaded = co2_info.co2_factor_loaded
        self._system_info.co2_info.co2_factor_kg_per_kwh = co2_info.co2_factor_kg_per_kwh

    def _get_thread_context(self) -> ThreadProfilingContext:
        """Get the thread-local profiling context."""
        thread_id = threading.get_ident()
        with self._contexts_lock:
            if thread_id not in self._thread_contexts:
                self._thread_contexts[thread_id] = ThreadProfilingContext()
            ctx = self._thread_contexts[thread_id]
            ctx.thread_id = thread_id
            return ctx

    def _read_process_data(self) -> None:
        """Read process-level data."""
        self._process_info.pid = os.getpid()
        self._process_info.cpu_percent = self._process_metrics.read_cpu_percent()
        self._process_info.threads = self._process_metrics.read_thread_count()

    def start_profiling(self, function_name: str, file: str, line: int) -> None:
        """
        Start profiling a function.

        Args:
            function_name: Name of the function being profiled.
            file: Source file where the function is defined.
            line: Line number in the source file.
        """
        self._read_process_data()

        ctx = self._get_thread_context()
        ctx.function_name = function_name
        ctx.file = file
        ctx.line = line
        ctx.start_time = time.perf_counter()
        ctx.start_cpu_time_us = self._thread_metrics.read_cpu_time_us()
        ctx.start_process_cpu_time_us = self._process_metrics.read_cpu_time_us()
        ctx.start_mem_kb = self._thread_metrics.read_memory_kb()

        io_bytes = self._thread_metrics.read_io_bytes()
        ctx.start_io_read_bytes = io_bytes.read_bytes
        ctx.start_io_write_bytes = io_bytes.write_bytes

        ctx.start_context_switches = self._thread_metrics.read_context_switches()
        ctx.start_cpu_energy_uj = self._energy_monitor.read_energy_uj(
            self._process_info.cpu_percent
        )

        if self._gpu_monitor.is_available():
            process_metrics = self._gpu_monitor.read_process_metrics()
            ctx.start_gpu_utilization_percent = (
                process_metrics.estimated_utilization_percent
                if process_metrics.is_using_gpu
                else 0.0
            )
            ctx.start_gpu_mem_kb = process_metrics.mem_used_kb
            ctx.start_gpu_energy_uj = self._gpu_monitor.read_process_energy_uj()
        else:
            ctx.start_gpu_utilization_percent = 0.0
            ctx.start_gpu_mem_kb = 0
            ctx.start_gpu_energy_uj = 0.0

    def stop_profiling(self) -> None:
        """Stop profiling the current function."""
        self._read_process_data()

        end_time = time.perf_counter()
        end_cpu_time_us = self._thread_metrics.read_cpu_time_us()
        end_process_cpu_time_us = self._process_metrics.read_cpu_time_us()
        end_mem_kb = self._thread_metrics.read_memory_kb()

        end_io_bytes = self._thread_metrics.read_io_bytes()
        end_io_read_bytes = end_io_bytes.read_bytes
        end_io_write_bytes = end_io_bytes.write_bytes

        end_context_switches = self._thread_metrics.read_context_switches()
        end_cpu_energy_uj = self._energy_monitor.read_energy_uj(
            self._process_info.cpu_percent
        )

        # GPU metrics
        end_gpu_utilization_percent = 0.0
        end_gpu_mem_kb = 0
        end_gpu_energy_uj = 0.0
        if self._gpu_monitor.is_available():
            process_metrics = self._gpu_monitor.read_process_metrics()
            end_gpu_utilization_percent = (
                process_metrics.estimated_utilization_percent
                if process_metrics.is_using_gpu
                else 0.0
            )
            end_gpu_mem_kb = process_metrics.mem_used_kb
            end_gpu_energy_uj = self._gpu_monitor.read_process_energy_uj()

        ctx = self._get_thread_context()

        # Calculate deltas
        wall_time_delta_us = (end_time - ctx.start_time) * 1_000_000.0
        thread_cpu_delta_us = float(end_cpu_time_us - ctx.start_cpu_time_us)
        process_cpu_delta_us = float(
            end_process_cpu_time_us - ctx.start_process_cpu_time_us
        )
        system_cpu_delta_us = wall_time_delta_us * float(
            self._process_metrics.get_num_cpus()
        )
        cpu_total_energy_delta_uj = end_cpu_energy_uj - ctx.start_cpu_energy_uj
        gpu_energy_delta_uj = end_gpu_energy_uj - ctx.start_gpu_energy_uj

        # Calculate thread-level CPU energy
        thread_cpu_energy_uj = self._energy_monitor.calculate_thread_energy_uj(
            thread_cpu_delta_us,
            process_cpu_delta_us,
            system_cpu_delta_us,
            cpu_total_energy_delta_uj,
        )

        # Total energy
        total_energy_uj = thread_cpu_energy_uj + gpu_energy_delta_uj

        # Prepare FunctionCall message
        call = FunctionCall()
        if self._node is not None:
            call.timestamp = self._node.get_clock().now().to_msg()

        call.data.wall_time_us = int(wall_time_delta_us)
        call.data.cpu_time_us = int(thread_cpu_delta_us)
        call.data.process_cpu_time_us = int(process_cpu_delta_us)
        call.data.system_cpu_time_us = int(system_cpu_delta_us)
        call.data.mem_kb = end_mem_kb - ctx.start_mem_kb
        call.data.io_read_bytes = end_io_read_bytes - ctx.start_io_read_bytes
        call.data.io_write_bytes = end_io_write_bytes - ctx.start_io_write_bytes
        call.data.ctx_switches = end_context_switches - ctx.start_context_switches
        call.data.cpu_energy_uj = thread_cpu_energy_uj
        call.data.cpu_total_energy_uj = cpu_total_energy_delta_uj

        # GPU data
        call.data.gpu_utilization_percent = (
            end_gpu_utilization_percent - ctx.start_gpu_utilization_percent
        )
        call.data.gpu_mem_kb = end_gpu_mem_kb - ctx.start_gpu_mem_kb
        call.data.gpu_energy_uj = gpu_energy_delta_uj

        call.data.total_energy_uj = total_energy_uj

        # Calculate CO2
        UJ_TO_KWH = 1.0 / 1e6 / 3600.0 / 1000.0
        KG_TO_UG = 1e9
        energy_kwh = call.data.total_energy_uj * UJ_TO_KWH
        call.data.co2_ug = (
            energy_kwh * self._system_info.co2_info.co2_factor_kg_per_kwh * KG_TO_UG
        )

        with self._statistics_lock:
            if ctx.function_name not in self._statistics:
                self._statistics[ctx.function_name] = FunctionStats()
            stats = self._statistics[ctx.function_name]
            stats.name = ctx.function_name
            stats.file = ctx.file
            stats.line = ctx.line
            stats.call_count += 1
            stats.call = call

        if self._verbose:
            print(
                f"[PROFILE] {ctx.function_name} | "
                f"Wall: {call.data.wall_time_us}us | "
                f"CPU: {call.data.cpu_time_us}us | "
                f"Mem: {call.data.mem_kb}KB | "
                f"IO R/W: {call.data.io_read_bytes}/{call.data.io_write_bytes}B | "
                f"CtxSw: {call.data.ctx_switches} | "
                f"CPU Energy: {call.data.cpu_energy_uj}uJ | "
                f"GPU Energy: {call.data.gpu_energy_uj}uJ | "
                f"Total Energy: {call.data.total_energy_uj}uJ | "
                f"CO2: {call.data.co2_ug}ug",
                file=sys.stderr,
            )

        self._publish_stats(ctx.function_name)

    def _publish_stats(self, function_name: str) -> None:
        """Publish profiling statistics."""
        if self._publisher is None:
            return

        with self._statistics_lock:
            if function_name not in self._statistics:
                return

            msg = ProfilingData()
            msg.system_info = self._system_info
            msg.process_info = self._process_info
            msg.function = self._statistics[function_name]
            msg.timestamp = msg.function.call.timestamp

            self._publisher.publish(msg)

    @classmethod
    def get_instance(cls) -> "Poirot":
        """
        Get the singleton instance of the Poirot profiler.

        Returns:
            Reference to the singleton Poirot instance.
        """
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = Poirot()
            return cls._instance

    @classmethod
    def set_verbose(cls, verbose: bool) -> None:
        """
        Set verbosity level for logging.

        Args:
            verbose: True to enable verbose logging, False to disable.
        """
        cls.get_instance()._verbose = verbose

    @classmethod
    def print_system_info(cls) -> None:
        """Print system information to stderr."""
        instance = cls.get_instance()
        info = instance._system_info

        print("\n" + "=" * 64, file=sys.stderr)
        print("              SYSTEM INFORMATION", file=sys.stderr)
        print("=" * 64, file=sys.stderr)
        print(f"OS:       {info.os_name}", file=sys.stderr)
        print(f"OS Version: {info.os_version}", file=sys.stderr)
        print(f"Hostname: {info.hostname}", file=sys.stderr)
        print(f"CPU:      {info.cpu_info.model}", file=sys.stderr)
        print(f"Cores:    {info.cpu_info.cores}", file=sys.stderr)
        print(f"Memory:   {info.mem_total_kb // 1024} MB", file=sys.stderr)
        print(
            f"RAPL:     {'Yes' if info.cpu_info.rapl_available else 'No'}",
            file=sys.stderr,
        )
        print(f"CPU TDP:  {info.cpu_info.tdp_watts} W", file=sys.stderr)
        print(f"TDP Type: {info.cpu_info.tdp_watts_type}", file=sys.stderr)
        print("-" * 64, file=sys.stderr)
        print(
            f"GPU:      {info.gpu_info.model if info.gpu_info.available else 'Not available'}",
            file=sys.stderr,
        )

        if info.gpu_info.available:
            print(f"GPU Vendor: {info.gpu_info.vendor}", file=sys.stderr)
            print(f"GPU Memory: {info.gpu_info.mem_total_kb // 1024} MB", file=sys.stderr)
            print(f"GPU TDP:  {info.gpu_info.tdp_watts} W", file=sys.stderr)
            print(
                f"GPU Power Mon: {'Yes' if info.gpu_info.power_monitoring else 'No'}",
                file=sys.stderr,
            )

        print("-" * 64, file=sys.stderr)
        print(f"Country:  {info.co2_info.country_code}", file=sys.stderr)
        print(
            f"CO2 Loaded: {'Yes' if info.co2_info.co2_factor_loaded else 'No'}",
            file=sys.stderr,
        )
        print(f"CO2:      {info.co2_info.co2_factor_kg_per_kwh} kg/kWh", file=sys.stderr)
        print("=" * 64, file=sys.stderr)


def profile_function(func: F) -> F:
    """
    Decorator to automatically profile a function's execution.

    This decorator starts profiling upon function entry and stops profiling
    upon function exit, ensuring that profiling is correctly managed even
    in the presence of exceptions or early returns.

    Args:
        func: The function to profile.

    Returns:
        The wrapped function.

    Example:
        @profile_function
        def my_function():
            # Function code here
            pass
    """

    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        profiler = Poirot.get_instance()

        # Get the fully qualified function name
        qualname = func.__qualname__ if hasattr(func, "__qualname__") else func.__name__

        # Add file and line information
        file = os.path.abspath(func.__code__.co_filename)
        line = func.__code__.co_firstlineno

        profiler.start_profiling(qualname, file, line)
        try:
            return func(*args, **kwargs)
        finally:
            profiler.stop_profiling()

    return wrapper  # type: ignore
