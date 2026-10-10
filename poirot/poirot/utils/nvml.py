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

import ctypes
from typing import List, Optional

NVML_LIBRARY = "libnvidia-ml.so.1"
NVML_SUCCESS = 0
NVML_ERROR_INSUFFICIENT_SIZE = 7
INITIAL_LIST_CAPACITY = 64
MAX_LIST_CAPACITY = 4096
NAME_BUFFER_SIZE = 96


class NvmlUtilization(ctypes.Structure):
    """NVML device utilization rates (GPU and memory), in percent."""

    _fields_ = [("gpu", ctypes.c_uint), ("memory", ctypes.c_uint)]


class NvmlMemory(ctypes.Structure):
    """NVML device memory information in bytes."""

    _fields_ = [
        ("total", ctypes.c_ulonglong),
        ("free", ctypes.c_ulonglong),
        ("used", ctypes.c_ulonglong),
    ]


class NvmlProcessUtilizationSample(ctypes.Structure):
    """NVML per-process utilization sample."""

    _fields_ = [
        ("pid", ctypes.c_uint),
        ("time_stamp_us", ctypes.c_ulonglong),
        ("sm_util", ctypes.c_uint),
        ("mem_util", ctypes.c_uint),
        ("enc_util", ctypes.c_uint),
        ("dec_util", ctypes.c_uint),
    ]


class NvmlProcessInfoV2(ctypes.Structure):
    """NVML process information (stable v2 layout)."""

    _fields_ = [
        ("pid", ctypes.c_uint),
        ("used_gpu_memory", ctypes.c_ulonglong),
        ("gpu_instance_id", ctypes.c_uint),
        ("compute_instance_id", ctypes.c_uint),
    ]


class NvmlLoader:
    """
    Minimal runtime loader for the NVIDIA Management Library.

    Loads libnvidia-ml.so.1 with ctypes and resolves only the symbols POIROT
    needs. The library is intentionally never unloaded and nvmlShutdown is
    never called because NVML is process-wide.
    """

    def __init__(self) -> None:
        """Constructor. Does not load the library."""
        self._available = False
        self._library = None
        self._device = ctypes.c_void_p()

    def initialize(self) -> bool:
        """Load NVML and acquire device 0."""
        if self._available:
            return True

        try:
            self._library = ctypes.CDLL(NVML_LIBRARY)
        except OSError:
            return False

        if not self._bind():
            return False

        if self._init() != NVML_SUCCESS:
            return False
        if self._get_handle(0, ctypes.byref(self._device)) != NVML_SUCCESS:
            return False

        self._available = True
        return True

    def _bind(self) -> bool:
        """Resolve all NVML symbols and declare their ctypes prototypes."""
        try:
            self._init = self._library.nvmlInit_v2
            self._get_handle = self._library.nvmlDeviceGetHandleByIndex_v2
            self._get_name = self._library.nvmlDeviceGetName
            self._get_memory = self._library.nvmlDeviceGetMemoryInfo
            self._get_power = self._library.nvmlDeviceGetPowerUsage
            self._get_limit = self._library.nvmlDeviceGetPowerManagementLimit
            self._get_util = self._library.nvmlDeviceGetUtilizationRates
            self._get_running = self._library.nvmlDeviceGetComputeRunningProcesses_v2
            self._get_proc_util = self._library.nvmlDeviceGetProcessUtilization
        except AttributeError:
            return False

        self._init.restype = ctypes.c_int
        self._init.argtypes = []
        self._get_handle.restype = ctypes.c_int
        self._get_handle.argtypes = [ctypes.c_uint, ctypes.POINTER(ctypes.c_void_p)]
        self._get_name.restype = ctypes.c_int
        self._get_name.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_char),
            ctypes.c_uint,
        ]
        self._get_memory.restype = ctypes.c_int
        self._get_memory.argtypes = [ctypes.c_void_p, ctypes.POINTER(NvmlMemory)]
        self._get_power.restype = ctypes.c_int
        self._get_power.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]
        self._get_limit.restype = ctypes.c_int
        self._get_limit.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint)]
        self._get_util.restype = ctypes.c_int
        self._get_util.argtypes = [ctypes.c_void_p, ctypes.POINTER(NvmlUtilization)]
        self._get_running.restype = ctypes.c_int
        self._get_running.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(NvmlProcessInfoV2),
        ]
        self._get_proc_util.restype = ctypes.c_int
        self._get_proc_util.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(NvmlProcessUtilizationSample),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.c_ulonglong,
        ]
        return True

    def available(self) -> bool:
        """Check whether NVML is loaded and device 0 is usable."""
        return self._available

    def name(self) -> str:
        """Read the device name, or an empty string when unavailable."""
        if not self._available:
            return ""
        buffer = ctypes.create_string_buffer(NAME_BUFFER_SIZE)
        if self._get_name(self._device, buffer, NAME_BUFFER_SIZE) != NVML_SUCCESS:
            return ""
        return buffer.value.decode("utf-8", errors="replace")

    def read_memory(self) -> Optional[NvmlMemory]:
        """Read device memory information."""
        if not self._available:
            return None
        memory = NvmlMemory()
        if self._get_memory(self._device, ctypes.byref(memory)) != NVML_SUCCESS:
            return None
        return memory

    def read_power_w(self) -> Optional[float]:
        """Read the current device power draw in watts."""
        if not self._available:
            return None
        milliwatts = ctypes.c_uint()
        if self._get_power(self._device, ctypes.byref(milliwatts)) != NVML_SUCCESS:
            return None
        return milliwatts.value / 1000.0

    def read_power_limit_w(self) -> Optional[float]:
        """Read the device power management limit in watts."""
        if not self._available:
            return None
        milliwatts = ctypes.c_uint()
        if self._get_limit(self._device, ctypes.byref(milliwatts)) != NVML_SUCCESS:
            return None
        return milliwatts.value / 1000.0

    def read_device_utilization(self) -> Optional[float]:
        """Read the device GPU utilization in percent."""
        if not self._available:
            return None
        utilization = NvmlUtilization()
        if self._get_util(self._device, ctypes.byref(utilization)) != NVML_SUCCESS:
            return None
        return float(utilization.gpu)

    def _read_list(self, function, element_type):
        """Read a variable-size NVML list, retrying on INSUFFICIENT_SIZE."""
        if not self._available:
            return None
        count = ctypes.c_uint(INITIAL_LIST_CAPACITY)
        elements = (element_type * INITIAL_LIST_CAPACITY)()
        result = function(self._device, ctypes.byref(count), elements)
        if result == NVML_ERROR_INSUFFICIENT_SIZE:
            if count.value == 0 or count.value > MAX_LIST_CAPACITY:
                return None
            count = ctypes.c_uint(count.value)
            elements = (element_type * count.value)()
            result = function(self._device, ctypes.byref(count), elements)
        if result != NVML_SUCCESS:
            return None
        return [elements[i] for i in range(count.value)]

    def read_running_processes(self) -> Optional[List[NvmlProcessInfoV2]]:
        """Read processes with GPU compute contexts."""
        if not self._available:
            return None
        return self._read_list(self._get_running, NvmlProcessInfoV2)

    def read_process_utilization(
        self,
    ) -> Optional[List[NvmlProcessUtilizationSample]]:
        """Read per-process utilization samples."""
        if not self._available:
            return None
        function = self._get_proc_util
        count = ctypes.c_uint(INITIAL_LIST_CAPACITY)
        samples = (NvmlProcessUtilizationSample * INITIAL_LIST_CAPACITY)()
        result = function(self._device, samples, ctypes.byref(count), 0)
        if result == NVML_ERROR_INSUFFICIENT_SIZE:
            if count.value == 0 or count.value > MAX_LIST_CAPACITY:
                return None
            count = ctypes.c_uint(count.value)
            samples = (NvmlProcessUtilizationSample * count.value)()
            result = function(self._device, samples, ctypes.byref(count), 0)
        if result != NVML_SUCCESS:
            return None
        return [samples[i] for i in range(count.value)]
