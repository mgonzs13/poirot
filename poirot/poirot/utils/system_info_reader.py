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
import socket
import platform
from dataclasses import dataclass


@dataclass
class CpuInfo:
    """Structure to hold CPU information."""

    model: str = ""
    cores: int = 0


@dataclass
class MemoryInfo:
    """Structure to hold Memory information."""

    total_kb: int = 0


@dataclass
class OsInfo:
    """Structure to hold OS information."""

    name: str = ""
    version: str = ""
    hostname: str = ""


class SystemInfoReader:
    """
    Class for reading system information.

    Provides methods to read CPU, memory, and OS information.
    """

    def __init__(self) -> None:
        """Default constructor."""
        pass

    def read_cpu_info(self) -> CpuInfo:
        """
        Read CPU information from /proc/cpuinfo.

        Returns:
            CpuInfo struct containing model and core count.
        """
        cpu_info = CpuInfo()

        try:
            with open("/proc/cpuinfo", "r") as f:
                content = f.read()

            # Parse model name and count physical cores
            physical_cores = set()
            current_physical_id = -1
            current_core_id = -1

            for line in content.split("\n"):
                if line.startswith("model name"):
                    cpu_info.model = line.split(":")[1].strip()
                elif line.startswith("physical id"):
                    current_physical_id = int(line.split(":")[1].strip())
                elif line.startswith("core id"):
                    current_core_id = int(line.split(":")[1].strip())
                    # Combine physical_id and core_id to get unique physical cores
                    unique_core = (current_physical_id << 16) | current_core_id
                    physical_cores.add(unique_core)

            # Use physical core count if available, otherwise fall back to hardware_concurrency
            if physical_cores:
                cpu_info.cores = len(physical_cores)
            else:
                cpu_info.cores = os.cpu_count() or 1

        except OSError:
            cpu_info.model = platform.processor() or "Unknown"
            cpu_info.cores = os.cpu_count() or 1

        if cpu_info.cores == 0:
            cpu_info.cores = os.cpu_count() or 1

        return cpu_info

    def read_memory_info(self) -> MemoryInfo:
        """
        Read total system memory.

        Returns:
            MemoryInfo struct containing total memory in KB.
        """
        mem_info = MemoryInfo()

        try:
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        # Parse value like "MemTotal:       16384000 kB"
                        parts = line.split()
                        if len(parts) >= 2:
                            mem_info.total_kb = int(parts[1])
                        break
        except (OSError, ValueError):
            pass

        return mem_info

    def read_os_info(self) -> OsInfo:
        """
        Read OS information from uname and /etc/os-release.

        Returns:
            OsInfo struct containing OS name, version, and hostname.
        """
        os_info = OsInfo()

        # Get uname info first (like C++ does)
        os_info.name = platform.system()  # sysname from uname
        os_info.version = platform.release()  # release from uname
        os_info.hostname = socket.gethostname()  # nodename from uname

        # Try to read PRETTY_NAME from /etc/os-release (overrides uname sysname)
        try:
            with open("/etc/os-release", "r") as f:
                for line in f:
                    if line.startswith("PRETTY_NAME="):
                        # Extract value after PRETTY_NAME="
                        value = line[13:].strip()
                        if value.endswith('"'):
                            value = value[:-1]
                        os_info.name = value
                        break
        except OSError:
            pass

        return os_info
