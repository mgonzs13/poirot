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
import resource
from dataclasses import dataclass

from .sysfs_reader import SysfsReader


@dataclass
class ThreadIoBytes:
    """Structure to hold thread I/O byte counts."""

    read_bytes: int = 0
    write_bytes: int = 0


class ThreadMetrics:
    """
    Utility class for reading thread-level metrics.

    Provides methods for measuring CPU time, memory, I/O, and context
    switches for the current thread. All methods are thread-safe and
    read data for the calling thread.
    """

    def __init__(self) -> None:
        """Default constructor."""
        self._pid = None

    def read_cpu_time_us(self) -> int:
        """
        Read thread CPU time in microseconds.

        Returns:
            CPU time in microseconds.
        """
        try:
            return int(time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID) * 1_000_000)
        except (OSError, AttributeError):
            return 0

    def read_memory_kb(self) -> int:
        """
        Read thread memory usage in KB (VmRSS).

        Returns:
            Memory usage in KB.
        """
        try:
            status_path = SysfsReader.get_thread_status_path("status")
            with open(status_path, "r") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        parts = line.split()
                        if len(parts) >= 2:
                            return int(parts[1])
        except (OSError, ValueError):
            pass

        # Fallback to process RSS
        try:
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return usage.ru_maxrss  # In KB on Linux
        except (ValueError, AttributeError):
            return 0

    def read_io_bytes(self) -> ThreadIoBytes:
        """
        Read thread I/O bytes.

        Returns:
            ThreadIoBytes structure with read and write byte counts.
        """
        io_bytes = ThreadIoBytes()

        try:
            io_path = SysfsReader.get_thread_status_path("io")
            with open(io_path, "r") as f:
                for line in f:
                    if line.startswith("read_bytes:"):
                        # grab part after ':' and strip
                        io_bytes.read_bytes = int(line.split(":", 1)[1].strip())
                    elif line.startswith("write_bytes:"):
                        io_bytes.write_bytes = int(line.split(":", 1)[1].strip())
        except (OSError, ValueError):
            pass

        return io_bytes

    def read_context_switches(self) -> int:
        """
        Read thread context switches (voluntary + non-voluntary).

        Returns:
            Total number of context switches.
        """
        try:
            status_path = SysfsReader.get_thread_status_path("status")

            voluntary = 0
            nonvoluntary = 0

            with open(status_path, "r") as f:
                for line in f:
                    if line.startswith("voluntary_ctxt_switches:"):
                        voluntary = int(line.split(":", 1)[1].strip())
                    elif line.startswith("nonvoluntary_ctxt_switches:"):
                        nonvoluntary = int(line.split(":", 1)[1].strip())

            return voluntary + nonvoluntary
        except (OSError, ValueError):
            pass

        # Fallback to resource usage
        try:
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return usage.ru_nvcsw + usage.ru_nivcsw
        except (ValueError, AttributeError):
            return 0
