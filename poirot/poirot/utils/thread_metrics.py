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
import os
import resource
from dataclasses import dataclass


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
        self._pid = os.getpid()

    def _get_tid(self) -> int:
        """Get the current thread ID."""
        try:
            # On Linux, we can get the native thread ID
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            SYS_gettid = 186  # syscall number for gettid on x86_64
            tid = libc.syscall(SYS_gettid)
            return tid if tid > 0 else self._pid
        except Exception:
            return self._pid

    def read_cpu_time_us(self) -> int:
        """
        Read thread CPU time in microseconds.

        Returns:
            CPU time in microseconds.
        """
        try:
            # Try to read from /proc/self/stat for thread CPU time
            tid = self._get_tid()
            stat_path = f"/proc/{self._pid}/task/{tid}/stat"

            with open(stat_path, "r") as f:
                parts = f.read().split()
                # utime (index 13) and stime (index 14) in clock ticks
                if len(parts) > 14:
                    utime = int(parts[13])
                    stime = int(parts[14])
                    # Convert from clock ticks to microseconds
                    clk_tck = os.sysconf("SC_CLK_TCK")
                    return int((utime + stime) * 1_000_000 / clk_tck)
        except (OSError, ValueError, IndexError):
            pass

        # Fallback to process-level resource usage
        try:
            usage = resource.getrusage(resource.RUSAGE_THREAD)
            return int((usage.ru_utime + usage.ru_stime) * 1_000_000)
        except (ValueError, AttributeError):
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return int((usage.ru_utime + usage.ru_stime) * 1_000_000)

    def read_memory_kb(self) -> int:
        """
        Read thread memory usage in KB (VmRSS).

        Returns:
            Memory usage in KB.
        """
        try:
            tid = self._get_tid()
            status_path = f"/proc/{self._pid}/task/{tid}/status"

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
            tid = self._get_tid()
            io_path = f"/proc/{self._pid}/task/{tid}/io"

            with open(io_path, "r") as f:
                for line in f:
                    if line.startswith("read_bytes:"):
                        io_bytes.read_bytes = int(line.split(":")[1].strip())
                    elif line.startswith("write_bytes:"):
                        io_bytes.write_bytes = int(line.split(":")[1].strip())
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
            tid = self._get_tid()
            status_path = f"/proc/{self._pid}/task/{tid}/status"

            voluntary = 0
            nonvoluntary = 0

            with open(status_path, "r") as f:
                for line in f:
                    if line.startswith("voluntary_ctxt_switches:"):
                        voluntary = int(line.split(":")[1].strip())
                    elif line.startswith("nonvoluntary_ctxt_switches:"):
                        nonvoluntary = int(line.split(":")[1].strip())

            return voluntary + nonvoluntary
        except (OSError, ValueError):
            pass

        # Fallback to resource usage
        try:
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return usage.ru_nvcsw + usage.ru_nivcsw
        except (ValueError, AttributeError):
            return 0
