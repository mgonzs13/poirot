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
import resource
import threading
import time
from dataclasses import dataclass


@dataclass
class ProcessIoBytes:
    """Structure to hold process I/O byte counts."""

    read_bytes: int = 0
    write_bytes: int = 0


class ProcessMetrics:
    """
    Class for reading process-level metrics with state tracking.

    Provides methods for measuring process CPU time, system CPU time,
    and thread count. Maintains state for CPU percentage calculation.
    """

    def __init__(self) -> None:
        """Default constructor."""
        self._pid = os.getpid()
        self._num_cpus = os.cpu_count() or 1
        self._prev_process_cpu_us: int = 0
        self._prev_cpu_read_time: float = time.time()
        self._cpu_read_mutex = threading.Lock()

    def read_cpu_time_us(self) -> int:
        """
        Read process CPU time in microseconds.

        Returns:
            CPU time in microseconds.
        """
        try:
            with open(f"/proc/{self._pid}/stat", "r") as f:
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

        # Fallback to resource usage
        try:
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return int((usage.ru_utime + usage.ru_stime) * 1_000_000)
        except (ValueError, AttributeError):
            return 0

    def get_num_cpus(self) -> int:
        """
        Get the number of CPU cores.

        Returns:
            Number of CPU cores.
        """
        return self._num_cpus

    def read_thread_count(self) -> int:
        """
        Read process thread count.

        Returns:
            Number of threads.
        """
        try:
            with open(f"/proc/{self._pid}/status", "r") as f:
                for line in f:
                    if line.startswith("Threads:"):
                        return int(line.split(":")[1].strip())
        except (OSError, ValueError):
            pass

        return threading.active_count()

    def read_cpu_percent(self) -> float:
        """
        Read process CPU usage percentage.

        This method requires state tracking and should be called on an instance.

        Returns:
            CPU usage percentage.
        """
        current_cpu_us = self.read_cpu_time_us()
        current_time = time.time()

        with self._cpu_read_mutex:
            elapsed_time = current_time - self._prev_cpu_read_time
            if elapsed_time <= 0:
                return 0.0

            cpu_delta_us = current_cpu_us - self._prev_process_cpu_us
            elapsed_us = elapsed_time * 1_000_000

            # Update previous values
            self._prev_process_cpu_us = current_cpu_us
            self._prev_cpu_read_time = current_time

            # Calculate percentage (accounting for multiple CPUs)
            if elapsed_us > 0:
                return (cpu_delta_us / elapsed_us) * 100.0

        return 0.0

    def read_memory_kb(self) -> int:
        """
        Read process memory usage in kilobytes.

        Returns:
            Memory usage in kilobytes.
        """
        try:
            with open(f"/proc/{self._pid}/status", "r") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        parts = line.split()
                        if len(parts) >= 2:
                            return int(parts[1])
        except (OSError, ValueError):
            pass

        try:
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return usage.ru_maxrss  # In KB on Linux
        except (ValueError, AttributeError):
            return 0

    def read_io_bytes(self) -> ProcessIoBytes:
        """
        Read process I/O bytes.

        Returns:
            ProcessIoBytes structure with read and write byte counts.
        """
        io_bytes = ProcessIoBytes()

        try:
            with open(f"/proc/{self._pid}/io", "r") as f:
                for line in f:
                    if line.startswith("read_bytes:"):
                        io_bytes.read_bytes = int(line.split(":")[1].strip())
                    elif line.startswith("write_bytes:"):
                        io_bytes.write_bytes = int(line.split(":")[1].strip())
        except (OSError, ValueError):
            pass

        return io_bytes
