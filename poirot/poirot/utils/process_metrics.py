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
import time
import resource
import threading
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
        # Prev CPU time in microseconds. 0 means "not initialized" (first call)
        self._prev_process_cpu_us: int = 0
        # Use a monotonic clock for elapsed time measurements; None until first call
        self._prev_cpu_read_time: float | None = None
        self._cpu_read_mutex = threading.Lock()

    def read_cpu_time_us(self) -> int:
        """
        Read process CPU time in microseconds.

        Returns:
            CPU time in microseconds.
        """
        try:
            # Multiply seconds to microseconds and round to int
            return int(
                round(time.clock_gettime(time.CLOCK_PROCESS_CPUTIME_ID) * 1_000_000)
            )
        except (OSError, AttributeError):
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
            # Reads /proc/self/status and
            # returns 1 as default when the file cannot be parsed.
            with open("/proc/self/status", "r") as f:
                for line in f:
                    if line.startswith("Threads:"):
                        return int(line.split(":")[1].strip())
        except (OSError, ValueError):
            # Fall through to return default
            pass

        # Returns 1 as default if it can't find the Threads: line
        return 1

    def read_cpu_percent(self) -> float:
        """
        Read process CPU usage percentage.

        This method requires state tracking and should be called on an instance.

        Returns:
            CPU usage percentage.
        """
        current_cpu_us = self.read_cpu_time_us()

        # If we couldn't read CPU time, return 0
        if current_cpu_us <= 0:
            return 0.0

        now = time.monotonic()

        with self._cpu_read_mutex:
            # First call: initialize stored state and return 0.0
            if self._prev_process_cpu_us == 0 or self._prev_cpu_read_time is None:
                self._prev_process_cpu_us = int(current_cpu_us)
                self._prev_cpu_read_time = now
                return 0.0

            elapsed_us = (now - self._prev_cpu_read_time) * 1_000_000.0
            if elapsed_us <= 0.0:
                return 0.0

            process_cpu_delta_us = float(current_cpu_us - self._prev_process_cpu_us)

            # Total available CPU time in this period across all CPUs
            total_available_cpu_us = elapsed_us * float(self._num_cpus)

            pct = 0.0
            if total_available_cpu_us > 0.0:
                pct = (process_cpu_delta_us / total_available_cpu_us) * 100.0

            # Update state
            self._prev_process_cpu_us = int(current_cpu_us)
            self._prev_cpu_read_time = now

            return pct

    def read_memory_kb(self) -> int:
        """
        Read process memory usage in kilobytes.

        Returns:
            Memory usage in kilobytes.
        """
        try:
            with open("/proc/self/status", "r") as f:
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
