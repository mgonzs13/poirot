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


class SysfsReader:
    """
    Static utility class for reading sysfs files.

    Provides methods for reading different types of values from
    sysfs virtual filesystem entries commonly used for system monitoring.
    """

    def __init__(self) -> None:
        """Prevent instantiation."""
        raise RuntimeError("SysfsReader is a static class and cannot be instantiated")

    @staticmethod
    def read_long(path: str) -> int:
        """
        Read a long integer value from a sysfs file.

        Args:
            path: Path to the sysfs file.

        Returns:
            The value read, or -1 on failure.
        """
        try:
            with open(path, "r") as f:
                return int(f.read().strip())
        except (OSError, ValueError):
            return -1

    @staticmethod
    def read_double(path: str) -> float:
        """
        Read a double value from a sysfs file.

        Args:
            path: Path to the sysfs file.

        Returns:
            The value read, or 0.0 on failure.
        """
        try:
            with open(path, "r") as f:
                return float(f.read().strip())
        except (OSError, ValueError):
            return 0.0

    @staticmethod
    def read_string(path: str) -> str:
        """
        Read a string value from a sysfs file.

        Args:
            path: Path to the sysfs file.

        Returns:
            The value read, or empty string on failure.
        """
        try:
            with open(path, "r") as f:
                return f.read().strip()
        except OSError:
            return ""

    @staticmethod
    def get_thread_status_path(filename: str) -> str:
        """
        Get the path to a thread's status file.

        Args:
            filename: The filename within the task directory.

        Returns:
            Path to the thread's status file.
        """
        pid = os.getpid()
        try:
            libc = ctypes.CDLL("libc.so.6", use_errno=True)
            SYS_gettid = 186  # syscall number for gettid on x86_64
            tid = libc.syscall(SYS_gettid)
            if tid <= 0:
                tid = pid
        except Exception:
            tid = pid
        return f"/proc/{pid}/task/{tid}/{filename}"
