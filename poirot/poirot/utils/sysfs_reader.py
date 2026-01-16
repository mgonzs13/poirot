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
import ctypes


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
        Tries to obtain the kernel thread id (TID) and return the
        thread-specific path /proc/self/task/<tid>/<filename> if it exists.
        Otherwise falls back to /proc/self/<filename>.
        """
        try:
            # Prefer threading.get_native_id() when available (Python 3.8+)
            tid = None
            try:
                import threading

                tid = threading.get_native_id()  # returns native thread id (TID)
            except Exception:
                tid = None

            if not tid:
                # Fallback to calling the libc syscall for gettid.
                libc = ctypes.CDLL("libc.so.6", use_errno=True)
                # SYS_gettid is platform dependent; 186 is common on x86_64.
                SYS_gettid = 186
                try:
                    tid = int(libc.syscall(SYS_gettid))
                except Exception:
                    tid = None

            if not tid or tid <= 0:
                # Could not obtain a valid thread id; fall back to process-level path
                return f"/proc/self/{filename}"

            path = f"/proc/self/task/{tid}/{filename}"
            # Return the thread-specific path if it actually exists, otherwise fall back
            if os.path.exists(path):
                return path

            return f"/proc/self/{filename}"
        except Exception:
            return f"/proc/self/{filename}"
