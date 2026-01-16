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

import secrets


class StringUtils:
    """
    Static utility class for string operations.

    Provides utility methods for string manipulation and generation.
    """

    def __init__(self) -> None:
        """Prevent instantiation."""
        raise RuntimeError("StringUtils is a static class and cannot be instantiated")

    @staticmethod
    def generate_uuid() -> str:
        """
        Generate a unique UUID as a string.
        Generate a 16-character lowercase hexadecimal UUID.

        (0-f). We use the OS-provided randomness via `secrets.token_hex` to
        produce 16 lowercase hex characters (8 bytes -> 16 hex chars).

        Returns:
            A string containing a 16-character lowercase hexadecimal UUID.
        """
        # token_hex(8) produces 16 hex characters (8 bytes). It returns
        # lowercase hex digits by default.
        return secrets.token_hex(8)

    @staticmethod
    def trim(s: str) -> str:
        """
        Trim whitespace from both ends of a string.

        Args:
            s: The string to trim.

        Returns:
            The trimmed string.
        """
        # Use the explicit whitespace characters:
        # space, tab, newline and carriage return.
        return s.strip(" \t\n\r")

    @staticmethod
    def to_lower(s: str) -> str:
        """
        Convert string to lowercase.

        Args:
            s: The string to convert.

        Returns:
            The lowercase string.
        """
        return s.lower()

    @staticmethod
    def to_upper(s: str) -> str:
        """
        Convert string to uppercase.

        Args:
            s: The string to convert.

        Returns:
            The uppercase string.
        """
        return s.upper()
