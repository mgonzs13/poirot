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
import json
import threading
from typing import Dict

# Default CO2 factor in kg CO2 per kWh (global average)
DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.436
# Timeout for CURL requests in seconds
CURL_TIMEOUT_SECONDS = 3

# Country code mapping from timezone
TIMEZONE_TO_COUNTRY: Dict[str, str] = {
    "Europe/Madrid": "ES",
    "Europe/London": "GB",
    "Europe/Paris": "FR",
    "Europe/Berlin": "DE",
    "Europe/Rome": "IT",
    "Europe/Amsterdam": "NL",
    "Europe/Brussels": "BE",
    "Europe/Vienna": "AT",
    "Europe/Zurich": "CH",
    "Europe/Stockholm": "SE",
    "Europe/Oslo": "NO",
    "Europe/Copenhagen": "DK",
    "Europe/Helsinki": "FI",
    "Europe/Warsaw": "PL",
    "Europe/Prague": "CZ",
    "Europe/Budapest": "HU",
    "Europe/Athens": "GR",
    "Europe/Lisbon": "PT",
    "Europe/Dublin": "IE",
    "America/New_York": "US",
    "America/Los_Angeles": "US",
    "America/Chicago": "US",
    "America/Denver": "US",
    "America/Toronto": "CA",
    "America/Vancouver": "CA",
    "America/Mexico_City": "MX",
    "America/Sao_Paulo": "BR",
    "America/Buenos_Aires": "AR",
    "Asia/Tokyo": "JP",
    "Asia/Shanghai": "CN",
    "Asia/Hong_Kong": "HK",
    "Asia/Singapore": "SG",
    "Asia/Seoul": "KR",
    "Asia/Mumbai": "IN",
    "Asia/Dubai": "AE",
    "Australia/Sydney": "AU",
    "Australia/Melbourne": "AU",
    "Pacific/Auckland": "NZ",
    "Africa/Johannesburg": "ZA",
    "Africa/Cairo": "EG",
}


class Co2Manager:
    """
    Class for managing CO2 emission factors.

    Provides methods for downloading CO2 factors from online sources
    and looking up factors by country.
    """

    CO2_DATA_URL = "https://raw.githubusercontent.com/mlco2/codecarbon/master/codecarbon/data/co2_grid_emission_factors.json"

    def __init__(self) -> None:
        """Constructor."""
        # Map of country code to CO2 factor
        self._co2_factors_by_country: Dict[str, float] = {}
        # Mutex for thread-safe access to CO2 factors
        self._co2_factors_mutex = threading.Lock()
        # Flag indicating if CO2 factors have been loaded
        self._co2_factors_loaded = False

    def download_factors(self) -> bool:
        """
        Download CO2 factors from online source.

        Returns:
            True if download succeeded, False otherwise.
        """
        try:
            import urllib.request

            request = urllib.request.Request(
                self.CO2_DATA_URL,
                headers={"User-Agent": "Poirot/1.0"},
            )

            with urllib.request.urlopen(
                request, timeout=CURL_TIMEOUT_SECONDS
            ) as response:
                data = json.loads(response.read().decode("utf-8"))

                with self._co2_factors_mutex:
                    # Parse the JSON data
                    for entry in data:
                        country_code = entry.get("country_code", "")
                        co2_factor = entry.get("co2_factor", 0.0)
                        if country_code and co2_factor > 0:
                            self._co2_factors_by_country[country_code] = co2_factor

                    self._co2_factors_loaded = len(self._co2_factors_by_country) > 0

            return self._co2_factors_loaded

        except Exception:
            return False

    def get_factor_for_country(self, country_code: str) -> float:
        """
        Get CO2 factor for a specific country.

        Args:
            country_code: ISO 2-letter country code.

        Returns:
            CO2 factor in kg CO2 per kWh.
        """
        with self._co2_factors_mutex:
            return self._co2_factors_by_country.get(
                country_code.upper(), DEFAULT_CO2_FACTOR_KG_PER_KWH
            )

    def get_country_from_timezone(self, timezone: str) -> str:
        """
        Get country code from timezone.

        Args:
            timezone: Timezone string (e.g., "Europe/Madrid").

        Returns:
            ISO 2-letter country code.
        """
        return TIMEZONE_TO_COUNTRY.get(timezone, "UNKNOWN")

    def get_system_timezone(self) -> str:
        """
        Get the current system timezone.

        Returns:
            Timezone string.
        """
        try:
            # Try to read from /etc/timezone
            with open("/etc/timezone", "r") as f:
                return f.read().strip()
        except OSError:
            pass

        try:
            # Try to read from /etc/localtime symlink
            link = os.readlink("/etc/localtime")
            # Extract timezone from path like /usr/share/zoneinfo/Europe/Madrid
            if "zoneinfo/" in link:
                return link.split("zoneinfo/")[1]
        except OSError:
            pass

        try:
            # Use Python's time module
            return time.tzname[0]
        except Exception:
            return "UTC"

    def factors_loaded(self) -> bool:
        """
        Check if CO2 factors have been loaded.

        Returns:
            True if factors are loaded.
        """
        return self._co2_factors_loaded

    def get_factor_count(self) -> int:
        """
        Get the number of loaded country factors.

        Returns:
            Number of countries with CO2 factors.
        """
        with self._co2_factors_mutex:
            return len(self._co2_factors_by_country)
