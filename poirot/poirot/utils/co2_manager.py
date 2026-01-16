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
import csv
import threading
from typing import Dict

# Default CO2 factor in kg CO2 per kWh (global average)
DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.436
# Timeout for CURL requests in seconds
CURL_TIMEOUT_SECONDS = 3

# Country code mapping from timezone
TIMEZONE_TO_COUNTRY: Dict[str, str] = {
    # Europe
    "Europe/Madrid": "ES",
    "Europe/Paris": "FR",
    "Europe/London": "GB",
    "Europe/Berlin": "DE",
    "Europe/Rome": "IT",
    "Europe/Amsterdam": "NL",
    "Europe/Brussels": "BE",
    "Europe/Zurich": "CH",
    "Europe/Vienna": "AT",
    "Europe/Warsaw": "PL",
    "Europe/Prague": "CZ",
    "Europe/Budapest": "HU",
    "Europe/Stockholm": "SE",
    "Europe/Oslo": "NO",
    "Europe/Copenhagen": "DK",
    "Europe/Helsinki": "FI",
    "Europe/Dublin": "IE",
    "Europe/Lisbon": "PT",
    "Europe/Athens": "GR",
    "Europe/Moscow": "RU",
    # Americas
    "America/New_York": "US",
    "America/Los_Angeles": "US",
    "America/Chicago": "US",
    "America/Denver": "US",
    "America/Phoenix": "US",
    "America/Toronto": "CA",
    "America/Vancouver": "CA",
    "America/Montreal": "CA",
    "America/Mexico_City": "MX",
    "America/Sao_Paulo": "BR",
    "America/Buenos_Aires": "AR",
    "America/Santiago": "CL",
    "America/Lima": "PE",
    "America/Bogota": "CO",
    # Asia
    "Asia/Tokyo": "JP",
    "Asia/Shanghai": "CN",
    "Asia/Hong_Kong": "HK",
    "Asia/Singapore": "SG",
    "Asia/Seoul": "KR",
    "Asia/Taipei": "TW",
    "Asia/Bangkok": "TH",
    "Asia/Jakarta": "ID",
    "Asia/Manila": "PH",
    "Asia/Kolkata": "IN",
    "Asia/Mumbai": "IN",
    "Asia/Dubai": "AE",
    "Asia/Jerusalem": "IL",
    # Oceania
    "Australia/Sydney": "AU",
    "Australia/Melbourne": "AU",
    "Australia/Brisbane": "AU",
    "Australia/Perth": "AU",
    "Pacific/Auckland": "NZ",
    # Africa
    "Africa/Cairo": "EG",
    "Africa/Johannesburg": "ZA",
    "Africa/Lagos": "NG",
    "Africa/Nairobi": "KE",
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
        # Download Ember CSV (yearly full release long format) and parse it.
        try:
            import urllib.request

            # Ember CSV URL
            url = (
                "https://ember-climate.org/app/uploads/2022/07/yearly_full_release_long_"
                "format-4.csv"
            )

            request = urllib.request.Request(url, headers={"User-Agent": "Poirot/1.0"})

            with urllib.request.urlopen(
                request, timeout=CURL_TIMEOUT_SECONDS
            ) as response:
                csv_data = response.read().decode("utf-8")

                with self._co2_factors_mutex:
                    self.parse_csv_data(csv_data)
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
            # Try direct match
            if country_code in self._co2_factors_by_country:
                return self._co2_factors_by_country[country_code]

            # Try lowercase
            lower = country_code.lower()
            if lower in self._co2_factors_by_country:
                return self._co2_factors_by_country[lower]

            # Try uppercase
            upper = country_code.upper()
            if upper in self._co2_factors_by_country:
                return self._co2_factors_by_country[upper]

            return DEFAULT_CO2_FACTOR_KG_PER_KWH

    def get_country_from_timezone(self, timezone: str) -> str:
        """
        Get country code from timezone.

        Args:
            timezone: Timezone string (e.g., "Europe/Madrid").

        Returns:
            ISO 2-letter country code.
        """
        # Exact map
        if timezone in TIMEZONE_TO_COUNTRY:
            return TIMEZONE_TO_COUNTRY[timezone]

        # Try partial matches
        for tz, country in TIMEZONE_TO_COUNTRY.items():
            if timezone.find(tz) != -1 or tz.find(timezone) != -1:
                return country

        return "UNKNOWN"

    def get_system_timezone(self) -> str:
        """
        Get the current system timezone.

        Returns:
            Timezone string.
        """
        # Try to read from /etc/timezone
        try:
            with open("/etc/timezone", "r") as f:
                tz = f.read().strip()
                if tz:
                    return tz
        except OSError:
            pass

        # Try to read from /etc/localtime symlink (extract zoneinfo path)
        try:
            link = os.readlink("/etc/localtime")
            if "zoneinfo/" in link:
                return link.split("zoneinfo/", 1)[1]
        except OSError:
            pass

        # Fallback to TZ environment variable
        tz_env = os.getenv("TZ")
        if tz_env:
            return tz_env

        return "UTC"

    def parse_csv_data(self, csv_data: str) -> None:
        """
        Parse CSV data and populate self._co2_factors_by_country.

        The CSV parser tries to find columns with headers containing
        'country'/'iso' and 'co2'/'carbon'/'intensity'. Values larger
        than 10 are assumed to be g/kWh and are converted to kg/kWh.
        """
        reader = csv.reader(csv_data.splitlines())
        rows = list(reader)
        if not rows:
            return

        # Find header indices
        headers = [h.strip().lower() for h in rows[0]]
        country_idx = -1
        co2_idx = -1
        for i, h in enumerate(headers):
            if "country" in h or "iso" in h:
                country_idx = i
            if "co2" in h or "carbon" in h or "intensity" in h:
                co2_idx = i

        if country_idx < 0 or co2_idx < 0:
            return

        for row in rows[1:]:
            if len(row) <= max(country_idx, co2_idx):
                continue
            country = row[country_idx].strip()
            co2_str = row[co2_idx].strip()
            if not country or not co2_str:
                continue
            try:
                co2_val = float(co2_str)
                # Convert from g/kWh to kg/kWh if necessary
                if co2_val > 10.0:
                    co2_val /= 1000.0
                self._co2_factors_by_country[country] = co2_val
            except Exception:
                continue

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
