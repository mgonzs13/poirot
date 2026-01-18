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
import json
import csv
import threading
import urllib.request
from typing import Dict
from dataclasses import dataclass

# Timeout for requests in seconds
CURL_TIMEOUT_SECONDS = 3
# Ember API base URL
EMBER_API_BASE_URL = "https://api.ember-energy.org/v1"


@dataclass
class Co2Info:
    """Structure to hold CO2 information."""

    country: str = ""
    co2_factor_loaded: bool = False
    co2_factor_kg_per_kwh: float = 0.0


class Co2Manager:
    """
    Class for managing CO2 emission factors.

    Provides methods for downloading CO2 factors from online sources
    and looking up factors by country.
    """

    def __init__(self) -> None:
        """Constructor."""
        # Mutex for thread-safe access to CO2 factors
        self._co2_factors_mutex = threading.Lock()
        # Cache for timezone to country mapping
        self._timezone_to_country: Dict[str, str] = {}
        # Flag indicating if timezone mapping has been loaded
        self._timezone_map_loaded = False
        # ISO2 to ISO3 mapping
        self._iso2_to_iso3: Dict[str, str] = {}
        # Flag indicating if ISO mapping has been loaded
        self._iso_map_loaded = False

        self._load_iso_mapping()

    def get_co2_info(self) -> Co2Info:
        """
        Get CO2 factor based on system timezone.

        Returns:
            CO2 factor information as a Co2Info object.
        """
        timezone = self.get_system_timezone()
        country_code = self.get_country_from_timezone(timezone)
        return self._get_co2_factor(country_code)

    def _get_co2_factor(self, country: str) -> Co2Info:
        """
        Download CO2 factor for a specific country from Ember API.
        Args:
            country: Country name (e.g., "Spain").
        Returns:
            CO2 factor information as a Co2Info object.
        """

        co2_info = Co2Info()
        co2_info.country = country

        # Get API key from environment
        api_key = os.getenv("EMBER_KEY")
        if not api_key:
            return co2_info

        try:
            country = self._iso2_to_iso3.get(country)
        except KeyError:
            return co2_info

        # Fetch CO2 intensity data from Ember API
        try:
            url = f"{EMBER_API_BASE_URL}/carbon-intensity/monthly?api_key={api_key}&entity_code={country}"
            request = urllib.request.Request(url, headers={"User-Agent": "Poirot/1.0"})

            with urllib.request.urlopen(
                request, timeout=CURL_TIMEOUT_SECONDS
            ) as response:
                data = json.loads(response.read().decode("utf-8"))

            with self._co2_factors_mutex:
                try:
                    factor = data["data"][-1]["emissions_intensity_gco2_per_kwh"] / 1000.0
                except (KeyError, IndexError):
                    return co2_info

            co2_info.co2_factor_loaded = True
            co2_info.co2_factor_kg_per_kwh = factor
            return co2_info

        except Exception:
            return co2_info

    def get_country_from_timezone(self, timezone: str) -> str:
        """
        Get country code from timezone by reading from system zone.tab file.

        Args:
            timezone: Timezone string (e.g., "Europe/Madrid").

        Returns:
            ISO 2-letter country code.
        """
        # Load timezone mapping from zone.tab if not already loaded
        if not self._timezone_map_loaded:
            self._load_timezone_mapping()

        # Try exact match
        if timezone in self._timezone_to_country:
            return self._timezone_to_country[timezone]

        # Try partial matches
        for tz, country in self._timezone_to_country.items():
            if timezone in tz or tz in timezone:
                return country

        return "UNKNOWN"

    def _load_timezone_mapping(self) -> None:
        """
        Load timezone to country mapping from system zone.tab file.
        Format: <country_code> <coordinates> <timezone> [<comments>]
        """
        zone_tab_paths = [
            "/usr/share/zoneinfo/zone.tab",
            "/usr/share/zoneinfo/zone1970.tab",
            "/usr/share/lib/zoneinfo/tab/zone_sun.tab",
        ]

        for zone_tab_path in zone_tab_paths:
            try:
                with open(zone_tab_path, "r") as f:
                    for line in f:
                        line = line.strip()
                        # Skip comments and empty lines
                        if not line or line.startswith("#"):
                            continue

                        # Split by whitespace
                        parts = line.split()
                        if len(parts) < 3:
                            continue

                        country_code = parts[0]
                        timezone = parts[2]
                        self._timezone_to_country[timezone] = country_code

                self._timezone_map_loaded = True
                return
            except OSError:
                continue

        # If we couldn't load from any file, mark as loaded anyway to avoid repeated attempts
        self._timezone_map_loaded = True

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

    def _load_iso_mapping(self) -> None:
        """
        Load ISO2 to ISO3 mapping from installed CSV file.
        """
        try:
            import ament_index_python

            package_path = ament_index_python.get_package_share_directory("poirot")
            csv_path = os.path.join(package_path, "iso_country_codes.csv")
            with open(csv_path, "r", encoding="utf-8") as f:
                csv_data = f.read()
            self._parse_iso_csv(csv_data)
            self._iso_map_loaded = True
        except Exception:
            self._iso_map_loaded = False

    def _parse_iso_csv(self, csv_data: str) -> None:
        """
        Parse ISO CSV data and populate iso2_to_iso3.

        Expected columns: name,alpha-2,alpha-3,country-code,iso_3166-2,region,sub-region
        """
        reader = csv.reader(csv_data.splitlines())
        rows = list(reader)
        if not rows:
            return

        # Skip header
        data_rows = rows[1:]

        for row in data_rows:
            if len(row) < 3:
                continue
            alpha2 = row[1].strip()
            alpha3 = row[2].strip()
            if alpha2 and alpha3:
                self._iso2_to_iso3[alpha2] = alpha3
