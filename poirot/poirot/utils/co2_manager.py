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
import urllib.request
from typing import Dict

# Default CO2 factor in kg CO2 per kWh (global average)
DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.436
# Timeout for CURL requests in seconds
CURL_TIMEOUT_SECONDS = 3
# URL for downloading CO2 intensity data
CO2_INTENSITY_DATA_URL = (
    "https://ourworldindata.org/grapher/carbon-intensity-electricity.csv"
)


class Co2Manager:
    """
    Class for managing CO2 emission factors.

    Provides methods for downloading CO2 factors from online sources
    and looking up factors by country.
    """

    def __init__(self) -> None:
        """Constructor."""
        # Map of country code to CO2 factor
        self._co2_factors_by_country: Dict[str, float] = {}
        # Mutex for thread-safe access to CO2 factors
        self._co2_factors_mutex = threading.Lock()
        # Flag indicating if CO2 factors have been loaded
        self._co2_factors_loaded = False
        # Cache for timezone to country mapping
        self._timezone_to_country: Dict[str, str] = {}
        self._timezone_map_loaded = False
        # ISO2 to ISO3 mapping
        self._iso2_to_iso3: Dict[str, str] = {}
        # Flag indicating if ISO mapping has been loaded
        self._iso_map_loaded = False

    def download_factors(self) -> bool:
        """
        Download CO2 factors from online source.

        Returns:
            True if download succeeded, False otherwise.
        """
        # Download ISO mapping first
        self._load_iso_mapping()

        # Download carbon intensity data from Our World in Data
        try:
            # Our World in Data carbon intensity electricity CSV
            request = urllib.request.Request(
                CO2_INTENSITY_DATA_URL, headers={"User-Agent": "Poirot/1.0"}
            )

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
            country_code: ISO 2-letter or 3-letter country code.

        Returns:
            CO2 factor in kg CO2 per kWh.
        """
        with self._co2_factors_mutex:
            # Try direct match (handles both 2-letter and 3-letter codes)
            if country_code in self._co2_factors_by_country:
                return self._co2_factors_by_country[country_code]

            # Try converting 2-letter to 3-letter codes
            if len(country_code) == 2 and self._iso_map_loaded:
                iso3_code = self._iso2_to_iso3.get(country_code.upper())
                if iso3_code and iso3_code in self._co2_factors_by_country:
                    return self._co2_factors_by_country[iso3_code]

            # Try 3-letter to 2-letter conversion (less common)
            if len(country_code) == 3 and self._iso_map_loaded:
                for iso2, iso3 in self._iso2_to_iso3.items():
                    if iso3 == country_code.upper():
                        return self._co2_factors_by_country.get(
                            iso3, DEFAULT_CO2_FACTOR_KG_PER_KWH
                        )

            return DEFAULT_CO2_FACTOR_KG_PER_KWH

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

    def parse_csv_data(self, csv_data: str) -> None:
        """
        Parse CSV data and populate co2_factors_by_country.

        The CSV format is: Entity,Code,Year,Lifecycle carbon intensity of electricity - gCO2/kWh
        We use the most recent year available for each country.
        Values are in gCO2/kWh and are converted to kgCO2/kWh.
        """
        reader = csv.reader(csv_data.splitlines())
        rows = list(reader)
        if not rows:
            return

        # Skip header
        data_rows = rows[1:]

        # Group by country code and find the most recent year
        country_data = {}
        for row in data_rows:
            if len(row) < 4:
                continue
            entity, code, year_str, intensity_str = row[:4]

            # Skip if no country code or invalid data
            if not code or code == "" or intensity_str == "":
                continue

            try:
                year = int(year_str)
                intensity = float(intensity_str)
                # Convert gCO2/kWh to kgCO2/kWh
                intensity_kg = intensity / 1000.0

                # Keep the most recent year for each country
                if code not in country_data or year > country_data[code][0]:
                    country_data[code] = (year, intensity_kg)
            except ValueError:
                continue

        # Store the most recent values
        for code, (year, intensity_kg) in country_data.items():
            self._co2_factors_by_country[code] = intensity_kg

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
