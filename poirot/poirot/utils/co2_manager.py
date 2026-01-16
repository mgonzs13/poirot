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
DEFAULT_CO2_FACTOR_KG_PER_KWH = 0.5
# Timeout for CURL requests in seconds
CURL_TIMEOUT_SECONDS = 3


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

    def download_factors(self) -> bool:
        """
        Download CO2 factors from online source.

        Returns:
            True if download succeeded, False otherwise.
        """
        # Download carbon intensity data from Our World in Data
        try:
            import urllib.request

            # Our World in Data carbon intensity electricity CSV
            url = "https://ourworldindata.org/grapher/carbon-intensity-electricity.csv"

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
            country_code: ISO 2-letter or 3-letter country code.

        Returns:
            CO2 factor in kg CO2 per kWh.
        """
        with self._co2_factors_mutex:
            # Try direct match (handles both 2-letter and 3-letter codes)
            if country_code in self._co2_factors_by_country:
                return self._co2_factors_by_country[country_code]

            # Try converting 2-letter to 3-letter codes
            iso2_to_iso3 = {
                "AF": "AFG",
                "AL": "ALB",
                "DZ": "DZA",
                "AS": "ASM",
                "AD": "AND",
                "AO": "AGO",
                "AI": "AIA",
                "AQ": "ATA",
                "AG": "ATG",
                "AR": "ARG",
                "AM": "ARM",
                "AW": "ABW",
                "AU": "AUS",
                "AT": "AUT",
                "AZ": "AZE",
                "BS": "BHS",
                "BH": "BHR",
                "BD": "BGD",
                "BB": "BRB",
                "BY": "BLR",
                "BE": "BEL",
                "BZ": "BLZ",
                "BJ": "BEN",
                "BM": "BMU",
                "BT": "BTN",
                "BO": "BOL",
                "BA": "BIH",
                "BW": "BWA",
                "BV": "BVT",
                "BR": "BRA",
                "IO": "IOT",
                "BN": "BRN",
                "BG": "BGR",
                "BF": "BFA",
                "BI": "BDI",
                "KH": "KHM",
                "CM": "CMR",
                "CA": "CAN",
                "CV": "CPV",
                "KY": "CYM",
                "CF": "CAF",
                "TD": "TCD",
                "CL": "CHL",
                "CN": "CHN",
                "CX": "CXR",
                "CC": "CCK",
                "CO": "COL",
                "KM": "COM",
                "CG": "COG",
                "CD": "COD",
                "CK": "COK",
                "CR": "CRI",
                "CI": "CIV",
                "HR": "HRV",
                "CU": "CUB",
                "CY": "CYP",
                "CZ": "CZE",
                "DK": "DNK",
                "DJ": "DJI",
                "DM": "DMA",
                "DO": "DOM",
                "EC": "ECU",
                "EG": "EGY",
                "SV": "SLV",
                "GQ": "GNQ",
                "ER": "ERI",
                "EE": "EST",
                "ET": "ETH",
                "FK": "FLK",
                "FO": "FRO",
                "FJ": "FJI",
                "FI": "FIN",
                "FR": "FRA",
                "GF": "GUF",
                "PF": "PYF",
                "TF": "ATF",
                "GA": "GAB",
                "GM": "GMB",
                "GE": "GEO",
                "DE": "DEU",
                "GH": "GHA",
                "GI": "GIB",
                "GR": "GRC",
                "GL": "GRL",
                "GD": "GRD",
                "GP": "GLP",
                "GU": "GUM",
                "GT": "GTM",
                "GG": "GGY",
                "GN": "GIN",
                "GW": "GNB",
                "GY": "GUY",
                "HT": "HTI",
                "HM": "HMD",
                "VA": "VAT",
                "HN": "HND",
                "HK": "HKG",
                "HU": "HUN",
                "IS": "ISL",
                "IN": "IND",
                "ID": "IDN",
                "IR": "IRN",
                "IQ": "IRQ",
                "IE": "IRL",
                "IM": "IMN",
                "IL": "ISR",
                "IT": "ITA",
                "JM": "JAM",
                "JP": "JPN",
                "JE": "JEY",
                "JO": "JOR",
                "KZ": "KAZ",
                "KE": "KEN",
                "KI": "KIR",
                "KP": "PRK",
                "KR": "KOR",
                "KW": "KWT",
                "KG": "KGZ",
                "LA": "LAO",
                "LV": "LVA",
                "LB": "LBN",
                "LS": "LSO",
                "LR": "LBR",
                "LY": "LBY",
                "LI": "LIE",
                "LT": "LTU",
                "LU": "LUX",
                "MO": "MAC",
                "MK": "MKD",
                "MG": "MDG",
                "MW": "MWI",
                "MY": "MYS",
                "MV": "MDV",
                "ML": "MLI",
                "MT": "MLT",
                "MH": "MHL",
                "MQ": "MTQ",
                "MR": "MRT",
                "MU": "MUS",
                "YT": "MYT",
                "MX": "MEX",
                "FM": "FSM",
                "MD": "MDA",
                "MC": "MCO",
                "MN": "MNG",
                "ME": "MNE",
                "MS": "MSR",
                "MA": "MAR",
                "MZ": "MOZ",
                "MM": "MMR",
                "NA": "NAM",
                "NR": "NRU",
                "NP": "NPL",
                "NL": "NLD",
                "NC": "NCL",
                "NZ": "NZL",
                "NI": "NIC",
                "NE": "NER",
                "NG": "NGA",
                "NU": "NIU",
                "NF": "NFK",
                "MP": "MNP",
                "NO": "NOR",
                "OM": "OMN",
                "PK": "PAK",
                "PW": "PLW",
                "PS": "PSE",
                "PA": "PAN",
                "PG": "PNG",
                "PY": "PRY",
                "PE": "PER",
                "PH": "PHL",
                "PN": "PCN",
                "PL": "POL",
                "PT": "PRT",
                "PR": "PRI",
                "QA": "QAT",
                "RE": "REU",
                "RO": "ROU",
                "RU": "RUS",
                "RW": "RWA",
                "BL": "BLM",
                "SH": "SHN",
                "KN": "KNA",
                "LC": "LCA",
                "MF": "MAF",
                "PM": "SPM",
                "VC": "VCT",
                "WS": "WSM",
                "SM": "SMR",
                "ST": "STP",
                "SA": "SAU",
                "SN": "SEN",
                "RS": "SRB",
                "SC": "SYC",
                "SL": "SLE",
                "SG": "SGP",
                "SX": "SXM",
                "SK": "SVK",
                "SI": "SVN",
                "SB": "SLB",
                "SO": "SOM",
                "ZA": "ZAF",
                "GS": "SGS",
                "SS": "SSD",
                "ES": "ESP",
                "LK": "LKA",
                "SD": "SDN",
                "SR": "SUR",
                "SJ": "SJM",
                "SZ": "SWZ",
                "SE": "SWE",
                "CH": "CHE",
                "SY": "SYR",
                "TW": "TWN",
                "TJ": "TJK",
                "TZ": "TZA",
                "TH": "THA",
                "TL": "TLS",
                "TG": "TGO",
                "TK": "TKL",
                "TO": "TON",
                "TT": "TTO",
                "TN": "TUN",
                "TR": "TUR",
                "TM": "TKM",
                "TC": "TCA",
                "TV": "TUV",
                "UG": "UGA",
                "UA": "UKR",
                "AE": "ARE",
                "GB": "GBR",
                "US": "USA",
                "UM": "UMI",
                "UY": "URY",
                "UZ": "UZB",
                "VU": "VUT",
                "VE": "VEN",
                "VN": "VNM",
                "VG": "VGB",
                "VI": "VIR",
                "WF": "WLF",
                "EH": "ESH",
                "YE": "YEM",
                "ZM": "ZMB",
                "ZW": "ZWE",
            }

            # Try 2-letter to 3-letter conversion
            if len(country_code) == 2:
                iso3_code = iso2_to_iso3.get(country_code.upper())
                if iso3_code and iso3_code in self._co2_factors_by_country:
                    return self._co2_factors_by_country[iso3_code]

            # Try 3-letter to 2-letter conversion
            if len(country_code) == 3:
                for iso2, iso3 in iso2_to_iso3.items():
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
