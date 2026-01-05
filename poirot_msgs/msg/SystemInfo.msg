uint8 INTEL_RAPL_TDP_TYPE=1
uint8 AMD_RAPL_TDP_TYPE=2
uint8 HWMON_RAPL_TDP_TYPE=3
uint8 THERMAL_POWER_TDP_TYPE=4
uint8 CPU_CORES_FREQUENCY_TYPE=5
uint8 CPU_CORES_TYPE=6


string os_name                      # Operating system name
string os_version                   # Operating system version
string hostname                     # Name of the host machine

string cpu_model                    # CPU model name
int32 cpu_cores                     # Number of CPU cores
int64 mem_total_kb                  # Total system memory

bool rapl_available                 # Whether RAPL energy readings are available
float64 cpu_tdp_watts               # CPU TDP in watts
uint8 cpu_tdp_watts_type            # Type of CPU TDP value
string country_code                 # Country code derived from timezone
float64 co2_factor_kg_per_kwh       # CO2 factor (kg/kWh) for the region
