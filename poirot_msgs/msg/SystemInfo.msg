string os_name                      # Operating system name
string os_version                   # Operating system version
string hostname                     # Name of the host machine

int64 mem_total_kb                  # Total system memory
CpuInfo cpu_info                    # CPU information
GpuInfo gpu_info                    # GPU information

# CO2 info
string country_code                 # Country code derived from timezone
float64 co2_factor_kg_per_kwh       # CO2 factor (kg/kWh) for the region
