uint8 INTEL_RAPL_TDP_TYPE=1
uint8 AMD_RAPL_TDP_TYPE=2
uint8 HWMON_RAPL_TDP_TYPE=3
uint8 THERMAL_POWER_TDP_TYPE=4
uint8 CPU_CORES_FREQUENCY_TYPE=5
uint8 CPU_CORES_TYPE=6


string model                    # CPU model name
int32 cores                     # Number of CPU cores

bool rapl_available             # Whether RAPL energy readings are available
float64 tdp_watts               # CPU TDP in watts
uint8 tdp_watts_type            # Type of CPU TDP value