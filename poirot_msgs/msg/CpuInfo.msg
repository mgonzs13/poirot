uint8 RAPL_TDP_TYPE=1
uint8 HWMON_TDP_TYPE=2
uint8 THERMAL_POWER_TDP_TYPE=3
uint8 CPU_CORES_FREQUENCY_TYPE=4
uint8 CPU_CORES_TYPE=5


string model                    # CPU model name
int32 cores                     # Number of CPU cores

float64 tdp_watts               # CPU TDP in watts
uint8 tdp_watts_type            # Type of CPU TDP value