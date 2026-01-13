uint8 NVIDIA_SMI_TDP_TYPE=1
uint8 AMD_ROCM_TDP_TYPE=2
uint8 SYSFS_TDP_TYPE=3
uint8 ESTIMATED_TDP_TYPE=4

string model                    # GPU model name
string vendor                   # GPU vendor (NVIDIA, AMD, Intel)
int32 index                     # GPU device index
int64 mem_total_kb              # Total GPU memory in KB
float64 tdp_watts               # GPU TDP in watts
uint8 tdp_watts_type            # Type of GPU TDP value detection method
bool available                  # Whether GPU is available
bool power_monitoring           # Whether GPU power monitoring is available
