int64 wall_time_us                    # Wall clock time in microseconds
int64 cpu_time_us                     # CPU time in microseconds
int64 process_cpu_time_us             # Process CPU time in microseconds
int64 system_cpu_time_us              # Total system CPU time in microseconds
float64 gpu_utilization_percent       # Percentage of GPU used

int64 mem_kb                          # Memory change during call in KB
int64 gpu_mem_kb                      # GPU memory used during call in KB

int64 io_read_bytes                   # Bytes read during call
int64 io_write_bytes                  # Bytes written during call
int64 ctx_switches                    # Context switches during call

float64 cpu_energy_uj                 # CPU energy consumed in microjoules
float64 cpu_total_energy_uj           # Total CPU energy consumed in microjoules
float64 gpu_energy_uj                 # GPU energy consumed in microjoules
float64 total_energy_uj               # Total energy (CPU + GPU) in microjoules
float64 co2_ug                        # CO2 emissions in micrograms

float64 gpu_temp_c                    # GPU temperature in Celsius
