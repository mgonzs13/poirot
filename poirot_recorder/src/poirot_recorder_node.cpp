// Copyright 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <iomanip>
#include <string>

#include "poirot_recorder/poirot_recorder_node.hpp"

using namespace poirot_recorder;

PoirotRecorderNode::PoirotRecorderNode()
    : Node("poirot_recorder"), record_count_(0) {
  // Declare parameter for CSV file path
  this->declare_parameter<std::string>("csv_file_path", "poirot_data.csv");

  // Get the CSV file path
  this->csv_file_path_ = this->get_parameter("csv_file_path").as_string();

  // Create parent directories if they don't exist
  std::filesystem::path file_path(this->csv_file_path_);
  if (file_path.has_parent_path()) {
    std::filesystem::create_directories(file_path.parent_path());
  }

  // Open CSV file and write header
  this->csv_file_.open(this->csv_file_path_, std::ios::out | std::ios::trunc);
  if (!this->csv_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s",
                 this->csv_file_path_.c_str());
    throw std::runtime_error("Failed to open CSV file");
  }

  // Write CSV header
  this->write_csv_header();

  // Create subscription to /poirot/data topic
  this->subscription_ =
      this->create_subscription<poirot_msgs::msg::ProfilingData>(
          "/poirot/data", 10,
          std::bind(&PoirotRecorderNode::data_callback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Poirot recorder started. Saving data to: %s",
              this->csv_file_path_.c_str());
}

PoirotRecorderNode::~PoirotRecorderNode() {
  std::lock_guard<std::mutex> lock(this->csv_mutex_);
  if (this->csv_file_.is_open()) {
    this->csv_file_.close();
  }
  RCLCPP_INFO(this->get_logger(), "Poirot recorder stopped. Data saved to: %s",
              this->csv_file_path_.c_str());
}

void PoirotRecorderNode::write_csv_header() {
  this->csv_file_
      << "timestamp_sec,timestamp_nanosec,"
      << "os_name,os_version,hostname,cpu_model,cpu_cores,mem_total_kb,"
      << "rapl_available,cpu_tdp_watts,cpu_tdp_watts_type,"
      << "gpu_available,gpu_model,gpu_vendor,gpu_mem_total_kb,"
      << "gpu_tdp_watts,gpu_tdp_watts_type,gpu_power_monitoring,"
      << "country_code,co2_factor_loaded,co2_factor_kg_per_kwh,"
      << "process_pid,process_cpu_percent,thread_cpu_percent,process_threads,"
      << "function_name,file,line,call_count,wall_time_us,"
      << "cpu_time_us,process_cpu_time_us,system_cpu_time_us,"
      << "memory_kb,io_read_bytes,io_write_bytes,"
      << "ctx_switches,cpu_energy_uj,cpu_total_energy_uj,"
      << "gpu_mem_kb,gpu_energy_uj,gpu_utilization_percent,"
      << "total_energy_uj,co2_ug\n";
}

void PoirotRecorderNode::data_callback(
    const poirot_msgs::msg::ProfilingData::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->csv_mutex_);

  if (!this->csv_file_.is_open()) {
    RCLCPP_WARN(this->get_logger(), "CSV file is not open, cannot record data");
    return;
  }

  // Write data row
  this->csv_file_ << msg->timestamp.sec << "," << msg->timestamp.nanosec
                  << ","

                  // System Info
                  << this->escape_csv(msg->system_info.os_name) << ","
                  << this->escape_csv(msg->system_info.os_version) << ","
                  << this->escape_csv(msg->system_info.hostname) << ","
                  << this->escape_csv(msg->system_info.cpu_info.model) << ","
                  << msg->system_info.cpu_info.cores << ","
                  << msg->system_info.mem_total_kb << ","
                  << (msg->system_info.cpu_info.rapl_available ? "1" : "0")
                  << "," << msg->system_info.cpu_info.tdp_watts << ","
                  << static_cast<int>(msg->system_info.cpu_info.tdp_watts_type)
                  << ","

                  // GPU Info
                  << (msg->system_info.gpu_info.available ? "1" : "0") << ","
                  << this->escape_csv(msg->system_info.gpu_info.model) << ","
                  << this->escape_csv(msg->system_info.gpu_info.vendor) << ","
                  << msg->system_info.gpu_info.mem_total_kb << ","
                  << msg->system_info.gpu_info.tdp_watts << ","
                  << static_cast<int>(msg->system_info.gpu_info.tdp_watts_type)
                  << ","
                  << (msg->system_info.gpu_info.power_monitoring ? "1" : "0")
                  << ","

                  // CO2 Info
                  << this->escape_csv(msg->system_info.co2_info.country_code)
                  << ","
                  << (msg->system_info.co2_info.co2_factor_loaded ? "1" : "0")
                  << "," << msg->system_info.co2_info.co2_factor_kg_per_kwh
                  << ","

                  // Process Info
                  << msg->process_info.pid << ","
                  << msg->process_info.cpu_percent << ","
                  << msg->process_info.thread_cpu_percent << ","
                  << std::to_string(msg->process_info.threads)
                  << ","

                  // Function Call Info
                  << this->escape_csv(msg->function.name) << ","
                  << this->escape_csv(msg->function.file) << ","
                  << msg->function.line << "," << msg->function.call_count
                  << ","

                  // CPU metrics
                  << msg->function.call.data.wall_time_us << ","
                  << msg->function.call.data.cpu_time_us << ","
                  << msg->function.call.data.process_cpu_time_us << ","
                  << msg->function.call.data.system_cpu_time_us << ","
                  << msg->function.call.data.mem_kb << ","
                  << msg->function.call.data.io_read_bytes << ","
                  << msg->function.call.data.io_write_bytes << ","
                  << msg->function.call.data.ctx_switches << ","
                  << msg->function.call.data.cpu_energy_uj << ","
                  << msg->function.call.data.cpu_total_energy_uj
                  << ","

                  // GPU metrics
                  << msg->function.call.data.gpu_mem_kb << ","
                  << msg->function.call.data.gpu_energy_uj << ","
                  << msg->function.call.data.gpu_utilization_percent
                  << ","

                  // Total energy
                  << msg->function.call.data.total_energy_uj << ","
                  << msg->function.call.data.co2_ug << "\n";

  this->csv_file_.flush(); // Ensure data is written to disk

  this->record_count_++;
  if (this->record_count_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "Recorded %ld profiling data entries",
                this->record_count_);
  }
}

std::string PoirotRecorderNode::escape_csv(const std::string &field) {
  if (field.find(',') != std::string::npos ||
      field.find('"') != std::string::npos ||
      field.find('\n') != std::string::npos) {
    std::string escaped = "\"";
    for (char c : field) {
      if (c == '"') {
        escaped += "\"\""; // Escape quotes by doubling them
      } else {
        escaped += c;
      }
    }
    escaped += "\"";
    return escaped;
  }
  return field;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<PoirotRecorderNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("poirot_recorder"), "Exception: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
