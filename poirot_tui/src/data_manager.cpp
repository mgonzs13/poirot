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

#include <algorithm>
#include <chrono>

#include "poirot_tui/data_manager.hpp"

using namespace poirot_tui;

DataManager::DataManager() : start_time_(0.0), first_message_(true) {}

void DataManager::process_profiling_data(
    const poirot_msgs::msg::ProfilingData::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  // Calculate timestamp
  double timestamp = static_cast<double>(msg->timestamp.sec) +
                     static_cast<double>(msg->timestamp.nanosec) / 1e9;

  if (this->first_message_) {
    this->start_time_ = timestamp;
    this->first_message_ = false;
  }

  double relative_time = timestamp - this->start_time_;

  FunctionRow row;
  row.pid = msg->process_info.pid;
  row.function_name = msg->function.name;
  row.call_count = msg->function.call_count;
  row.wall_time_us = msg->function.call.data.wall_time_us;
  row.cpu_time_us = msg->function.call.data.cpu_time_us;
  row.mem_kb = msg->function.call.data.mem_kb;
  row.gpu_mem_kb = msg->function.call.data.gpu_mem_kb;
  row.io_read_bytes = msg->function.call.data.io_read_bytes;
  row.io_write_bytes = msg->function.call.data.io_write_bytes;
  row.ctx_switches = msg->function.call.data.ctx_switches;
  row.cpu_energy_uj = msg->function.call.data.cpu_energy_uj;
  row.gpu_energy_uj = msg->function.call.data.gpu_energy_uj;
  row.energy_uj = msg->function.call.data.total_energy_uj;
  row.co2_ug = msg->function.call.data.co2_ug;
  row.gpu_temp_c = msg->function.call.data.gpu_temp_c;
  row.last_update_time = relative_time;

  std::string key = row.get_key();
  this->function_rows_[key] = row;

  // Add to history
  DataPoint dp;
  dp.timestamp = relative_time;
  dp.wall_time_us = msg->function.call.data.wall_time_us;
  dp.cpu_time_us = msg->function.call.data.cpu_time_us;
  dp.mem_kb = msg->function.call.data.mem_kb;
  dp.gpu_mem_kb = msg->function.call.data.gpu_mem_kb;
  dp.io_read_bytes = msg->function.call.data.io_read_bytes;
  dp.io_write_bytes = msg->function.call.data.io_write_bytes;
  dp.ctx_switches = msg->function.call.data.ctx_switches;
  dp.cpu_energy_uj = msg->function.call.data.cpu_energy_uj;
  dp.gpu_energy_uj = msg->function.call.data.gpu_energy_uj;
  dp.energy_uj = msg->function.call.data.total_energy_uj;
  dp.co2_ug = msg->function.call.data.co2_ug;
  dp.gpu_temp_c = msg->function.call.data.gpu_temp_c;

  auto &history = this->function_history_[key];
  history.push_back(dp);

  // Limit history size
  while (history.size() > MAX_HISTORY_SIZE) {
    history.pop_front();
  }
}

std::vector<FunctionRow> DataManager::get_sorted_rows(SortColumn column,
                                                      bool ascending) const {
  std::lock_guard<std::mutex> lock(this->mutex_);

  std::vector<FunctionRow> rows;
  rows.reserve(this->function_rows_.size());

  for (const auto &pair : this->function_rows_) {
    rows.push_back(pair.second);
  }

  auto compare = [column, ascending](const FunctionRow &a,
                                     const FunctionRow &b) {
    bool result = false;
    switch (column) {
    case SortColumn::PID:
      result = a.pid < b.pid;
      break;
    case SortColumn::FUNCTION_NAME:
      result = a.function_name < b.function_name;
      break;
    case SortColumn::CALL_COUNT:
      result = a.call_count < b.call_count;
      break;
    case SortColumn::WALL_TIME:
      result = a.wall_time_us < b.wall_time_us;
      break;
    case SortColumn::CPU_TIME:
      result = a.cpu_time_us < b.cpu_time_us;
      break;
    case SortColumn::MEMORY:
      result = a.mem_kb < b.mem_kb;
      break;
    case SortColumn::GPU_MEMORY:
      result = a.gpu_mem_kb < b.gpu_mem_kb;
      break;
    case SortColumn::GPU_TEMP:
      result = a.gpu_temp_c < b.gpu_temp_c;
      break;
    case SortColumn::IO_READ:
      result = a.io_read_bytes < b.io_read_bytes;
      break;
    case SortColumn::IO_WRITE:
      result = a.io_write_bytes < b.io_write_bytes;
      break;
    case SortColumn::CTX_SWITCHES:
      result = a.ctx_switches < b.ctx_switches;
      break;
    case SortColumn::CPU_ENERGY:
      result = a.cpu_energy_uj < b.cpu_energy_uj;
      break;
    case SortColumn::GPU_ENERGY:
      result = a.gpu_energy_uj < b.gpu_energy_uj;
      break;
    case SortColumn::ENERGY:
      result = a.energy_uj < b.energy_uj;
      break;
    case SortColumn::CO2:
      result = a.co2_ug < b.co2_ug;
      break;
    }
    return ascending ? result : !result;
  };

  std::sort(rows.begin(), rows.end(), compare);
  return rows;
}

std::vector<DataPoint>
DataManager::get_history(const std::string &function_key) const {
  std::lock_guard<std::mutex> lock(this->mutex_);

  auto it = this->function_history_.find(function_key);
  if (it != this->function_history_.end()) {
    return std::vector<DataPoint>(it->second.begin(), it->second.end());
  }
  return {};
}

std::vector<std::string> DataManager::get_all_function_keys() const {
  std::lock_guard<std::mutex> lock(this->mutex_);

  std::vector<std::string> keys;
  keys.reserve(this->function_rows_.size());

  for (const auto &pair : this->function_rows_) {
    keys.push_back(pair.first);
  }
  return keys;
}

bool DataManager::is_function_enabled(const std::string &function_key) const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->enabled_functions_.find(function_key) !=
         this->enabled_functions_.end();
}

void DataManager::enable_function(const std::string &function_key) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->enabled_functions_.insert(function_key);
}

void DataManager::disable_function(const std::string &function_key) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->enabled_functions_.erase(function_key);
}

void DataManager::toggle_function(const std::string &function_key) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  auto it = this->enabled_functions_.find(function_key);
  if (it != this->enabled_functions_.end()) {
    this->enabled_functions_.erase(it);
  } else {
    this->enabled_functions_.insert(function_key);
  }
}

std::set<std::string> DataManager::get_enabled_functions() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->enabled_functions_;
}

void DataManager::clear() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->function_rows_.clear();
  this->function_history_.clear();
  this->enabled_functions_.clear();
  this->first_message_ = true;
  this->start_time_ = 0.0;
}

size_t DataManager::get_function_count() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->function_rows_.size();
}
