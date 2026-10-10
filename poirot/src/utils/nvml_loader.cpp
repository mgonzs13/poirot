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

#include <dlfcn.h>

#include "poirot/utils/nvml_loader.hpp"

namespace poirot {
namespace utils {

namespace {

constexpr char NVML_LIBRARY[] = "libnvidia-ml.so.1";
constexpr int NVML_SUCCESS = 0;
constexpr int NVML_ERROR_INSUFFICIENT_SIZE = 7;
constexpr unsigned int INITIAL_LIST_CAPACITY = 64;
constexpr unsigned int MAX_LIST_CAPACITY = 4096;
constexpr unsigned int NAME_BUFFER_SIZE = 96;

} // namespace

NvmlLoader::NvmlLoader() = default;

NvmlLoader::~NvmlLoader() {
  // NVML is process-wide: the library handle is intentionally not closed and
  // nvmlShutdown() is not called.
}

bool NvmlLoader::initialize() {
  if (this->available_) {
    return true;
  }

  this->library_handle_ = dlopen(NVML_LIBRARY, RTLD_LAZY | RTLD_LOCAL);
  if (this->library_handle_ == nullptr) {
    return false;
  }

  this->init_ =
      reinterpret_cast<InitFn>(dlsym(this->library_handle_, "nvmlInit_v2"));
  this->get_handle_ = reinterpret_cast<GetHandleFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetHandleByIndex_v2"));
  this->get_name_ = reinterpret_cast<GetNameFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetName"));
  this->get_memory_ = reinterpret_cast<GetMemoryFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetMemoryInfo"));
  this->get_power_ = reinterpret_cast<GetPowerFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetPowerUsage"));
  this->get_limit_ = reinterpret_cast<GetLimitFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetPowerManagementLimit"));
  this->get_util_ = reinterpret_cast<GetUtilFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetUtilizationRates"));
  this->get_running_ = reinterpret_cast<GetRunningFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetComputeRunningProcesses_v2"));
  this->get_proc_util_ = reinterpret_cast<GetProcUtilFn>(
      dlsym(this->library_handle_, "nvmlDeviceGetProcessUtilization"));

  if (this->init_ == nullptr || this->get_handle_ == nullptr ||
      this->get_name_ == nullptr || this->get_memory_ == nullptr ||
      this->get_power_ == nullptr || this->get_limit_ == nullptr ||
      this->get_util_ == nullptr || this->get_running_ == nullptr ||
      this->get_proc_util_ == nullptr) {
    return false;
  }

  if (this->init_() != NVML_SUCCESS) {
    return false;
  }

  if (this->get_handle_(0, &this->device_) != NVML_SUCCESS) {
    return false;
  }

  this->available_ = true;
  return true;
}

std::string NvmlLoader::name() const {
  if (!this->available_) {
    return "";
  }

  char buffer[NAME_BUFFER_SIZE] = {0};
  if (this->get_name_(this->device_, buffer, sizeof(buffer)) != NVML_SUCCESS) {
    return "";
  }

  return std::string(buffer);
}

bool NvmlLoader::read_memory(NvmlMemory &memory) const {
  if (!this->available_) {
    return false;
  }

  return this->get_memory_(this->device_, &memory) == NVML_SUCCESS;
}

bool NvmlLoader::read_power_w(double &power_w) const {
  if (!this->available_) {
    return false;
  }

  unsigned int milliwatts = 0;
  if (this->get_power_(this->device_, &milliwatts) != NVML_SUCCESS) {
    return false;
  }

  power_w = static_cast<double>(milliwatts) / 1000.0;
  return true;
}

bool NvmlLoader::read_power_limit_w(double &power_limit_w) const {
  if (!this->available_) {
    return false;
  }

  unsigned int milliwatts = 0;
  if (this->get_limit_(this->device_, &milliwatts) != NVML_SUCCESS) {
    return false;
  }

  power_limit_w = static_cast<double>(milliwatts) / 1000.0;
  return true;
}

bool NvmlLoader::read_device_utilization(double &percent) const {
  if (!this->available_) {
    return false;
  }

  NvmlUtilization utilization;
  if (this->get_util_(this->device_, &utilization) != NVML_SUCCESS) {
    return false;
  }

  percent = static_cast<double>(utilization.gpu);
  return true;
}

bool NvmlLoader::read_running_processes(
    std::vector<NvmlProcessInfoV2> &processes) const {
  if (!this->available_) {
    return false;
  }

  processes.assign(INITIAL_LIST_CAPACITY, NvmlProcessInfoV2{});
  unsigned int count = INITIAL_LIST_CAPACITY;
  int result = this->get_running_(this->device_, &count, processes.data());

  if (result == NVML_ERROR_INSUFFICIENT_SIZE) {
    if (count == 0 || count > MAX_LIST_CAPACITY) {
      processes.clear();
      return false;
    }
    processes.assign(count, NvmlProcessInfoV2{});
    result = this->get_running_(this->device_, &count, processes.data());
  }

  if (result != NVML_SUCCESS) {
    processes.clear();
    return false;
  }

  processes.resize(count);
  return true;
}

bool NvmlLoader::read_process_utilization(
    std::vector<NvmlProcessUtilizationSample> &samples) const {
  if (!this->available_) {
    return false;
  }

  samples.assign(INITIAL_LIST_CAPACITY, NvmlProcessUtilizationSample{});
  unsigned int count = INITIAL_LIST_CAPACITY;
  int result = this->get_proc_util_(this->device_, samples.data(), &count, 0);

  if (result == NVML_ERROR_INSUFFICIENT_SIZE) {
    if (count == 0 || count > MAX_LIST_CAPACITY) {
      samples.clear();
      return false;
    }
    samples.assign(count, NvmlProcessUtilizationSample{});
    result = this->get_proc_util_(this->device_, samples.data(), &count, 0);
  }

  if (result != NVML_SUCCESS) {
    samples.clear();
    return false;
  }

  samples.resize(count);
  return true;
}

} // namespace utils
} // namespace poirot
