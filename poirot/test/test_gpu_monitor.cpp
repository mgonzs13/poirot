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

#include <gtest/gtest.h>

#include <vector>

#include "poirot/utils/gpu_monitor.hpp"
#include "poirot/utils/nvml_loader.hpp"

using poirot::utils::GpuMonitor;
using poirot::utils::NvmlLoader;
using poirot::utils::NvmlMemory;
using poirot::utils::NvmlProcessInfoV2;
using poirot::utils::NvmlProcessUtilizationSample;

TEST(NvmlLoaderTest, UninitializedLoaderReturnsNoData) {
  NvmlLoader loader;
  EXPECT_FALSE(loader.available());
  EXPECT_TRUE(loader.name().empty());

  double power_w = 1.0;
  EXPECT_FALSE(loader.read_power_w(power_w));
  double limit_w = 1.0;
  EXPECT_FALSE(loader.read_power_limit_w(limit_w));
  double utilization = 1.0;
  EXPECT_FALSE(loader.read_device_utilization(utilization));

  NvmlMemory memory;
  EXPECT_FALSE(loader.read_memory(memory));
  std::vector<NvmlProcessInfoV2> processes;
  EXPECT_FALSE(loader.read_running_processes(processes));
  std::vector<NvmlProcessUtilizationSample> samples;
  EXPECT_FALSE(loader.read_process_utilization(samples));
}

TEST(NvmlLoaderTest, InitializeDoesNotThrowAndReportsConsistently) {
  NvmlLoader loader;
  bool initialized = loader.initialize();
  EXPECT_EQ(loader.available(), initialized);

  if (initialized) {
    EXPECT_FALSE(loader.name().empty());
    std::vector<NvmlProcessInfoV2> processes;
    loader.read_running_processes(processes); // must not crash on any setup
    double power_w = -1.0;
    if (loader.read_power_w(power_w)) {
      EXPECT_GE(power_w, 0.0);
    }
  } else {
    EXPECT_TRUE(loader.name().empty());
  }
}

TEST(GpuShareTest, NormalizesFreshSamples) {
  NvmlProcessUtilizationSample ours;
  ours.pid = 100;
  ours.time_stamp_us = 1000000;
  ours.sm_util = 50;

  NvmlProcessUtilizationSample other;
  other.pid = 200;
  other.time_stamp_us = 1000000;
  other.sm_util = 50;

  std::vector<NvmlProcessUtilizationSample> samples{ours, other};
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 2000000, samples, {}),
                   0.5);
}

TEST(GpuShareTest, FreshZeroSampleReturnsZero) {
  NvmlProcessUtilizationSample ours;
  ours.pid = 100;
  ours.time_stamp_us = 1000000;
  ours.sm_util = 0;

  std::vector<NvmlProcessUtilizationSample> samples{ours};
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 2000000, samples, {}),
                   0.0);
}

TEST(GpuShareTest, StaleSampleUsesMemoryFallback) {
  NvmlProcessUtilizationSample stale;
  stale.pid = 100;
  stale.time_stamp_us = 1000000;
  stale.sm_util = 90;

  NvmlProcessInfoV2 ours;
  ours.pid = 100;
  ours.used_gpu_memory = 1ULL << 30;

  NvmlProcessInfoV2 other;
  other.pid = 200;
  other.used_gpu_memory = 3ULL << 30;

  std::vector<NvmlProcessUtilizationSample> samples{stale};
  std::vector<NvmlProcessInfoV2> running{ours, other};
  // now - sample = 10 s > 5 s freshness window -> memory fallback: 1/4.
  EXPECT_DOUBLE_EQ(
      GpuMonitor::compute_process_share(100, 11000000, samples, running), 0.25);
}

TEST(GpuShareTest, FutureTimestampIsIgnored) {
  NvmlProcessUtilizationSample future;
  future.pid = 100;
  future.time_stamp_us = 5000000;
  future.sm_util = 90;

  std::vector<NvmlProcessUtilizationSample> samples{future};
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 1000000, samples, {}),
                   0.0);
}

TEST(GpuShareTest, NoDataReturnsZero) {
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 2000000, {}, {}),
                   0.0);
}

TEST(GpuShareTest, MultipleSamplesPerPidUseNewest) {
  NvmlProcessUtilizationSample old_ours;
  old_ours.pid = 100;
  old_ours.time_stamp_us = 1000000;
  old_ours.sm_util = 10;

  NvmlProcessUtilizationSample new_ours;
  new_ours.pid = 100;
  new_ours.time_stamp_us = 2000000;
  new_ours.sm_util = 50;

  NvmlProcessUtilizationSample other;
  other.pid = 200;
  other.time_stamp_us = 2000000;
  other.sm_util = 50;

  std::vector<NvmlProcessUtilizationSample> samples{old_ours, new_ours, other};
  // Newest ours (50) over (50 + 50) = 0.5; a naive sum would double count.
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 3000000, samples, {}),
                   0.5);
}

TEST(GpuShareTest, StaleSamplesExcludedFromNormalizer) {
  NvmlProcessUtilizationSample ours;
  ours.pid = 100;
  ours.time_stamp_us = 10000000;
  ours.sm_util = 40;

  NvmlProcessUtilizationSample stale_other;
  stale_other.pid = 200;
  stale_other.time_stamp_us = 1000000;
  stale_other.sm_util = 60;

  std::vector<NvmlProcessUtilizationSample> samples{ours, stale_other};
  // Only ours is fresh -> normalizer 40 -> share 1.0.
  EXPECT_DOUBLE_EQ(
      GpuMonitor::compute_process_share(100, 11000000, samples, {}), 1.0);
}

TEST(GpuShareTest, FreshnessBoundaryExactlyWindowIsFresh) {
  NvmlProcessUtilizationSample ours;
  ours.pid = 100;
  ours.time_stamp_us = 1000000;
  ours.sm_util = 40;

  NvmlProcessUtilizationSample other;
  other.pid = 200;
  other.time_stamp_us = 1000000;
  other.sm_util = 40;

  std::vector<NvmlProcessUtilizationSample> samples{ours, other};
  // now - timestamp == FRESH_SAMPLE_WINDOW_US -> still fresh -> 0.5.
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 6000000, samples, {}),
                   0.5);
}

TEST(GpuShareTest, AbsentPidWithOthersFreshUsesMemoryFallback) {
  NvmlProcessUtilizationSample other;
  other.pid = 200;
  other.time_stamp_us = 2000000;
  other.sm_util = 90;

  NvmlProcessInfoV2 ours;
  ours.pid = 100;
  ours.used_gpu_memory = 1ULL << 30;

  NvmlProcessInfoV2 other_process;
  other_process.pid = 200;
  other_process.used_gpu_memory = 3ULL << 30;

  std::vector<NvmlProcessUtilizationSample> samples{other};
  std::vector<NvmlProcessInfoV2> running{ours, other_process};
  EXPECT_DOUBLE_EQ(
      GpuMonitor::compute_process_share(100, 3000000, samples, running), 0.25);
}

TEST(GpuShareTest, ZeroMemoryForOurPidReturnsZero) {
  NvmlProcessInfoV2 ours;
  ours.pid = 100;
  ours.used_gpu_memory = 0;

  NvmlProcessInfoV2 other;
  other.pid = 200;
  other.used_gpu_memory = 1ULL << 30;

  std::vector<NvmlProcessInfoV2> running{ours, other};
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 2000000, {}, running),
                   0.0);
}

TEST(GpuShareTest, AbsentPidWithoutSamplesReturnsZero) {
  NvmlProcessInfoV2 other;
  other.pid = 200;
  other.used_gpu_memory = 1ULL << 30;

  std::vector<NvmlProcessInfoV2> running{other};
  EXPECT_DOUBLE_EQ(GpuMonitor::compute_process_share(100, 2000000, {}, running),
                   0.0);
}

TEST(GpuIntegrationTest, TrapezoidOverGap) {
  // 0.5 * (10 W * 0.5 + 20 W * 1.0) * 1 s = 12.5 J = 12.5e6 uJ.
  EXPECT_DOUBLE_EQ(GpuMonitor::integrate_gap(10.0, 0.5, 20.0, 1.0, 1e6),
                   12.5e6);
}

TEST(GpuIntegrationTest, ZeroOrNegativeGapIsZero) {
  EXPECT_DOUBLE_EQ(GpuMonitor::integrate_gap(10.0, 1.0, 20.0, 1.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(GpuMonitor::integrate_gap(10.0, 1.0, 20.0, 1.0, -5.0), 0.0);
}

TEST(GpuPowerTest, PrefersValidPowerReading) {
  EXPECT_DOUBLE_EQ(GpuMonitor::select_power_w(true, 42.0, 180.0, 50.0), 42.0);
}

TEST(GpuPowerTest, FallsBackToPowerLimitTimesDeviceUtilization) {
  EXPECT_DOUBLE_EQ(GpuMonitor::select_power_w(false, 0.0, 180.0, 50.0), 90.0);
  EXPECT_DOUBLE_EQ(GpuMonitor::select_power_w(true, 0.0, 180.0, 50.0), 90.0);
}

TEST(GpuPowerTest, NoDataReturnsZero) {
  EXPECT_DOUBLE_EQ(GpuMonitor::select_power_w(false, 0.0, 0.0, 50.0), 0.0);
  EXPECT_DOUBLE_EQ(GpuMonitor::select_power_w(false, 0.0, 180.0, 0.0), 0.0);
}

TEST(GpuMonitorTest, EnergyAccumulatorIsMonotonic) {
  GpuMonitor monitor;
  double first_energy = monitor.read_process_energy_uj();
  double first_utilization = monitor.read_process_utilization_us();
  double second_energy = monitor.read_process_energy_uj();
  double second_utilization = monitor.read_process_utilization_us();

  // No accrual happens on the first call (state initialization only).
  EXPECT_DOUBLE_EQ(first_energy, 0.0);
  EXPECT_DOUBLE_EQ(first_utilization, 0.0);

  EXPECT_GE(first_energy, 0.0);
  EXPECT_GE(second_energy, first_energy);
  EXPECT_GE(first_utilization, 0.0);
  EXPECT_GE(second_utilization, first_utilization);
}
