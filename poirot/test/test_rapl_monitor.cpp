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

#include "poirot/utils/rapl_monitor.hpp"

using poirot::utils::RaplMonitor;

TEST(RaplMonitorTest, AttributeEnergyIntervalPath) {
  // 1000 uJ measured over 100 ms of system busy time; thread used 50 ms.
  double energy = RaplMonitor::attribute_energy(1000.0, 100000.0, 50000.0, 0.0);
  EXPECT_DOUBLE_EQ(energy, 500.0);
}

TEST(RaplMonitorTest, AttributeEnergyClampsShareToOne) {
  // Thread CPU time exceeds system busy time -> share clamped to 1.
  double energy =
      RaplMonitor::attribute_energy(1000.0, 100000.0, 200000.0, 0.0);
  EXPECT_DOUBLE_EQ(energy, 1000.0);
}

TEST(RaplMonitorTest, AttributeEnergyUsesRateForShortIntervals) {
  // Busy delta below MIN_INTERVAL_US (20 ms) -> fallback rate path.
  double energy = RaplMonitor::attribute_energy(1000.0, 5000.0, 10000.0, 0.05);
  EXPECT_DOUBLE_EQ(energy, 500.0);
}

TEST(RaplMonitorTest, AttributeEnergyUsesRateWhenNoEnergyDelta) {
  double energy = RaplMonitor::attribute_energy(0.0, 100000.0, 10000.0, 0.2);
  EXPECT_DOUBLE_EQ(energy, 2000.0);
}

TEST(RaplMonitorTest, AttributeEnergyReturnsZeroWithoutData) {
  EXPECT_DOUBLE_EQ(RaplMonitor::attribute_energy(0.0, 100000.0, 10000.0, 0.0),
                   0.0);
  EXPECT_DOUBLE_EQ(RaplMonitor::attribute_energy(1000.0, 100000.0, 0.0, 0.5),
                   0.0);
}

TEST(RaplMonitorTest, AttributeEnergyNeverReturnsNegative) {
  // Negative inputs must never produce negative attributed energy.
  EXPECT_DOUBLE_EQ(
      RaplMonitor::attribute_energy(-1000.0, 100000.0, 10000.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(RaplMonitor::attribute_energy(0.0, 100000.0, 10000.0, -0.5),
                   0.0);
}

TEST(RateWindowTest, InitializesWithoutRate) {
  poirot::utils::RateWindow window;
  window.update(1000, 100.0, 200.0, 250000);
  EXPECT_DOUBLE_EQ(window.rate_uj_per_busy_us, 0.0);
  EXPECT_EQ(window.start_time_us, 1000);
}

TEST(RateWindowTest, UpdatesAfterWindowElapsed) {
  poirot::utils::RateWindow window;
  window.update(1000, 100.0, 200.0, 250000);
  // Window not yet elapsed: no update.
  window.update(200000, 140.0, 280.0, 250000);
  EXPECT_DOUBLE_EQ(window.rate_uj_per_busy_us, 0.0);
  // Window elapsed: (150 - 100) / (300 - 200) = 0.5 uJ per busy us.
  window.update(251000, 150.0, 300.0, 250000);
  EXPECT_DOUBLE_EQ(window.rate_uj_per_busy_us, 0.5);
}

TEST(RateWindowTest, DiscardsIdleWindowAndResets) {
  poirot::utils::RateWindow window;
  window.update(1000, 100.0, 200.0, 250000);
  window.update(251000, 150.0, 300.0, 250000); // rate = 0.5
  // No deltas: discard sample, keep previous rate, reset window.
  window.update(502000, 150.0, 300.0, 250000);
  EXPECT_DOUBLE_EQ(window.rate_uj_per_busy_us, 0.5);
  // New valid window: (250 - 150) / (400 - 300) = 1.0.
  window.update(753000, 250.0, 400.0, 250000);
  EXPECT_DOUBLE_EQ(window.rate_uj_per_busy_us, 1.0);
}

TEST(RaplMonitorTest, ParseProcStatBusyTime) {
  // user nice system idle iowait irq softirq steal guest guest_nice
  std::string line = "cpu  100 0 50 200 10 0 5 0 7 3";
  // busy = 100 + 0 + 50 + 0 + 5 + 0 = 155 jiffies; at 100 Hz -> 1,550,000 us.
  // Guest time (7, 3) is already included in user/nice and must be ignored.
  EXPECT_DOUBLE_EQ(RaplMonitor::parse_proc_stat_busy_us(line, 100), 1550000.0);
  // clk_tck <= 0 falls back to 100.
  EXPECT_DOUBLE_EQ(RaplMonitor::parse_proc_stat_busy_us(line, 0), 1550000.0);
}

TEST(RaplMonitorTest, ParseProcStatRejectsMalformedLines) {
  EXPECT_DOUBLE_EQ(RaplMonitor::parse_proc_stat_busy_us("", 100), 0.0);
  EXPECT_DOUBLE_EQ(RaplMonitor::parse_proc_stat_busy_us("cpu0 1 2 3 4", 100),
                   0.0);
  EXPECT_DOUBLE_EQ(RaplMonitor::parse_proc_stat_busy_us("cpu  1 2 3", 100),
                   0.0);
}

TEST(RaplMonitorTest, CalculateCallEnergyReturnsZeroForInvalidSnapshots) {
  RaplMonitor monitor;
  poirot::utils::EnergySnapshot start;
  poirot::utils::EnergySnapshot end;
  EXPECT_DOUBLE_EQ(monitor.calculate_call_energy_uj(start, end, 1000.0), 0.0);
}

TEST(RaplMonitorTest, CalculateCallEnergyUsesIntervalPath) {
  // Valid snapshots: 1000 uJ over 100 ms busy; thread used 50 ms -> 500 uJ.
  RaplMonitor monitor;
  poirot::utils::EnergySnapshot start{0.0, 0.0, true};
  poirot::utils::EnergySnapshot end{1000.0, 100000.0, true};
  EXPECT_DOUBLE_EQ(monitor.calculate_call_energy_uj(start, end, 50000.0),
                   500.0);
}
