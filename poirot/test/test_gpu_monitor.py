# Copyright 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes
import os
import time

import pytest

from poirot.utils.gpu_monitor import GpuMonitor
from poirot.utils.nvml import (
    NvmlLoader,
    NvmlMemory,
    NvmlProcessInfoV2,
    NvmlProcessUtilizationSample,
    NvmlUtilization,
)


def test_uninitialized_loader_returns_no_data():
    loader = NvmlLoader()
    assert loader.available() is False
    assert loader.name() == ""
    assert loader.read_power_w() is None
    assert loader.read_power_limit_w() is None
    assert loader.read_device_utilization() is None
    assert loader.read_memory() is None
    assert loader.read_running_processes() is None
    assert loader.read_process_utilization() is None


def test_initialize_reports_consistently():
    loader = NvmlLoader()
    initialized = loader.initialize()
    assert loader.available() is initialized
    if initialized:
        assert loader.name() != ""
        power_w = loader.read_power_w()
        assert power_w is None or power_w >= 0.0
        memory = loader.read_memory()
        if memory is not None:
            assert memory.total > 0
        limit_w = loader.read_power_limit_w()
        if limit_w is not None:
            assert limit_w >= 0.0
        utilization = loader.read_device_utilization()
        if utilization is not None:
            assert utilization >= 0.0
        loader.read_running_processes()
        loader.read_process_utilization()
    else:
        assert loader.name() == ""


def test_struct_layouts():
    assert ctypes.sizeof(NvmlUtilization) == 8
    assert ctypes.sizeof(NvmlMemory) == 24
    assert ctypes.sizeof(NvmlProcessUtilizationSample) == 32
    assert ctypes.sizeof(NvmlProcessInfoV2) == 24


def _sample(pid, timestamp_us, sm_util):
    sample = NvmlProcessUtilizationSample()
    sample.pid = pid
    sample.time_stamp_us = timestamp_us
    sample.sm_util = sm_util
    return sample


def _process(pid, used_gpu_memory):
    process = NvmlProcessInfoV2()
    process.pid = pid
    process.used_gpu_memory = used_gpu_memory
    return process


def test_share_normalizes_fresh_samples():
    samples = [_sample(100, 1_000_000, 50), _sample(200, 1_000_000, 50)]
    assert GpuMonitor.compute_process_share(100, 2_000_000, samples, []) == 0.5


def test_share_fresh_zero_sample_returns_zero():
    samples = [_sample(100, 1_000_000, 0)]
    assert GpuMonitor.compute_process_share(100, 2_000_000, samples, []) == 0.0


def test_share_stale_sample_uses_memory_fallback():
    samples = [_sample(100, 1_000_000, 90)]
    running = [_process(100, 1 << 30), _process(200, 3 << 30)]
    assert GpuMonitor.compute_process_share(100, 11_000_000, samples, running) == 0.25


def test_share_future_timestamp_ignored():
    samples = [_sample(100, 5_000_000, 90)]
    assert GpuMonitor.compute_process_share(100, 1_000_000, samples, []) == 0.0


def test_share_no_data_returns_zero():
    assert GpuMonitor.compute_process_share(100, 2_000_000, [], []) == 0.0


def test_share_multiple_samples_per_pid_use_newest():
    samples = [
        _sample(100, 1_000_000, 10),
        _sample(100, 2_000_000, 50),
        _sample(200, 2_000_000, 50),
    ]
    # Newest ours (50) over (50 + 50); a naive sum-all would give a different value.
    assert GpuMonitor.compute_process_share(100, 3_000_000, samples, []) == 0.5


def test_share_stale_samples_excluded_from_normalizer():
    samples = [_sample(100, 10_000_000, 40), _sample(200, 1_000_000, 60)]
    # Only ours is fresh -> normalizer 40 -> share 1.0.
    assert GpuMonitor.compute_process_share(100, 11_000_000, samples, []) == 1.0


def test_share_freshness_boundary_exactly_window_is_fresh():
    samples = [_sample(100, 1_000_000, 40), _sample(200, 1_000_000, 40)]
    assert GpuMonitor.compute_process_share(100, 6_000_000, samples, []) == 0.5


def test_integrate_gap_trapezoid():
    assert GpuMonitor.integrate_gap(10.0, 0.5, 20.0, 1.0, 1e6) == 12.5e6


def test_integrate_gap_zero_or_negative():
    assert GpuMonitor.integrate_gap(10.0, 1.0, 20.0, 1.0, 0.0) == 0.0
    assert GpuMonitor.integrate_gap(10.0, 1.0, 20.0, 1.0, -5.0) == 0.0


def test_select_power_prefers_reading():
    assert GpuMonitor.select_power_w(True, 42.0, 180.0, 50.0) == 42.0


def test_select_power_fallback():
    assert GpuMonitor.select_power_w(False, 0.0, 180.0, 50.0) == 90.0
    assert GpuMonitor.select_power_w(True, 0.0, 180.0, 50.0) == 90.0


def test_select_power_no_data():
    assert GpuMonitor.select_power_w(False, 0.0, 0.0, 50.0) == 0.0
    assert GpuMonitor.select_power_w(False, 0.0, 180.0, 0.0) == 0.0


def test_monitor_energy_accumulator_monotonic():
    monitor = GpuMonitor()
    first_energy = monitor.read_process_energy_uj()
    first_utilization = monitor.read_process_utilization_us()
    assert first_energy == 0.0
    assert first_utilization == 0.0
    second_energy = monitor.read_process_energy_uj()
    second_utilization = monitor.read_process_utilization_us()
    assert first_energy >= 0.0
    assert second_energy >= first_energy
    assert first_utilization >= 0.0
    assert second_utilization >= first_utilization


class _StubNvml:
    def __init__(self, power_w, sm_util):
        self._power_w = power_w
        self._sm_util = sm_util

    def read_power_w(self):
        return self._power_w

    def read_device_utilization(self):
        return 0.0

    def read_process_utilization(self):
        sample = NvmlProcessUtilizationSample()
        sample.pid = os.getpid()
        sample.time_stamp_us = time.time_ns() // 1_000 - 1_000
        sample.sm_util = self._sm_util
        return [sample]

    def read_running_processes(self):
        return []


def test_energy_accrual_with_stubbed_nvml():
    monitor = GpuMonitor()
    monitor._use_nvml = True
    monitor._nvml = _StubNvml(power_w=100.0, sm_util=50)

    # First call only seeds the integration state.
    assert monitor.read_process_energy_uj() == 0.0

    start_us = monitor._last_process_read_us
    time.sleep(0.05)
    monitor.read_process_energy_uj()
    dt_us = monitor._last_process_read_us - start_us

    # Only our PID has a fresh sample -> share = 1.0; equal trapezoid endpoints.
    assert pytest.approx(monitor._accumulated_process_energy_uj) == 100.0 * dt_us
    assert pytest.approx(monitor._accumulated_share_us) == dt_us
