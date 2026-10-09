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

from types import SimpleNamespace
import subprocess
import rclpy
import pytest

from poirot.poirot import (
    Poirot,
    profile_function_with_children,
    rusage_cpu_delta_us,
)


def test_adds_user_and_system_time():
    before = SimpleNamespace(ru_utime=1.5, ru_stime=0.25)
    after = SimpleNamespace(ru_utime=2.1, ru_stime=0.30)
    assert rusage_cpu_delta_us(before, after) == pytest.approx(650000.0)


def test_handles_microsecond_carry():
    before = SimpleNamespace(ru_utime=1.9, ru_stime=0.0)
    after = SimpleNamespace(ru_utime=2.1, ru_stime=0.0)
    assert rusage_cpu_delta_us(before, after) == pytest.approx(200000.0)


def test_returns_zero_for_equal_usage():
    usage = SimpleNamespace(ru_utime=3.0, ru_stime=1.0)
    assert rusage_cpu_delta_us(usage, usage) == 0.0


def test_clamps_negative_delta_to_zero():
    before = SimpleNamespace(ru_utime=2.0, ru_stime=1.0)
    after = SimpleNamespace(ru_utime=1.0, ru_stime=0.5)
    assert rusage_cpu_delta_us(before, after) == 0.0


@profile_function_with_children
def decorated_burn_child():
    subprocess.run(
        ["sh", "-c", "i=0; while [ $i -lt 1000000 ]; do i=$((i+1)); done"],
        check=True,
    )


def test_folds_reaped_child_into_cpu_time():
    rclpy.init()
    try:
        profiler = Poirot.get_instance()
        profiler.start_profiling(
            "python_child_window", __file__, 0, include_children=True
        )
        subprocess.run(
            ["sh", "-c", "i=0; while [ $i -lt 1000000 ]; do i=$((i+1)); done"],
            check=True,
        )
        profiler.stop_profiling()

        stats = profiler._statistics["python_child_window"]
        assert stats.call.data.child_cpu_time_us > 0
        assert stats.call.data.cpu_time_us >= stats.call.data.child_cpu_time_us

        decorated_burn_child()
        decorated_stats = profiler._statistics["decorated_burn_child"]
        assert decorated_stats.call.data.child_cpu_time_us > 0
    finally:
        rclpy.shutdown()
