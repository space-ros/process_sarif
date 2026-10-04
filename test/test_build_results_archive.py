# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Tests for the error-reporting paths of build_results_archive."""

import os

import pytest

from process_sarif.build_results_archive import get_build_cmd
from process_sarif.build_results_archive import get_test_cmd


def test_get_build_cmd_missing_log(tmp_path, monkeypatch):
    """get_build_cmd reports a missing build log as RuntimeError, not TypeError."""
    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError, match='No colcon build log available'):
        get_build_cmd()


def test_get_test_cmd_missing_log(tmp_path, monkeypatch):
    """get_test_cmd reports a missing test log as RuntimeError, not TypeError."""
    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError, match='No colcon test log available'):
        get_test_cmd()


def test_get_build_cmd_log_without_command_line(tmp_path, monkeypatch):
    """A build log without a command-line record raises RuntimeError."""
    monkeypatch.chdir(tmp_path)
    log_dir = os.path.join('log', 'latest_build')
    os.makedirs(log_dir)
    with open(os.path.join(log_dir, 'logger_all.log'), 'w') as log:
        log.write('nothing relevant here\n')
    with pytest.raises(RuntimeError, match='No command line arguments found'):
        get_build_cmd()


def test_get_test_cmd_log_without_command_line(tmp_path, monkeypatch):
    """A test log without a command-line record raises RuntimeError."""
    monkeypatch.chdir(tmp_path)
    log_dir = os.path.join('log', 'latest_test')
    os.makedirs(log_dir)
    with open(os.path.join(log_dir, 'logger_all.log'), 'w') as log:
        log.write('nothing relevant here\n')
    with pytest.raises(RuntimeError, match='No command line arguments found'):
        get_test_cmd()
