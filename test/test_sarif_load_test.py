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

"""Tests for the sarif_load_test entry point."""

import pytest

import process_sarif.sarif_load_test as sarif_load_test


def test_main_exits_when_no_build_dir(monkeypatch):
    """Regression test for #17.

    When no ``build/`` directory is found (and no fallback workspace exists),
    main() must terminate cleanly with a non-zero exit status rather than
    continuing into get_sarif_in_build() and raising FileNotFoundError.
    """
    monkeypatch.setattr(sarif_load_test.os.path, "exists", lambda _path: False)
    with pytest.raises(SystemExit) as excinfo:
        sarif_load_test.main()
    assert excinfo.value.code == 1
