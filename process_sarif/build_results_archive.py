# Copyright 2022 Open Source Robotics Foundation
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

import os
import pathlib
import tarfile
import shutil
import subprocess

from .sarif import SarifFile
from .sarif_helpers import remove_duplicate_results, get_sarif_in_build
from typing import List


def main():
    # TODO(Steven!) make this path independent.
    if not os.path.isdir('log/build_results_archives'):
        os.makedirs('log/build_results_archives')
    with tarfile.open('log/build_results_archives/current.tar', 'w') as archive:
        with open('colcon-build-cmd', 'w') as build_cmd_file:
            build_cmd = get_build_cmd()
            build_cmd_file.write(build_cmd)
        archive.add('colcon-build-cmd')
        with open('colcon-test-cmd', 'w') as test_cmd_file:
            test_cmd = get_test_cmd()
            test_cmd_file.write(test_cmd)
        archive.add('colcon-test-cmd')

        sarif_files = get_sarif_in_build(verbose=False)

        remove_duplicate_results(sarif_files)

        with open('build-results-archive', 'w') as config:
            config.write(
                    '''
                    { "version": 1 }
                    ''')

        archive.add('build-results-archive')

        # Assume we are at the workspace root, since log/build_results_archives is relative to ws_root.
        repos = subprocess.run(["vcs", "export", "--exact", "src"], capture_output=True).stdout.decode()

        with open('vcs-export-exact.repos', 'w') as repos_file:
            repos_file.write(repos)

        archive.add('vcs-export-exact.repos')


        for sarif in sarif_files:
            archive.add(sarif.path, recursive=True)
            processed = processed_path(str(sarif.path))
            processed_dir = os.path.dirname(processed)
            if not os.path.isdir(processed_dir):
                os.makedirs(processed_dir)
            sarif.write_json(processed)
            archive.add(processed, recursive=True)

        # Cleanup
        os.remove('build-results-archive')
        os.remove('colcon-build-cmd')
        os.remove('colcon-test-cmd')
        os.remove('vcs-export-exact.repos')
        shutil.rmtree('processed')


def extract_cmd(logline):
    argliststr = logline.strip().split(':colcon:Command line arguments: ')[1]
    assert argliststr.startswith('[') and argliststr.endswith(']')
    # this list will retain the quote characters *within* the strings.
    arglist = argliststr[1:-1].split(', ')
    # replace $0 (which will normally be an absolute path such as
    # `/usr/bin/colcon` or `/venv/path/bin/colcon` with just `colcon`
    # so it will work in a different context.
    arglist[0] = "'colcon'"
    return ' '.join(arglist)


def get_build_cmd():
    build_log_path = 'log/latest_build/logger_all.log'
    if not os.path.exists(build_log_path):
        raise 'No colcon build log available'
    with open(build_log_path, 'r') as log:
        for line in log:
            if 'colcon:Command line arguments:' in line:
                return extract_cmd(line)
        raise 'No command line arguments found in log file'


def get_test_cmd():
    test_log_path = 'log/latest_test/logger_all.log'
    if not os.path.exists(test_log_path):
        raise 'No colcon test log available'
    with open(test_log_path, 'r') as log:
        for line in log:
            if 'colcon:Command line arguments:' in line:
                return extract_cmd(line)
        raise 'No command line arguments found in log file'


def processed_path(sarif_path: str):
    return sarif_path.replace('build/', 'processed/')

