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

import datetime
import os
import os.path
import pathlib
import tarfile
import shutil
import sys
import subprocess

from .sarif import SarifFile
from .sarif_helpers import remove_duplicate_results, get_sarif_in_build
from typing import List


def main():
    if not os.path.isdir('log/build_results_archives'):
        os.makedirs('log/build_results_archives')
    archive_name = archive_filename()
    archive_path = os.path.join('log', 'build_results_archives', archive_name)
    with tarfile.open(archive_path, 'w:bz2') as archive:
        with open('colcon-build-cmd', 'w') as build_cmd_file:
            build_cmd = get_build_cmd()
            build_cmd_file.write(build_cmd)
        archive.add('colcon-build-cmd')
        with open('colcon-test-cmd', 'w') as test_cmd_file:
            test_cmd = get_test_cmd()
            test_cmd_file.write(test_cmd)
        archive.add('colcon-test-cmd')
        with open('processed-sarif-cmd', 'w') as sarif_cmd_file:
            sarif_cmd = processed_sarif_cmd()
            sarif_cmd_file.write(sarif_cmd)
        archive.add('processed-sarif-cmd')

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
            archive.add(os.path.relpath(sarif.path), recursive=True)
            processed = processed_path(str(sarif.path))
            processed_dir = os.path.dirname(processed)
            if not os.path.isdir(processed_dir):
                os.makedirs(processed_dir)
            sarif.write_json(processed)
            archive.add(os.path.relpath(processed), recursive=True)

        # Cleanup
        os.remove('build-results-archive')
        os.remove('colcon-build-cmd')
        os.remove('colcon-test-cmd')
        os.remove('processed-sarif-cmd')
        os.remove('vcs-export-exact.repos')
        shutil.rmtree('processed')

    wsdir = os.getcwd()
    os.chdir(os.path.join('log', 'build_results_archives'))
    link_name = 'latest_build_results.tar.bz2'
    if os.path.exists(link_name):
        os.remove(link_name)
    os.symlink(archive_name, link_name)
    os.chdir(wsdir)


def processed_sarif_cmd():
    # # TODO(steven): without orig_argv this just gives the script path rather
    # # than the invocation. For now us the expected invocation. This may get
    # # easier if we integrate the sarif processing with an external tool 
    # # rather than an in-workspace one.
    # if hasattr(sys, 'orig_argv'):
    #     return ' '.join(sys.orig_argv)
    # else:
    #     return ' '.join(sys.argv)
    return 'ros2 run process_sarif make_build_archive'


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


def archive_filename():
    ts = datetime.datetime.utcnow()
    return ts.strftime('build_results_%Y-%m-%dT%H%M%SZ.tar.bz2')


def processed_path(sarif_path: str):
    return sarif_path.replace('build/', 'processed/')

