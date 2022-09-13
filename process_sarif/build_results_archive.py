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
import subprocess

from .sarif import SarifFile
from .sarif_helpers import remove_duplicate_results
from typing import List


def main():
    # TODO(Steven!) make this path independent.
    if not os.path.isdir('log/build_results_archives'):
        os.makedirs('log/build_results_archives')
    with tarfile.open('log/build_results_archives/current.tar', 'w') as archive:
        sarif_files = [
                SarifFile.from_path(sarif_path, verbose=True)
                for sarif_path in
                sorted(pathlib.Path('build').glob('**/*.sarif'))]

        remove_duplicate_results(sarif_files)

        with open('build-results-archive', 'w') as config:
            config.write(
                    '''
                    { "version": 1 }
                    ''')

        archive.add('build-results-archive')

        # TODO(Steven!) Add colcon-build-cmd, colcon-test-cmd, and process-sarif-cmd.
        
        # TODO(Steven!) Add vcs-export-exact.repos
        # Assume we are at the workspace root, since log/build_results_archives is relative to ws_root.
        repos = subprocess.call(["vcs", "export", "--exact", "src"])

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




def processed_path(sarif_path: str):
    return sarif_path.replace('build/', 'processed/')


#def make_build_results_archive(sarif_files: List[SarifFile], processors):
#    '''
#    Create an archive of build results from the latest colcon bu
#    '''
