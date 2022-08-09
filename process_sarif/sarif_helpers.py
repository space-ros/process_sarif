#!/usr/bin/env python3

import os

from typing import List
from process_sarif.sarif import SarifFile


# This library wraps around the objects in sarif.py, and provides helper functions
# for loading, processing, and manipulating them. Each function here has documentaiton
# as to its purpose and capabilities.

def get_sarif_in_build(whitelist=[], verbose=True) -> List[SarifFile]:
    '''
    Returns a list of SarifFiles, loaded from the build directory.
    If len(whitelist) > 0, then only sarif files in packages within the whitelist are returned.
    '''

    sarif_files = []

    # TODO: Find a different way to get at the build directory, since this will no longer work.
    build_dirs = os.listdir("build")

    for package in build_dirs:
        # If a whitelist was provided and this package isn't in it, then discard.
        if len(whitelist) > 0 and package not in whitelist: continue
        
        # TODO: os.getcwd() isn't going to work anymore since this is in a ROS package.
        results_path = os.path.join(os.getcwd(), "build", package, "test_results", package)

        # If the results don't exist for this package, skip it.
        if not os.path.exists(results_path): continue

        # For all files in the results path, if its a SARIF file (and has content), attempt to load it.
        # A return value of None indicated a failure, so don't include it.
        for f in os.listdir(results_path):
            sarif_path = os.path.join(results_path, f)
            if f.endswith(".sarif") and os.stat(sarif_path).st_size != 0:
                sarif_file = SarifFile.from_path(sarif_path, verbose=verbose)
                if sarif_file is not None:
                    sarif_files.append(sarif_file)

    return sarif_files

def find_duplicate_results(sarif: List[SarifFile]):
    pass

if __name__ == "__main__":
    print("This is a library! Don't run me :(")