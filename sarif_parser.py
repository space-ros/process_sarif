#!/usr/bin/env python3

import os

from typing import List
from sarif import SarifFile


def get_sarif_in_build(whitelist=[], verbose=True) -> List[SarifFile]:
    '''
    Returns a list of SarifFiles, loaded from the build directory.
    If len(whitelist) > 0, then only sarif files in packages within the whitelist are returned.
    '''

    sarif_files = []

    build_dirs = os.listdir("build")

    for package in build_dirs:
        if len(whitelist) > 0 and package not in whitelist: continue
        
        results_path = os.path.join(os.getcwd(), "build", package, "test_results", package)

        if not os.path.exists(results_path): continue

        for f in os.listdir(results_path):
            sarif_path = os.path.join(results_path, f)
            if f.endswith(".sarif") and os.stat(sarif_path).st_size != 0:
                sarif_file = SarifFile.fromPath(sarif_path, verbose=verbose)
                if sarif_file is not None:
                    sarif_files.append(sarif_file)

    return sarif_files

if __name__ == "__main__":
    print("This is a library! Don't run me :(")