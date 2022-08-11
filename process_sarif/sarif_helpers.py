#!/usr/bin/env python3

import os

from typing import List, Tuple, Optional
from ament_index_python.packages import get_package_share_directory
from process_sarif.sarif import SarifFile, Result


# This library wraps around the objects in sarif.py, and provides helper functions
# for loading, processing, and manipulating them. Each function here has documentaiton
# as to its purpose and capabilities.

def get_sarif_in_build(whitelist=[], verbose=True, log_path: Optional[str] = None) -> List[SarifFile]:
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
                sarif_file = SarifFile.from_path(sarif_path, verbose=verbose, log_path=log_path)
                if sarif_file is not None:
                    sarif_files.append(sarif_file)

    return sarif_files

def replace_misra_results(results: List[Result], verbose=False) -> List[Result]:
    '''
    Load MISRA rules from share/config/misra_rules.txt, and applies them to any Result referencing a MISRA rule.
    '''

    rules = {}

    share_dir = get_package_share_directory("process_sarif")

    # Load into rules dict from misra_rules
    with open(os.path.join(share_dir, "config/misra_rules.txt")) as f:
        for line in f.readlines():
            rule_id, text = line.split("\t")
            rules[rule_id] = text

    total_replaced = 0

    # Check and replace all matching results
    for result in results:
        if result.ruleId in rules:
            result.message = rules[result.ruleId]

            total_replaced += 1

    if verbose: print(f"Replaced {total_replaced} MISRA rules total.")

    return results

def find_duplicate_results(results: List[Result]) -> Tuple[List[List[Result]], List[Result]]:
    '''
    Takes a list of results, and finds duplicate results across them. The return type is complicated, but is just a tuple of two items:
        1. List[List[Result]]. This is a list of lists, where each inner list is Results that are duplicated of each other.
        2. List[Result]. The Results that are remaining after duplicates are removed.

    A Result is the same as another if they share the same Artifact and Region.
    '''

    unique_results = []
    duplicate_groups = []
    
    for r1 in results:
        for r2 in results:
            # Skip the ones that are the same, since we're iterating through a 1d list.
            if r1 is r2: continue

            if r1.artifact == r2.artifact:
                in_duplicates = False

                for group in duplicate_groups:
                    if r1 in group:
                        group.append(r1)
                        in_duplicates = True
                        break
                    elif r2 in group:
                        group.append(r2)
                        in_duplicates = True
                        break

                if not in_duplicates:
                    duplicate_groups.append([r1, r2])

                # Only add r1 to 


if __name__ == "__main__":
    print("This is a library! Don't run me :(")