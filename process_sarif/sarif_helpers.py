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

def find_duplicate_results(results: List[Result], verbose=False) -> List[Result]:
    '''
    Takes a list of results, and finds duplicate results across them. The returned
      Results are the unique Results after removing duplicates.
    A Result is the same as another if they share an identical Artifact and Region.
      This is a poor assumption, but a reasonable first pass.
    '''

    unique_results = []
    duplicate_groups = []
    
    for res in results:
        matched_group = False

        for group in duplicate_groups:
            # If the artifact and region match, its likely this is the same result
            # We don't need to search every element in group, since they are guaranteed to
            #   all match artifact and region, based on how the list is constructed.
            if res.artifact == group[0].artifact and res.region == group[0].region:
                group.append(res)
                matched_group = True
                break

        # If this result didn't match a group, create a new one and add this to unique results.
        if not matched_group:
            duplicate_groups.append([res])
            unique_results.append(res)

    total_duplicates = 0

    for group in duplicate_groups:
        if len(group) > 1:
            total_duplicates += len(group)

    if verbose: print(f"Total number of duplicates found: {total_duplicates}")

if __name__ == "__main__":
    print("This is a library! Don't run me :(")