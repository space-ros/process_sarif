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

    build_dirs = os.listdir("build")

    for package in build_dirs:
        # If a whitelist was provided and this package isn't in it, then discard.
        if len(whitelist) > 0 and package not in whitelist: continue
        
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

def replace_misra_rules(files: List[SarifFile], rules_txt_path: str, verbose=False) -> List[SarifFile]:
    """
    Load MISRA rules from share/config/misra_rules.txt, and applies them to any Result in a
    SarifFile referencing a MISRA rule.

    The rules text should be in this format:

    rule_id_1\tRule ID 1 description
    rule_id_2\tRule ID 2 description
    ...
    rule_id_n\tRule ID n description
    """

    rules = {}

    if not os.path.exists(rules_txt_path):
        if verbose: print("Rules text path does not exist! Not replacing rules.")

        return files

    # Load into rules dict from misra_rules
    with open(rules_txt_path) as f:
        for line in f.readlines():
            rule_id, text = line.split("\t")
            rules[rule_id] = text

    total_replaced = 0

    # Check and replace all matching results
    for f in files:
        results = f.results
        for result in results:
            if result.ruleId in rules:
                print(f"Replacing {result.ruleId}")
                result.message = rules[result.ruleId]

                total_replaced += 1

        # Explicitly set the SarifFile.results so that JSON is updated
        f.results = results

    if verbose: print(f"Replaced {total_replaced} MISRA rules total.")

    return files

def remove_duplicate_results(files: List[SarifFile], verbose=False) -> List[SarifFile]:
    '''
    Takes a list of results, and removes duplicate results across them. The returned
      SarifFiles will only have one Result out of the set of duplicates included.
    A Result is the same as another if they share an identical Artifact, Region, and Result.ruleId.
    The first SarifFile in the returned List[SarifFile] for which a duplicate was found will retain the Result.

    TODO: If a removed result is the last of that ruleId in the SarifFile, remove that Rule from the Tool.
    TODO: If a remove result is the last referring to an Artifact in the SarifFile, remove that Artifact from the SarifFile.
    TODO: Once the above 2 are implemented, rework test_sarif:test_duplicate_result() to conform.
    '''

    # Dictionary of {rule_id_1: {artifact_1: {region_1, region_2, ...}, ...}, ...}
    # This is for speedy checking of duplicate results.
    results_lookup = {}

    removed_res_count = 0
    
    for f in files:
        results = f.results
        for res in results[:]:
            # If we haven't encountered this rule, it can't be a duplicate.
            if res.ruleId not in results_lookup:
                if verbose: print(f"Rule {res.ruleId} does not exist, adding entry.")

                results_lookup[res.ruleId] = {}
                results_lookup[res.ruleId][res.artifact] = {}
                results_lookup[res.ruleId][res.artifact] = {res.region}
                continue

            # Note the short circuit: if the first term is False, the second doesn't evaluate.
            # If both are true, we have a duplicate! This lookup is hopefully ~O(1).
            if res.artifact in results_lookup[res.ruleId] and res.region in results_lookup[res.ruleId][res.artifact]:
                if verbose: print(f"Result duplicate found, removing res idx {results.index(res)} from sarif file idx {files.index(f)}")
                
                results.remove(res)
                removed_res_count += 1
            # Otherwise, we need to update the results_lookup dict with data that missed on lookup.
            elif not res.artifact in results_lookup[res.ruleId]:
                results_lookup[res.ruleId][res.artifact] = {res.region}
            elif not res.region in results_lookup[res.ruleId][res.artifact]:
                results_lookup[res.ruleId][res.artifact].add(res.region)

        # Trigger json update by updating the results field.
        f.results = results

    if verbose: print(f"Total number of duplicates removed: {removed_res_count}")

    return files

if __name__ == "__main__":
    print("This is a library! Don't run me :(")