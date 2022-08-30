#!/usr/bin/env python3

from process_sarif.sarif_helpers import get_sarif_in_build, find_duplicate_results, replace_misra_results


def main():
    sarif_files = get_sarif_in_build(verbose=False)

    results = []
    for f in sarif_files:
        results.extend(f.results)

    # TODO: Refactor to SarifFiles instead of Results
    # misra_results = replace_misra_results(results)
    unique_results = find_duplicate_results(results, verbose=True)

if __name__ == "__main__":
    main()