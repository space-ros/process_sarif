#!/usr/bin/env python3

from process_sarif.sarif_helpers import get_sarif_in_build, remove_duplicate_results


def main():
    sarif_files = get_sarif_in_build(verbose=False)

    unique = remove_duplicate_results(sarif_files, verbose=False)

if __name__ == "__main__":
    main()