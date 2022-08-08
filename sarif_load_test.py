#!/usr/bin/env python3

import os

from sarif import SarifFile, Artifact, Result, Tool


def main():
    sarif_files = []

    build_dirs = os.listdir("build")

    for package in build_dirs:
        results_path = os.path.join(os.getcwd(), "build", package, "test_results", package)

        if not os.path.exists(results_path): continue

        for f in os.listdir(results_path):
            sarif_path = os.path.join(results_path, f)
            if f.endswith(".sarif") and os.stat(sarif_path).st_size != 0:
                sarif_file = SarifFile.fromPath(os.path.join(results_path, f), verbose=False)
                if sarif_file is not None:
                    sarif_files.append(sarif_file)

    print(f"Loaded in {len(sarif_files)} SARIF files.")

    print(f"Results from SARIF file idx 0:")

    use_pprint = False

    try:
        from rich.pretty import pprint
        use_pprint = True
    except ImportError as e:
        pass
    
    for f in sarif_files:
        if len(f.results) > 0:
            if use_pprint:
                pprint(f)
            else:
                print(f)
            input()

if __name__ == "__main__":
    main()