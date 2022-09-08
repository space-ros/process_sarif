#!/usr/bin/env python3

import os
from process_sarif.sarif_helpers import get_sarif_in_build, replace_misra_rules


def main():
    # Verify we're at the workspace root
    if not os.path.exists("build"):
        # Guess the root workspace
        ws_default = "/root/src/spaceros_ws"
        if os.path.exists(ws_default):
            os.chdir("/root/src/spaceros_ws")
            
    # If it still doesn't exist, fail
    if not os.path.exists("build"):
        print("Failed to find build directoy. Please run this script from your workspace root.")

    sarif_files = get_sarif_in_build(verbose=False)

    print(f"Loaded in {len(sarif_files)} SARIF files.")

    result_count = sum([len(s.results) for s in sarif_files])

    print(f"Total results: {result_count}")

    # Uncomment if you want to replace_misra_rules. Replace `rules_txt_path=None` with the path to your rules.
    # print("Replacing MISRA rule text")

    # for f in sarif_files:
    #     f.results = replace_misra_rules(files=f.results, rules_txt_path=None)

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
            input("(enter to continue)")

if __name__ == "__main__":
    main()