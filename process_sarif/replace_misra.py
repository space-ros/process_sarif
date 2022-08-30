#!/usr/bin/env python3

from process_sarif.sarif_helpers import get_sarif_in_build, replace_misra_rules


def main():
    sarif_files = get_sarif_in_build(verbose=False)
    
    print(f"SARIF files loaded: {len(sarif_files)}")
    input()

    misra_replaced = replace_misra_rules(sarif_files)

    for f in misra_replaced:
        f.write_json()

if __name__ == "__main__":
    main()