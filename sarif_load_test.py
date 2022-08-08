#!/usr/bin/env python3

from sarif_parser import get_sarif_in_build


def main():
    sarif_files = get_sarif_in_build(verbose=False)

    print(f"Loaded in {len(sarif_files)} SARIF files.")

    input(f"Results from SARIF files with results (press enter):")

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