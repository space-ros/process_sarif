#!/usr/bin/env python3

import os

from process_sarif.sarif_helpers import get_sarif_in_build


def main():
    log_path = "conformance_log.txt"

    if os.path.exists(log_path):
        os.remove(log_path)

    get_sarif_in_build(verbose=False, log_path=log_path)
    
    print(f"Logs written to {os.path.join(os.getcwd(), log_path)}")

if __name__ == "__main__":
    main()