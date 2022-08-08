# SARIF Processing

## 0. What is this?
This repository is for tools and scripts to assist in parsing and aggregating SARIF output. Currently, these tools are built in python, but could be (relatively) easily adapted to C++.

## 1. Features
- sarif.py
    - This library has dataclass models for different interesting components from the (SARIF spec)[https://docs.oasis-open.org/sarif/sarif/v2.1.0/csprd01/sarif-v2.1.0-csprd01.html]. It does NOT implement the whole spec (nowhere near). The main entry point is SarifFile.fromPath, which will create a SarifFile object (composition of Tool, Artifacts, and Results) from the file path passed in.
- sarif_parser.py
    - This file contains helper functions for manipulating objects defined in sarif.py. Currently, it only implmenets `get_sarif_in_build`, which returns `List[SarifFile]`, one SarifFile for every `*.sarif` in `build/**`.
- sarif_load_test.py
    - This is an example script to demonstrate library capabilities from sarif.py above. It should be run from the root of the workspace (eg. `python3 /root/src/spaceros_ws/sarif_load_test.py`). It will load all SARIF found in build/ into SarifFile objects, then print out all of the objects (pausing for user input). Optionally install rich (`python3 -m pip install rich`) in the docker container to get prettier output.

## 2. Desired features (not yet implemented)
- There should be functions provided to aggregate Results by:
    - ROS package, to determine which packages need the most work.
    - Static analysis tool, to determine whether some tools are too "loud" or aren't providing the results we want.
    - Source file (including location within the file), to be able to compare results pointing at the same file/line.
    - `colcon test` run, to compare results across runs of tests.
- There should be visualizations of these different aggregations.
- There should be better error reporting for when SARIF does not match the expecte format (Cobra is a current repeat offender), so that we can create issues and PRs to align our SARIF results.