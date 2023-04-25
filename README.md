# SARIF Processing

## 0. What is this?
This repository is for tools and scripts to assist in parsing and aggregating SARIF output. Currently, these tools are built in python, but could be (relatively) easily adapted to C++.

## 1. Tools
All of the following tools are run with `ros2 run process_sarif <tool_name>` after building `process_sarif` and sourcing spaceros_ws/install/setup.bash.

- sarif_load_test
    - This tool demonstrates library capabilities in sarif.py, and helper functions from sarif_helpers.py. It should be run from the root of the workspace, otherwise it may be unable to find SARIF from test output. It will load all SARIF found in build/ into SarifFile objects, then print out all of the objects (pausing for user input). Optionally install rich (`python3 -m pip install rich`) in the docker container to get prettier output.
- conformance
    - This tool outputs a `spaceros_ws/conformance_log.txt` file, which includes logs from sarif.py if it struggled to parse the given SARIF path into a SarifFile object. It is useful for identifying tools that are not currently conforming to the SARIF spec.
- duplicates
    - This tool finds SARIF Results across tools that 1) share the same Artifact and 2) share the same Region, and 3) match the same ruleId. It will help us identify if two tools report the same Result, or if two different rules across tools actually refer to the same kind of violation.
- visualize
    - This tool generates visualizations. Currently, it creates a `spaceros_ws/level_count.png` histogram that displays how many Results are present at each warning level (Error, Warning, Unknown).

## 2. Files
These are files that aren't run directly, but can be imported by your own scripts to use SARIF processing tools present in this package.
- sarif.py
    - This library has dataclass models for different interesting components from the (SARIF spec)[https://docs.oasis-open.org/sarif/sarif/v2.1.0/csprd01/sarif-v2.1.0-csprd01.html]. It does NOT implement the whole spec (nowhere near). The main entry point is SarifFile.fromPath, which will create a SarifFile object (composition of Tool, Artifacts, and Results) from the file path passed in.
- sarif_helpers.py
    - This file contains helper functions for manipulating objects defined in sarif.py. The most common one used is `get_sarif_in_build`, which will return a SarifFile for every `build/**/*.sarif`.

## 3. Desired features (not yet implemented)
- There should be functions provided to aggregate Results by:
    - ROS package, to determine which packages need the most work.
    - Static analysis tool, to determine whether some tools are too "loud" or aren't providing the results we want.
    - Source file (including location within the file), to be able to compare results pointing at the same file/line.
    - `colcon test` run, to compare results across runs of tests.
- There should be visualizations of these different aggregations.
- All testing tools should output valid SARIF.
    - Many Cobra files cannot be loaded due to invalid JSON, or non-conformance to the SARIF spec.

## 4. Snap
These tools are available to install from the [Snap Store](https://snapcraft.io/about) as a [snap](https://snapcraft.io/process-sarif) as a complementary packaging option maintained by Canonical.

### Install: 
`snap install process-sarif`
### Usage
Navigate to a directory containing the SARIF file(s) you wish to analyze. Run any of the available commands (sarif-load-test, conformance, duplicates, visualize), as `process-sarif.<app_name>` (ie: process-sarif.sarif-load-test)

