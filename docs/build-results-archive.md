# Build results archive.

This document describes a build results archive format which can be used to store the sarif results of a build-and-test procedure.

The archive will take the form of a compressed (zstd if available, or bzip2 otherwise) tar file containing the following files.
Once created this archive and its contents _should not_ be modified by any downstream program so that the integrity of the original build and its results can be maintained.

Applications which integrate the data and enhance or elaborate it are _recommended_ to maintain their own data storage for these enhancements.

```
./build-results-archive
./build/**/*.sarif
./colcon-build-cmd
./colcon-test-cmd
./processed.sarif
./vcs-export-exact.repos
```

The individual files are further described:

## build-results-archive

This is a JSON file containing metadata regarding the current archive.
The file _must_ contain an single top-level JSON object and that object _must_ contain the `"version"` key with the value of `1`.
Future revisions of this format may add additional valid versions.

## build/**/*.sarif

The build/ subdirectory _must_ contain the unaltered sarif content of the workspace build directory after the end of the latest colcon test run.

## colcon-build-cmd

This file _must_ contain the full colcon build command described in the colcon logs under the `latest_build` symlink.
The file, when run as a /bin/sh program or prefixed with the `exec` shell command in a workspace root _should_ invoke colcon in the local environment using the same arguments as the original build.

## colcon-test-cmd

This file is identical in behavior to `colcon-build-cmd` but _must_ describe the test command under the `latest_test` symlink.

## processed-sarif-cmd

This file is optional but recommended.
If it is present, the processed.sarif file _must_ also be present.
If present it should be identical to the colcon-build-cmd file in format but _must_ describe instead the command used to generate the processed sarif files.

## processed.sarif

This file is optional but recommended.
If it is present, the process-sarif-cmd file _must_ also be present.
If present, it _should_ contain the aggregated, consolidated sarif results from the entire build/ directory.

## vcs-export-exact.repos

This file _must_ contain the output of the command `vcs export --exact src` as run from the workspace root.
The results of this file will then be suitable to reproduce the workspace sources using `vcs import src` in a clean workspace.
