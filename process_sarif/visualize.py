#!/usr/bin/env python3

from matplotlib import pyplot as plt

from process_sarif.sarif import ResultKind, Level
from process_sarif.sarif_helpers import get_sarif_in_build


def main():
    sarif_files = get_sarif_in_build(verbose=False)

    results = []
    for f in sarif_files:
        results.extend(f.results)

    result_kinds =  {val.name: 0 for val in ResultKind}
    result_levels = {val.name: 0 for val in Level}

    for res in results:
        result_kinds[res.kind.name] += 1
        result_levels[res.level.name] += 1

    print(f"Result Kinds: {result_kinds}")
    print(f"Result Levels: {result_levels}")

    level_names = list(result_levels.keys())
    level_counts = list(result_levels.values())

    level_counts, level_names = (list(t) for t in zip(*sorted(zip(level_counts, level_names), reverse=True)))

    plt.bar(level_names, level_counts)

    plt.title("Results by Level")
    plt.ylabel("Result count")
    plt.xlabel("Result level")

    plt.savefig("level_count")

if __name__ == "__main__":
    main()