import json
import os
from copy import deepcopy
from typing import Tuple
import pytest

from process_sarif.sarif import Artifact, Level, Region, Result, ResultKind, Rule, SarifFile, Tool
from process_sarif.sarif_helpers import remove_duplicate_results


def get_sarif_path(filename: str) -> str:
    return os.path.join(os.path.dirname(__file__), 'fixtures', filename)


def load_file(filename: str) -> Tuple[SarifFile, dict]:
    path = get_sarif_path(filename)

    loaded_file = SarifFile.from_path(path)

    json_dict = None
    with open(path) as f:
        json_dict = json.load(f)

    return json_dict, loaded_file


def test_empty_load():
    """Test loading of an empty sarif file."""
    json_dict, loaded_file = load_file('empty.sarif')

    expected_file = SarifFile(
        _tool=Tool(
            name='empty',
            version='0.0',
            informationUri='https://github.com/space-ros/process_sarif',
            rules=[]
        ),
        _artifacts=[],
        _results=[],
        _json_dict=json_dict,
        _path=get_sarif_path('empty.sarif')
    )

    assert loaded_file == expected_file


def test_simple_load():
    """Test loading of a SARIF file with one artifact and result."""
    json_dict, loaded_file = load_file('simple.sarif')

    artifact = Artifact(
                uri='relative/path/to/some_output.sarif',
                uriBaseId='/some/absolute/path',
                filename='some_output.sarif'
            )

    tool = Tool(
            name='simple',
            version='0.0',
            informationUri='https://github.com/space-ros/process_sarif',
            rules=[
                Rule(
                    ruleId='simple-rule-id',
                    description='This is a simple rule violation.',
                    helpUri='https://lmgtfy.com'
                )
            ]
        )

    expected_file = SarifFile(
        _tool=tool,
        _artifacts=[
            artifact
        ],
        _results=[
            Result(
                ruleId='simple-rule-id',
                level=Level.NOTE,
                kind=ResultKind.INFORMATIONAL,
                message='This is a simple rule violation.',
                artifact=artifact,
                artifactIdx=0,
                region=Region(
                    startLine=5,
                    startColumn=10
                ),
                tool=tool,
                package=''
            )
        ],
        _json_dict=json_dict,
        _path=get_sarif_path('simple.sarif')
    )

    assert expected_file == loaded_file


def test_load_json():
    """Test that SarifFile.json returns the same json as it loaded if no changes are made."""
    json_empty, loaded_empty = load_file('empty.sarif')
    json_simple, loaded_simple = load_file('simple.sarif')

    assert json_empty == loaded_empty.json_dict
    assert json_simple == loaded_simple.json_dict


@pytest.mark.skip(reason="Not yet implemented")
def test_modified_result():
    """Test that SarifFile.json is properly updated after modifying contents."""
    json_simple, loaded_simple = load_file('simple.sarif')

    artifact = Artifact(
                uri='relative/path/to/some_output.sarif',
                uriBaseId='/some/absolute/path',
                filename='some_output.sarif'
            )

    tool = Tool(
            name='simple',
            version='0.0',
            informationUri='https://github.com/space-ros/process_sarif',
            rules=[
                Rule(
                    ruleId='simple-rule-id',
                    description='This is a simple rule violation.',
                    helpUri='https://lmgtfy.com'
                )
            ]
        )

    # Deepcopy to avoid by-reference shenanigans. May not be needed.
    results = deepcopy(loaded_simple.results)
    results.append(Result(
        ruleId='simple-rule-id',
        level=Level.NOTE,
        kind=ResultKind.INFORMATIONAL,
        message='This is a simple rule violation.',
        artifact=artifact,
        artifactIdx=0,
        region=Region(
            startLine=5,
            startColumn=10
        ),
        tool=tool,
        package=''
    ))

    print(loaded_simple._json_dict)

    # TODO: Finish implementing test
    assert 0 == 1


def test_duplicate_result_single_file():
    """Test that duplicate results in one SARIF files reduced to a single result with sarif_helpers."""
    _, loaded = load_file('duplicate.sarif')
    _, simple         = load_file('simple.sarif')

    reduced = remove_duplicate_results([loaded])[0]

    # duplicate.sarif is just simple.sarif, with the single result occurring twice. Therefore, the only
    # change should be the path attribute of simple.
    simple._path = get_sarif_path('duplicate.sarif')

    assert reduced == simple

def test_duplicate_result_multiple_files():
    """Test that duplicate results across two SARIF files are eliminated with sarif_helpers."""
    json_dict, loaded_a = load_file('simple.sarif')
    _, loaded_b = load_file('simple.sarif')

    files = [loaded_a, loaded_b]

    reduced_files = remove_duplicate_results(files)

    # TODO: Rework this once sarif_helpers:remove_duplicate_results TODO's are addressed.
    json_dict["runs"][0]["results"] = []
    empty = SarifFile(
        _tool=Tool(
            name='simple',
            version='0.0',
            informationUri='https://github.com/space-ros/process_sarif',
            rules=[
                Rule(
                    ruleId='simple-rule-id',
                    description='This is a simple rule violation.',
                    helpUri='https://lmgtfy.com'
                )
            ]
        ),
        _artifacts=[
            Artifact(
                uri='relative/path/to/some_output.sarif',
                uriBaseId='/some/absolute/path',
                filename='some_output.sarif'
            )
        ],
        _results=[],
        _json_dict=json_dict,
        _path=get_sarif_path('simple.sarif')
    )

    assert reduced_files[1] == empty


@pytest.mark.skip(reason="Not yet implemented")
def test_duplicate_across_files():
    """Same as above, except the duplicates are in different SarifFiles."""
    assert 0 == 1
