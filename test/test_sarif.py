import pytest, os, json

from process_sarif.sarif import SarifFile, Tool
from ament_index_python.packages import get_package_share_directory

def test_empty_load():
    empty_sarif = os.path.join(get_package_share_directory("process_sarif"), "resources", "empty.sarif")

    loaded_file = SarifFile.from_path(empty_sarif)

    json_dict = None
    with open(empty_sarif) as f:
        json_dict = json.load(f)

    expected_file = SarifFile(
        tool = Tool(
            name="empty",
            version="0.0",
            informationUri="https://github.com/space-ros/process_sarif",
            rules=[]
        ),
        artifacts = [],
        results = [],
        _json_dict = json_dict
    )

    assert loaded_file == expected_file