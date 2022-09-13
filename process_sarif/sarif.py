#!/usr/bin/env python3

import os, json

from dataclasses import dataclass
from copy import deepcopy

from typing import List, Optional
from enum import Enum


# This library provides dataclasses for a few of the SARIF components that we care
# about. The main entry point is SarifFile.from_path(), which takes in a file path
# that points to a SARIF file, and will return a SarifFile object with a Tool, 
# Artifacts, and Results. The sarif_helpers.py file provides some helper functions
# to load and process these objects in various ways.

def _log(log_path: str, message: str):
    '''
    Used by classes in this file to report issues. Not meant for outside use.
    '''

    if log_path is not None:
        with open(log_path, "a") as f:
            f.write(message + "\n")

@dataclass(eq=True, frozen=True)
class Artifact:
    '''
    An Artifact refers to a file. A SARIF file may have many artifacts, and Results within refer to them.

    uri: Relative or absolute path to the file in question.
    uriBaseId: os.path.join(uriBaseId, uri), producing the absolute path to this Artifact.
               This is only needed if the `uri` is relative.
    filename: os.path.basename(uriBaseId, uri), producing the filename of this Artifact.
    '''

    uri: str
    uriBaseId: str
    filename: str

    def to_dict(self) -> dict:
        return {
            "location": {
                "uri": self.uri,
                "uriBaseId": self.uriBaseId
            }
        }

    @staticmethod
    def from_dict(artifact_dict: dict, log_path: Optional[str] = None) -> "Artifact":
        uri = artifact_dict["location"]["uri"]
        uriBaseId = artifact_dict["location"]["uriBaseId"] if "uriBaseId" in artifact_dict["location"] else ""
        filename = os.path.basename(uri)

        if not os.path.isabs(os.path.join(uriBaseId, uri)):
            _log(log_path, f"\tArtifact {os.path.join(uriBaseId, uri)} is not absolute!")

        return Artifact(uri=uri, uriBaseId=uriBaseId, filename=filename)

    def __eq__(self: "Artifact", other: "Artifact") -> bool:
        '''
        Two Artifacts are equivalent if the absolute path between them is the same.
        TODO: Consider consolidating `uri` and `uriBaseId` into a single `path` member,
              composed from the two current valued in from_dict.
        '''
        return os.path.join(self.uriBaseId, self.uri) == os.path.join(other.uriBaseId, other.uri)

    def get_path(self) -> str:
        '''
        Returns the absolute path to this Artifact.
        '''

        return os.path.join(self.uriBaseId, self.uri)

@dataclass
class Rule:
    '''
    A Rule is a static analyzer rule that it may apply in Results.

    ruleId: The name of the rule.
    description: A brief description of the rule.
    helpUri: An optional link to a file/webpage for more information on the rule.
    '''

    ruleId: str
    description: str
    helpUri: str

    def to_dict(self) -> dict:
        return {
            "id": self.ruleId,
            "shortDescription": {
                "text": self.description
            },
            "helpUri": self.helpUri
        }

    @staticmethod
    def from_dict(rule_dict: dict) -> "Rule":
        ruleId = rule_dict["id"] if "id" in rule_dict else ""
        description = rule_dict["shortDescription"]["text"] if "shortDescription" in rule_dict else ""
        helpUri = rule_dict["helpUri"] if "helpUri" in rule_dict else ""

        return Rule(ruleId=ruleId, description=description, helpUri=helpUri)

@dataclass
class Tool:
    '''
    A Tool is the static analyzer. There is one per SARIF file.

    name: The name of the static analyzer.
    version: The version of the static analyzer.
    informationUri: A file/webpage with more information about the tool.
    rules: A list of Rules, see docstring for Rule().
    '''

    name: str
    version: str
    informationUri: str
    rules: List[Rule]

    def to_dict(self):
        return {
            "driver": {
                "name": self.name,
                "version": self.version,
                "informationUri": self.informationUri,
                "rules": [rule.to_dict() for rule in self.rules]
            }
        }

    @staticmethod
    def from_dict(tool_dict: dict, log_path: Optional[str] = None) -> "Tool":
        driver = tool_dict["driver"]

        name = driver["name"] if "name" in driver else ""
        version = driver["version"] if "version" in driver else ""
        informationUri = driver["informationUri"] if "informationUri" in driver else ""

        rules = []

        if "rules" in driver:
            for rule in driver["rules"]:
                rules.append(Rule.from_dict(rule))

        return Tool(name=name, version=version, informationUri=informationUri, rules=rules)

class Level(Enum):
    '''
    A Level is the "severity" of a Result.
    '''

    UNKNOWN = 0
    ERROR = 1
    WARNING = 2
    NOTE = 3
    NONE = 4

    @staticmethod
    def from_str(level_str: str) -> Optional["Level"]:
        if level_str.lower() == "error":
            return Level.ERROR
        elif level_str.lower() == "warning":
            return Level.WARNING
        elif level_str.lower() == "note":
            return Level.NOTE
        elif level_str.lower() == "none":
            return Level.NONE
        else:
            print(f"[sarif][Level] Error! Level Enum from_str called with invalid level_str {level_str}.")
            return None

    def __str__(self) -> str:
        # There must be a better way to do this...
        if self == Level.UNKNOWN:
            return "unknown"
        elif self == Level.ERROR:
            return "error"
        elif self == Level.WARNING:
            return "warning"
        elif self == Level.NOTE:
            return "note"
        elif self == Level.NONE:
            return "none"

class ResultKind(Enum):
    '''
    A ResultKind is similar but not identical to Level.
    '''

    UNKNOWN = 0
    PASS = 1
    OPEN = 2
    INFORMATIONAL = 3
    REVIEW = 4
    FAIL = 5

    @staticmethod
    def from_str(kind_str: str) -> Optional["ResultKind"]:
        if kind_str.lower() == "pass":
            return ResultKind.PASS
        elif kind_str.lower() == "open":
            return ResultKind.OPEN
        elif kind_str.lower() == "informational":
            return ResultKind.INFORMATIONAL
        elif kind_str.lower() == "review":
            return ResultKind.REVIEW
        elif kind_str.lower() == "fail":
            return ResultKind.FAIL
        else:
            print(f"[sarif][ResultKind] Error! ResultKind Enum from_str called with invalid kind_str {kind_str}.")
            return ResultKind.UNKNOWN

    def __str__(self) -> str:
        if self == ResultKind.UNKNOWN:
            return "unknown"
        elif self == ResultKind.PASS:
            return "pass"
        elif self == ResultKind.OPEN:
            return "open"
        elif self == ResultKind.INFORMATIONAL:
            return "informational"
        elif self == ResultKind.REVIEW:
            return "review"
        elif self == ResultKind.FAIL:
            return "fail"

@dataclass(eq=True, frozen=True)
class Region:
    '''
    A Region is the line (and optionally column) within an Artifact that a Result is referring to.
    '''

    startLine: int
    startColumn: int

    def to_dict(self) -> dict:
        region_dict = {}

        if self.startLine != -1:
            region_dict["startLine"] = self.startLine

        if self.startColumn != -1:
            region_dict["startColumn"] = self.startColumn

        return region_dict

    @staticmethod
    def from_dict(region_dict: dict, log_path: Optional[str] = None) -> "Region":
        startLine = region_dict["startLine"] if "startLine" in region_dict else -1
        startColumn = region_dict["startColumn"] if "startColumn" in region_dict else -1

        if startLine == "":
            _log(log_path, "\tStart line not set!")
        ""
        return Region(startLine=startLine, startColumn=startColumn)

@dataclass
class Result:
    '''
    A Result is a single output from a static analyzer.

    ruleId: The rule that is violated.
    level: See Level
    kind: see ResultKind
    message: The message ouptut from the static analyzer.
    artifact: See Artifact. Optional because not all tools currently conform and refer to one directly.
    region: See Region. Optional, same as above.
    tool: See Tool.
    package: The ROS2 package that this result is from. Not currently implemented.
    '''

    ruleId: str
    level: Level
    kind: ResultKind
    message: str
    artifact: Optional[Artifact]
    artifactIdx: int
    region: Optional[Region]
    tool: Tool
    package: str

    def to_dict(self) -> dict:
        return {
            "ruleId": self.ruleId,
            "level": str(self.level),
            "kind": str(self.kind),
            "message": {
                "text": self.message
            },
            "locations": [{
                "physicalLocation": {
                    "artifactLocation": {
                        "uri": self.artifact.uri,
                        "index": self.artifactIdx
                    },
                    "region": self.region.to_dict()
                }
            }]
        }

    @staticmethod
    def from_dict(result_dict: dict, artifacts: List[Artifact], tool: Tool, verbose=True, log_path: Optional[str] = None) -> "Result":
        ruleId = result_dict["ruleId"]
        level = Level.from_str(result_dict["level"]) if "level" in result_dict else Level.UNKNOWN
        kind = ResultKind.from_str(result_dict["kind"]) if "kind" in result_dict else ResultKind.UNKNOWN
        message = result_dict["message"]["text"] if "text" in result_dict["message"] else ""
        
        if len(result_dict["locations"]) > 1 and verbose:
            print("Multiple locations for a Result not yet implemented, only taking the first.")
            _log(log_path, f"\tFound multiple locations in a result, not yet implemented.")

        artifact = None
        region = None

        if len(result_dict["locations"]) >= 1:
            location = result_dict["locations"][0]
            artLoc = location["physicalLocation"]["artifactLocation"]
            
            # If we have an index, grab artifact by index
            if "index" in artLoc:
                idx = artLoc["index"]
                artifact = artifacts[idx]
            # If not, search by URI
            else:
                idx = -1
                uri = artLoc["uri"]
                for art in artifacts:
                    if uri == art.uri or uri == os.path.join(art.uriBaseId, art.uri):
                        artifact = art

            # If we didn't find the Artifact after searching, create a new one (not great)
            if artifact is None:
                if verbose: print(f"[sarif][Warning] Artifact not found while parsing Result.")
                _log(log_path, f"\tArtifact not found while parsing Result.")
                artifact = Artifact.from_dict({"location": artLoc})
            
            region = Region.from_dict(location["physicalLocation"]["region"])

        # TODO: Figure out how to implement this based on the selected Artifact.
        package = ""

        return Result(ruleId=ruleId,
                      level=level,
                      kind=kind,
                      message=message,
                      artifact=artifact,
                      artifactIdx=idx,
                      region=region,
                      tool=tool,
                      package=package)

@dataclass
class SarifFile:
    '''
    A SarifFile is a composition of a Tool, one or multiple Artifacts, and one or multiple Results.
    Each of these objects handles its own initialization through calls to their from_dict() @staticmethods.
    '''
    _tool: Optional[Tool]
    _artifacts: List[Artifact]
    _results: List[Result]

    _json_dict: str
    _path: str

    # Properties for each of the core SarifFile fields. Allows for _json_dict to be automatically
    # updated when they are set with .setter()s
    @property
    def path(self):
        return self._path

    @property
    def tool(self):
        return self._tool

    @tool.setter
    def tool(self, value: Tool):
        self._json_dict["runs"][0]["tool"] = value.to_dict()
        self._tool = value

    @property
    def artifacts(self):
        return self._artifacts

    @artifacts.setter
    def artifacts(self, value: List[Artifact]):
        self._json_dict["runs"][0]["artifacts"] = [artifact.to_dict() for artifact in value]
        self._artifacts = value

    @property
    def results(self):
        return self._results

    @results.setter
    def results(self, value: List[Result]):
        #print(self._json_dict["runs"][0]["results"])
        self._json_dict["runs"][0]["results"] = [result.to_dict() for result in value]
        self._results = value

    @property
    def json_dict(self):
        return self._json_dict

    @property
    def json(self):
        return json.dumps(self._json_dict, indent=2)

    def write_json(self, path: Optional[str] = None, verbose=False, log_path: Optional[str] = None) -> None:
        '''
        SarifFile.write_json applies the results field to the original _json_dict field, and
        writes this back to the path provided. If path is not provided, then this will overwrite
        where this file was originally loaded from.
        '''

        if path is None:
            path = self._path

        with open(path, "w+") as f:
            f.write(self.json)

    @staticmethod
    def from_path(path: str, verbose=True, log_path: Optional[str] = None) -> Optional["SarifFile"]:
        '''
        SarifFile.from_path constructs a SarifFile from a provided path.
        '''
        if verbose: print(f"Loading {path} as SarifFile.")
        with open(path) as sarif_f:
            try:
                sarif = json.load(sarif_f)
                _log(log_path, f"Loaded {path}.")
            except json.decoder.JSONDecodeError as e:
                if verbose:
                    print(f"Failed to parse {path}")
                    print(e)

                    _log(log_path, f"Failed to parse {path}.")
                    _log(log_path, str(e))
                return None

        if "runs" in sarif:
            # TODO: Maybe change this assumption.
            # Grab the single run from the SARIF file.
            run = sarif["runs"][0]

            if len(sarif["runs"]) > 1 and verbose:
                print("More than one run found!")
                _log(log_path, "\tMore than one run found!")

            tool = None
            artifacts = []
            results = []

            if "tool" in run:
                tool = Tool.from_dict(run["tool"], log_path=log_path)
            else:
                _log(log_path, "\tNo tool field found!")

            if "artifacts" in run:
                for artifact in run["artifacts"]:
                    artifacts.append(Artifact.from_dict(artifact, log_path=log_path))
            else:
                _log(log_path, "\tNo artifacts field found!")

            if "results" in run:
                for result in run["results"]:
                    # Results need to be able to reference Artifacts and the Tool
                    # Also, don't include duplicate (exactly identical) results.
                    result = Result.from_dict(result, artifacts, tool, verbose=verbose, log_path=log_path)
                    if result not in results:
                        results.append(result)

            else:
                _log(log_path, "\tNo results field found!")

            return SarifFile(
                _tool=tool,
                _artifacts=artifacts,
                _results=results,
                _json_dict=sarif,
                _path=path
            )
                
if __name__ == "__main__":
    print("Please don't run me, I'm a library :(")