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

@dataclass
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
            return None

@dataclass
class Region:
    '''
    A Region is the line (and optionally column) within an Artifact that a Result is referring to.
    '''

    startLine: int
    startColumn: int

    @staticmethod
    def from_dict(region_dict: dict, log_path: Optional[str] = None) -> "Region":
        startLine = region_dict["startLine"] if "startLine" in region_dict else ""
        startColumn = region_dict["startColumn"] if "startColumn" in region_dict else ""

        if startLine == "":
            _log(log_path, "\tStart line not set!")
        
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
    region: Optional[Region]
    tool: Tool
    package: str

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
                      region=region,
                      tool=tool,
                      package=package)

@dataclass
class SarifFile:
    '''
    A SarifFile is a composition of a Tool, one or multiple Artifacts, and one or multiple Results.
    Each of these objects handles its own initialization through calls to their from_dict() @staticmethods.
    '''
    tool: Optional[Tool]
    artifacts: List[Artifact]
    results: List[Result]

    @staticmethod
    def from_path(path: str, verbose=True, log_path: Optional[str] = None) -> Optional["SarifFile"]:
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

            return SarifFile(tool=tool, artifacts=artifacts, results=results)
                
if __name__ == "__main__":
    print("Please don't run me, I'm a library :(")