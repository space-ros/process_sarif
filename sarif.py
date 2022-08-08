#!/usr/bin/env python3

import os, json

from dataclasses import dataclass
from copy import deepcopy

from typing import List, Optional
from enum import Enum


@dataclass
class Artifact:
    uri: str
    uriBaseId: str
    filename: str

    @staticmethod
    def fromDict(artifact_dict: dict) -> "Artifact":
        uri = artifact_dict["location"]["uri"]
        uriBaseId = artifact_dict["location"]["uriBaseId"] if "uriBaseId" in artifact_dict["location"] else ""
        filename = os.path.basename(uri)

        return Artifact(uri=uri, uriBaseId=uriBaseId, filename=filename)

@dataclass
class Rule:
    ruleId: str
    description: str
    helpUri: str

    @staticmethod
    def fromDict(rule_dict: dict) -> "Rule":
        ruleId = rule_dict["id"] if "id" in rule_dict else ""
        description = rule_dict["shortDescription"]["text"] if "shortDescription" in rule_dict else ""
        helpUri = rule_dict["helpUri"] if "helpUri" in rule_dict else ""

        return Rule(ruleId=ruleId, description=description, helpUri=helpUri)

@dataclass
class Tool:
    name: str
    version: str
    informationUri: str
    rules: List[Rule]

    @staticmethod
    def fromDict(tool_dict: dict) -> "Tool":
        driver = tool_dict["driver"]

        name = driver["name"] if "name" in driver else ""
        version = driver["version"] if "version" in driver else ""
        informationUri = driver["informationUri"] if "informationUri" in driver else ""

        rules = []

        if "rules" in driver:
            for rule in driver["rules"]:
                rules.append(Rule.fromDict(rule))

        return Tool(name=name, version=version, informationUri=informationUri, rules=rules)

class Level(Enum):
    ERROR = 1
    WARNING = 2
    NOTE = 3
    NONE = 4

    @staticmethod
    def fromStr(level_str: str) -> Optional["Level"]:
        if level_str.lower() == "error":
            return Level.ERROR
        elif level_str.lower() == "warning":
            return Level.WARNING
        elif level_str.lower() == "note":
            return Level.NOTE
        elif level_str.lower() == "none":
            return Level.NONE
        else:
            print(f"[sarif][Level] Error! Level Enum fromStr called with invalid level_str {level_str}.")
            return None

class ResultKind(Enum):
    PASS = 1
    OPEN = 2
    INFORMATIONAL = 3
    REVIEW = 4
    FAIL = 5

    @staticmethod
    def fromStr(kind_str: str) -> Optional["ResultKind"]:
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
            print(f"[sarif][ResultKind] Error! ResultKind Enum fromStr called with invalid kind_str {kind_str}.")
            return None

@dataclass
class Region:
    startLine: int
    startColumn: int

    @staticmethod
    def fromDict(region_dict: dict) -> "Region":
        startLine = region_dict["startLine"] if "startLine" in region_dict else ""
        startColumn = region_dict["startColumn"] if "startColumn" in region_dict else ""
        
        return Region(startLine=startLine, startColumn=startColumn)

@dataclass
class Result:
    ruleId: str
    level: Level
    kind: ResultKind
    message: str
    artifact: Optional[Artifact]
    region: Optional[Region]
    tool: Tool
    package: str

    @staticmethod
    def fromDict(result_dict: dict, artifacts: List[Artifact], tool: Tool, verbose=True) -> "Result":
        ruleId = result_dict["ruleId"]
        level = Level.fromStr(result_dict["level"]) if "level" in result_dict else ""
        kind = ResultKind.fromStr(result_dict["kind"]) if "kind" in result_dict else ""
        message = result_dict["message"]["text"] if "text" in result_dict["message"] else ""
        
        if len(result_dict["locations"]) > 1 and verbose:
            print(f"[sarif][TODO] Multiple locations for a Result not yet implemented, only taking the first.")

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
                artifact = Artifact.fromDict({"location": artLoc})
            
            region = Region.fromDict(location["physicalLocation"]["region"])

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
    Wrapper for a Sarif object, to initialize all the sub-objects from json.
    '''
    tool: Optional[Tool]
    artifacts: List[Artifact]
    results: List[Result]

    @staticmethod
    def fromPath(path: str, verbose=True) -> Optional["SarifFile"]:
        with open(path) as sarif_f:
            try:
                sarif = json.load(sarif_f)
            except json.decoder.JSONDecodeError as e:
                if verbose:
                    print(f"Failed to parse {path}")
                    print(e)
                return None

        if "runs" in sarif:
            # TODO: Maybe change this assumption.
            # Grab the single run from the SARIF file.
            run = sarif["runs"][0]

            tool = None
            artifacts = []
            results = []

            if "tool" in run:
                tool = Tool.fromDict(run["tool"])

            if "artifacts" in run:
                for artifact in run["artifacts"]:
                    artifacts.append(Artifact.fromDict(artifact))

            if "results" in run:
                for result in run["results"]:
                    # Results need to be able to reference Artifacts and the Tool
                    results.append(Result.fromDict(result, artifacts, tool, verbose=verbose))

            return SarifFile(tool=tool, artifacts=artifacts, results=results)
                
if __name__ == "__main__":
    print("Please don't run me, I'm a library :(")