#!/usr/bin/env python3
"""Validate the repository's early-stage shared contracts without ROS startup."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = "examples/interfaces"
YAW_TOLERANCE = 1e-3
POSITION_TOLERANCE = 1e-3

# (schema, example files); a .jsonl example is validated line by line.
JSON_CONTRACTS = (
    ("schemas/target_list.schema.json", ("task1_targets.example.json",)),
    ("schemas/mission_plan.schema.json", ("mission_plan.example.json",)),
    (
        "schemas/mission_intake.schema.json",
        (
            "mission_intake_accepted.example.json",
            "mission_intake_rejected.example.json",
        ),
    ),
    ("schemas/event_record.schema.json", ("events.example.jsonl",)),
    ("schemas/manifest.schema.json", ("manifest.example.json",)),
    ("schemas/component_versions.schema.json", ("component_versions.example.json",)),
)


def load_json(relative_path: str) -> dict:
    path = ROOT / relative_path
    value = json.loads(path.read_text(encoding="utf-8"))
    print(f"OK JSON: {relative_path}")
    return value


def load_examples(relative_path: str) -> list[dict]:
    if relative_path.endswith(".jsonl"):
        lines = (ROOT / relative_path).read_text(encoding="utf-8").splitlines()
        values = [json.loads(line) for line in lines if line.strip()]
        print(f"OK JSONL: {relative_path} ({len(values)} records)")
        return values
    return [load_json(relative_path)]


def make_validator_factory(schemas: dict[str, dict]):
    """Return a callable building a validator that resolves cross-schema $ref."""
    try:
        import jsonschema
    except ImportError:
        return None

    by_id = {schema["$id"]: schema for schema in schemas.values()}
    format_checker = jsonschema.FormatChecker()
    try:
        from referencing import Registry, Resource

        registry = Registry().with_resources(
            (schema_id, Resource.from_contents(schema))
            for schema_id, schema in by_id.items()
        )

        def factory(schema: dict):
            return jsonschema.Draft202012Validator(
                schema, registry=registry, format_checker=format_checker
            )
    except ImportError:

        def factory(schema: dict):
            resolver = jsonschema.RefResolver.from_schema(schema, store=by_id)
            return jsonschema.Draft202012Validator(
                schema, resolver=resolver, format_checker=format_checker
            )

    return factory


def validate_json_contracts() -> dict[str, list[dict]]:
    schemas = {path: load_json(path) for path, _ in JSON_CONTRACTS}
    factory = make_validator_factory(schemas)
    if factory is None:
        print("WARN jsonschema unavailable; schema validation skipped")

    loaded: dict[str, list[dict]] = {}
    for schema_path, example_names in JSON_CONTRACTS:
        schema = schemas[schema_path]
        expected_version = schema["properties"]["schema_version"]["const"]
        if factory is not None:
            factory(schema).check_schema(schema)
        for name in example_names:
            example_path = f"{EXAMPLES}/{name}"
            records = load_examples(example_path)
            for index, record in enumerate(records, start=1):
                if record.get("schema_version") != expected_version:
                    raise ValueError(
                        f"{example_path}[{index}]: schema_version must be "
                        f"{expected_version}"
                    )
                if factory is not None:
                    errors = sorted(
                        factory(schema).iter_errors(record), key=lambda e: e.path
                    )
                    if errors:
                        error = errors[0]
                        location = "/".join(str(part) for part in error.path)
                        raise ValueError(
                            f"{example_path}[{index}] /{location}: {error.message}"
                        )
            if factory is not None:
                print(f"OK schema validation: {example_path}")
            loaded[name] = records
    return loaded


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(a: float, b: float) -> float:
    return abs(normalize_angle(a - b))


def yaw_from_planar_quaternion(quaternion: dict, where: str) -> float:
    norm = math.sqrt(sum(quaternion[axis] ** 2 for axis in "xyzw"))
    if abs(norm - 1.0) > 1e-4:
        raise ValueError(f"{where}: quaternion is not normalized ({norm:.6f})")
    if abs(quaternion["x"]) > 1e-6 or abs(quaternion["y"]) > 1e-6:
        raise ValueError(f"{where}: persisted poses must be yaw-only")
    return math.atan2(2.0 * quaternion["w"] * quaternion["z"],
                      1.0 - 2.0 * quaternion["z"] ** 2)


def expected_rescue_zones(target: dict, geometry: dict) -> list[tuple[float, float, float]]:
    """Development rescue geometry from docs/marker_and_rescue_geometry.md."""
    position = target["marker_pose"]["position"]
    yaw = target["marker_yaw_rad"]
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-math.sin(yaw), math.cos(yaw))
    zone_yaw = normalize_angle(yaw + math.pi)
    if target["target_type"] == "normal":
        rule = geometry["normal"]
        lateral_offsets = [0.0]
    else:
        rule = geometry["cooperative"]
        lateral_offsets = rule["lateral_offsets_m"]
    offset = rule["forward_offset_m"]
    return [
        (
            position["x"] + offset * forward[0] + lateral * left[0],
            position["y"] + offset * forward[1] + lateral * left[1],
            zone_yaw,
        )
        for lateral in lateral_offsets
    ]


def validate_target_list_semantics(target_list: dict, geometry: dict) -> None:
    where = "task1_targets.example.json"
    version = target_list["rescue_geometry"]["version"]
    source = target_list["rescue_geometry"]["source"]
    if (version is None) != (source is None):
        raise ValueError(f"{where}: rescue_geometry version/source must both be null or set")

    seen_ids: set[str] = set()
    for target in target_list["targets"]:
        target_where = f"{where}: {target['target_id']}"
        if target["target_id"] in seen_ids:
            raise ValueError(f"{target_where}: duplicate target_id")
        seen_ids.add(target["target_id"])

        yaw = yaw_from_planar_quaternion(
            target["marker_pose"]["orientation"], f"{target_where} marker_pose"
        )
        if angle_difference(yaw, target["marker_yaw_rad"]) > YAW_TOLERANCE:
            raise ValueError(f"{target_where}: marker_yaw_rad disagrees with quaternion")

        zones = target["rescue_zones"]
        indices = [zone["index"] for zone in zones]
        if indices != list(range(len(zones))):
            raise ValueError(f"{target_where}: rescue zone indices must be 0..n-1 in order")
        if version is None:
            if zones:
                raise ValueError(f"{target_where}: rescue_zones must be empty without geometry")
            continue
        expected_count = 1 if target["target_type"] == "normal" else 2
        if len(zones) != expected_count:
            raise ValueError(f"{target_where}: expected {expected_count} rescue zones")

        if source != "development":
            continue
        if version != geometry["version"]:
            raise ValueError(f"{target_where}: unknown development geometry {version}")
        for zone, (x, y, zone_yaw) in zip(zones, expected_rescue_zones(target, geometry)):
            pose = zone["pose"]
            actual_yaw = yaw_from_planar_quaternion(
                pose["orientation"], f"{target_where} zone {zone['index']}"
            )
            if (
                abs(pose["position"]["x"] - x) > POSITION_TOLERANCE
                or abs(pose["position"]["y"] - y) > POSITION_TOLERANCE
                or angle_difference(actual_yaw, zone_yaw) > YAW_TOLERANCE
            ):
                raise ValueError(
                    f"{target_where}: zone {zone['index']} does not match "
                    f"{version} (expected x={x:.3f}, y={y:.3f}, yaw={zone_yaw:.3f})"
                )
    print("OK semantics: target list poses and rescue geometry")


def validate_step_graph(steps: list[dict], where: str) -> None:
    step_ids = [step["step_id"] for step in steps]
    if len(step_ids) != len(set(step_ids)):
        raise ValueError(f"{where}: duplicate step_id")
    predecessors = {step["step_id"]: step["predecessor_step_ids"] for step in steps}
    for step_id, before in predecessors.items():
        unknown = set(before) - set(predecessors)
        if unknown:
            raise ValueError(f"{where}: {step_id} has unknown predecessors {sorted(unknown)}")

    visiting: set[str] = set()
    done: set[str] = set()

    def visit(step_id: str) -> None:
        if step_id in done:
            return
        if step_id in visiting:
            raise ValueError(f"{where}: dependency cycle through {step_id}")
        visiting.add(step_id)
        for predecessor in predecessors[step_id]:
            visit(predecessor)
        visiting.discard(step_id)
        done.add(step_id)

    for step_id in predecessors:
        visit(step_id)


def validate_mission_semantics(loaded: dict[str, list[dict]]) -> None:
    plan = loaded["mission_plan.example.json"][0]
    validate_step_graph(plan["steps"], "mission_plan.example.json")
    accepted = loaded["mission_intake_accepted.example.json"][0]
    if accepted["candidate_steps"] != plan["steps"]:
        raise ValueError("accepted intake candidate_steps must equal the validated plan steps")
    for key in ("run_id", "mission_id", "original_instruction", "model"):
        if accepted[key] != plan[key]:
            raise ValueError(f"accepted intake and mission plan disagree on {key}")
    print("OK semantics: mission dependency graph and intake record")


def validate_event_semantics(events: list[dict]) -> None:
    run_ids = {event["run_id"] for event in events}
    if len(run_ids) != 1:
        raise ValueError(f"events.example.jsonl: expected one run_id, got {sorted(run_ids)}")
    for expected_seq, event in enumerate(events, start=1):
        if event["seq"] != expected_seq:
            raise ValueError(f"events.example.jsonl: seq must increase from 1 without gaps")
        if event["category"] == "rescue":
            payload = event["payload"]
            lengths = {
                len(payload["robot_ids"]),
                len(payload["rescue_zone_indices"]),
                len(payload["robot_positions"]),
            }
            if len(lengths) != 1:
                raise ValueError(
                    f"events.example.jsonl seq {event['seq']}: robot arrays must align"
                )
    print("OK semantics: event log ordering")


def lookup(mapping: dict, dotted_path: str):
    value = mapping
    for key in dotted_path.split("."):
        if not isinstance(value, dict) or key not in value:
            raise KeyError(dotted_path)
        value = value[key]
    return value


def validate_yaml_if_available() -> dict | None:
    paths = (
        "config/competition_parameters.yaml",
        "config/interface_runtime.yaml",
        "config/development_scenarios.yaml",
    )
    try:
        import yaml
    except ImportError:
        print("WARN PyYAML unavailable; YAML semantic parsing skipped")
        return None

    values = {}
    for relative_path in paths:
        path = ROOT / relative_path
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        if value.get("schema_version") != 1:
            raise ValueError(f"{relative_path}: schema_version must be 1")
        values[relative_path] = value
        print(f"OK YAML: {relative_path}")

    parameters = values["config/competition_parameters.yaml"]
    run_modes = parameters["run_modes"]
    if run_modes["default"] not in run_modes["allowed"]:
        raise ValueError("competition_parameters: run_modes.default is not allowed")
    for dotted_path in run_modes["required_for_motion"]:
        try:
            lookup(parameters["official"], dotted_path)
        except KeyError:
            raise ValueError(
                f"competition_parameters: required_for_motion '{dotted_path}' "
                "does not exist under official"
            ) from None
        try:
            lookup(parameters["development"], dotted_path)
        except KeyError:
            raise ValueError(
                f"competition_parameters: required_for_motion '{dotted_path}' has no "
                "development fallback"
            ) from None

    development_rescue = parameters["development"]["rescue"]
    geometry = values["config/development_scenarios.yaml"]["rescue_geometry"]
    if development_rescue["geometry_version"] != geometry["version"]:
        raise ValueError("development rescue geometry versions disagree")
    if development_rescue["arrival_xy_tolerance_m"] >= development_rescue["position_tolerance_m"]:
        raise ValueError("arrival_xy_tolerance_m must be smaller than position_tolerance_m")

    runtime = values["config/interface_runtime.yaml"]
    phases = set(read_constants("msg/CompetitionState.msg", "PHASE_"))
    allowed_targets = phases | {"RESUME_INTERRUPTED"}
    for transition in runtime["phase_transitions"]:
        if transition["from"] not in phases or transition["to"] not in allowed_targets:
            raise ValueError(f"interface_runtime: unknown phase in transition {transition}")
        if not set(transition["requesters"]) <= {"operator", "system"}:
            raise ValueError(f"interface_runtime: unknown requester in {transition}")

    layout = (ROOT / "docs/package_and_node_layout.md").read_text(encoding="utf-8")
    for role, node in runtime["authority"].items():
        if node not in layout:
            raise ValueError(
                f"interface_runtime: authority {role} names {node}, "
                "which is missing from docs/package_and_node_layout.md"
            )
    print("OK semantics: run modes, geometry, phase transitions and authorities")
    return geometry


def validate_xml() -> None:
    paths = (
        "src/edgeai_robot_interfaces/package.xml",
        "src/edgeai_robot_description/package.xml",
        "src/edgeai_robot_description/urdf/edgeai_robot_real.urdf.xacro",
        "src/edgeai_robot_description/worlds/empty.sdf",
    )
    for relative_path in paths:
        ET.parse(ROOT / relative_path)
        print(f"OK XML: {relative_path}")


def read_constants(relative_path: str, prefix: str) -> dict[str, str]:
    path = ROOT / "src/edgeai_robot_interfaces" / relative_path
    constants = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(rf"\s*\w+\s+{prefix}([A-Z0-9_]+)=(\S+)\s*", line)
        if match:
            constants[match.group(1)] = match.group(2)
    return constants


def validate_interface_shape() -> None:
    package = ROOT / "src/edgeai_robot_interfaces"
    cmake_text = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    field_pattern = re.compile(
        r"^(?:[A-Za-z][A-Za-z0-9_/]*)(?:\[\])?\s+"
        r"[a-z][a-z0-9_]*$"
    )
    constant_pattern = re.compile(
        r"^(?:bool|byte|char|float32|float64|int(?:8|16|32|64)|"
        r"uint(?:8|16|32|64)|string)\s+[A-Z][A-Z0-9_]*=.*$"
    )

    for folder, expected_separators in (("msg", 0), ("srv", 1), ("action", 2)):
        for path in sorted((package / folder).glob(f"*.{folder}")):
            cmake_entry = f'"{folder}/{path.name}"'
            if cmake_entry not in cmake_text:
                raise ValueError(
                    f"{path.relative_to(ROOT)} is missing from rosidl_generate_interfaces"
                )
            separators = 0
            for line_number, raw_line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), start=1
            ):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line == "---":
                    separators += 1
                    continue
                if field_pattern.fullmatch(line) or constant_pattern.fullmatch(line):
                    continue
                raise ValueError(
                    f"{path.relative_to(ROOT)}:{line_number}: "
                    f"unrecognized interface declaration: {line}"
                )
            if separators != expected_separators:
                raise ValueError(
                    f"{path.relative_to(ROOT)}: expected {expected_separators} "
                    f"section separators, got {separators}"
                )
            print(f"OK interface shape: {path.relative_to(ROOT)}")

    # Phase and stage enumerations are duplicated across interfaces and must agree.
    reference_phases = read_constants("msg/CompetitionState.msg", "PHASE_")
    reference_phases.pop("NONE")
    for relative_path in ("msg/RobotPhaseAck.msg", "srv/SetCompetitionPhase.srv"):
        if read_constants(relative_path, "PHASE_") != reference_phases:
            raise ValueError(f"{relative_path}: PHASE_ constants differ from CompetitionState")
    if read_constants("msg/RescueTask.msg", "STAGE_") != read_constants(
        "msg/RescueEvent.msg", "STAGE_"
    ):
        raise ValueError("RescueTask and RescueEvent STAGE_ constants differ")
    print("OK interface enumerations are consistent")


def validate_required_files() -> None:
    required = (
        "docs/system_architecture_and_interfaces.md",
        "docs/rule_assumptions_and_tbd.md",
        "docs/package_and_node_layout.md",
        "docs/marker_and_rescue_geometry.md",
        "docs/decisions/0001-task1-v1-strategy.md",
        "config/competition_parameters.yaml",
        "config/interface_runtime.yaml",
        "config/development_scenarios.yaml",
        "src/edgeai_robot_interfaces/CHANGELOG.md",
    )
    for relative_path in required:
        if not (ROOT / relative_path).is_file():
            raise FileNotFoundError(relative_path)
    print("OK required preparation files")


def main() -> int:
    try:
        validate_required_files()
        loaded = validate_json_contracts()
        geometry = validate_yaml_if_available()
        if geometry is not None:
            validate_target_list_semantics(
                loaded["task1_targets.example.json"][0], geometry
            )
        validate_mission_semantics(loaded)
        validate_event_semantics(loaded["events.example.jsonl"])
        validate_xml()
        validate_interface_shape()
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    print("All available contract checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
