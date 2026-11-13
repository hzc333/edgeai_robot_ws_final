"""Validate untrusted MissionPlan-shaped dictionaries without mutating inputs.

Contract baseline: edgeai_robot_ws_final main 95cc294. Codes and message
shapes are checked against the repository by the contract tests.
"""

import json
import math
from collections import Counter
from importlib.resources import files

from jsonschema import Draft202012Validator, FormatChecker, validators


CODE_SCHEMA_INVALID = 100
CODE_DUPLICATE_STEP_ID = 101
CODE_UNKNOWN_TARGET = 102
CODE_TARGET_TYPE_MISMATCH = 103  # Reserved: no candidate target_type field.
CODE_UNKNOWN_PREDECESSOR = 104
CODE_DEPENDENCY_CYCLE = 105
CODE_INSUFFICIENT_ROBOTS = 106
CODE_FORBIDDEN_CONTENT = 107
CODE_STALE_REVISION = 108
CODE_RUN_ID_MISMATCH = 109
CODE_PHASE_NOT_ALLOWED = 110
ACTION_RESCUE = 1
PHASE_TASK2B = 5

# JSON Schema accepts 1.0 as an integer; ROS integer fields must be Python ints.
_Checker = Draft202012Validator.TYPE_CHECKER.redefine(
    'integer', lambda checker, value: type(value) is int)
_Validator = validators.extend(Draft202012Validator, type_checker=_Checker)


def _load_schema(name):
    return json.loads(files(__package__).joinpath('schemas', name).read_text())


_CANDIDATE = _Validator(_load_schema('candidate.schema.json'))
_TARGETS = _Validator(_load_schema('target_list.schema.json'),
                      format_checker=FormatChecker())
_FORBIDDEN = frozenset({
    'robot', 'robots', 'robot_id', 'robot_ids', 'assigned_robot_id',
    'assigned_robot_ids', 'robot_assignment', 'assignments', 'assigned_robots',
    'speed', 'velocity', 'linear_velocity', 'angular_velocity', 'cmd_vel',
    'twist', 'path', 'trajectory', 'waypoints', 'pose', 'position',
    'orientation', 'coordinates',
})


def _is_json_tree(value):
    """Reject cycles, non-string keys and non-JSON values before schema traversal."""
    active = set()
    stack = [(value, False)]
    while stack:
        item, leaving = stack.pop()
        if leaving:
            active.remove(id(item))
        elif type(item) in (dict, list):
            if id(item) in active:
                return False
            active.add(id(item))
            stack.append((item, True))
            if type(item) is dict:
                if any(type(key) is not str for key in item):
                    return False
                stack.extend((child, False) for child in item.values())
            else:
                stack.extend((child, False) for child in item)
        elif item is not None and type(item) not in (str, int, bool):
            if type(item) is not float or not math.isfinite(item):
                return False
    return True


def validate_mission(candidate, frozen_targets, available_robots, run_id,
                     map_id, map_revision, target_revision, phase):
    """Return ``(accepted, issues)``; never edit steps or any other input.

    candidate follows MissionPlan.msg, including header and valid=False.
    frozen_targets follows schemas/target_list.schema.json. Context is the
    caller's authoritative snapshot. Only PHASE_TASK2B permits validation.
    Quantity checks require one robot for normal and two for cooperative
    rescue; independent steps need not execute simultaneously.

    issues contain only MissionValidationIssue.msg fields. Code 103 cannot
    be inferred from this interface. Forbidden keys are checked in raw
    dictionaries before ROS conversion; text values are not interpreted.
    """
    issues = []

    def issue(code, detail, step=None):
        step = step if isinstance(step, dict) else {}
        issues.append(dict(code=code,
                           step_id=step.get('step_id') if isinstance(
                               step.get('step_id'), str) else '',
                           target_id=step.get('target_id') if isinstance(
                               step.get('target_id'), str) else '',
                           detail=detail))

    if not _is_json_tree(candidate):
        issue(CODE_SCHEMA_INVALID, 'candidate must be a finite JSON-compatible tree')
        candidate = None
    if not _is_json_tree(frozen_targets):
        issue(CODE_SCHEMA_INVALID, 'frozen_targets must be a finite JSON-compatible tree')
        frozen_targets = None

    for label, value, validator in (
            ('candidate', candidate, _CANDIDATE),
            ('frozen_targets', frozen_targets, _TARGETS)):
        for error in validator.iter_errors(value):
            path = '.'.join(str(part) for part in error.absolute_path)
            step = None
            parts = list(error.absolute_path)
            if label == 'candidate' and len(parts) >= 2 and parts[0] == 'steps':
                step = candidate['steps'][parts[1]]
            # Do not copy entire input values into diagnostic output.
            issue(CODE_SCHEMA_INVALID,
                  f'{label}.{path}: violates {error.validator}', step)

    count_ok = type(available_robots) is int and available_robots >= 0
    if not count_ok:
        issue(CODE_SCHEMA_INVALID, 'available_robots must be a nonnegative integer')
    for name, value in (('run_id', run_id), ('map_id', map_id)):
        if not isinstance(value, str) or not value:
            issue(CODE_SCHEMA_INVALID, f'context.{name} must be a nonempty string')
    for name, value, minimum in (('map_revision', map_revision, 1),
                                 ('target_revision', target_revision, 0)):
        if type(value) is not int or not minimum <= value <= 2**32 - 1:
            issue(CODE_SCHEMA_INVALID, f'context.{name} must be a valid uint32')
    if type(phase) is not int or phase != PHASE_TASK2B:
        issue(CODE_PHASE_NOT_ALLOWED, 'mission validation requires PHASE_TASK2B')

    plan = candidate if isinstance(candidate, dict) else {}
    registry = frozen_targets if isinstance(frozen_targets, dict) else {}
    for label, obj in (('candidate', plan), ('frozen_targets', registry)):
        if 'run_id' in obj and obj['run_id'] != run_id:
            issue(CODE_RUN_ID_MISMATCH, f'{label}.run_id differs from current run')
    registry_map = registry.get('map')
    registry_map = registry_map if isinstance(registry_map, dict) else {}
    for name, obj, key, current in (
            ('candidate.map_id', plan, 'map_id', map_id),
            ('candidate.map_revision', plan, 'map_revision', map_revision),
            ('candidate.target_revision', plan, 'target_revision', target_revision),
            ('frozen_targets.map.map_id', registry_map, 'map_id', map_id),
            ('frozen_targets.map.revision', registry_map, 'revision', map_revision),
            ('frozen_targets.revision', registry, 'revision', target_revision)):
        if key in obj and obj[key] != current:
            issue(CODE_STALE_REVISION, f'{name} differs from current snapshot')

    # Iterative traversal: inspect structured keys, never original_instruction text.
    pending = [(plan, 'candidate', None)]
    while pending:
        value, path, step = pending.pop()
        if isinstance(value, dict):
            for key, child in value.items():
                if isinstance(key, str) and key.lower() in _FORBIDDEN:
                    issue(CODE_FORBIDDEN_CONTENT, f'{path}.{key}: forbidden field', step)
                pending.append((child, f'{path}.{key}', step))
        elif isinstance(value, list):
            for index, child in enumerate(value):
                pending.append((child, f'{path}[{index}]',
                                child if path == 'candidate.steps' else step))

    raw_targets = registry.get('targets', [])
    targets = {}
    marker_ids = set()
    for target in raw_targets if isinstance(raw_targets, list) else []:
        if not isinstance(target, dict):
            continue
        tid = target.get('target_id')
        if isinstance(tid, str):
            if tid in targets:
                issue(CODE_SCHEMA_INVALID, 'frozen_targets has duplicate target_id')
            targets[tid] = target
        marker = target.get('marker_id')
        if type(marker) is int:
            if marker in marker_ids:
                issue(CODE_SCHEMA_INVALID, 'frozen_targets has duplicate marker_id')
            marker_ids.add(marker)

    raw_steps = plan.get('steps', [])
    steps = [s for s in raw_steps if isinstance(s, dict)] if isinstance(raw_steps, list) else []
    counts = Counter(s['step_id'] for s in steps if isinstance(s.get('step_id'), str))
    graph = {sid: [] for sid in counts}
    step_by_id = {}
    for step in steps:
        sid, tid = step.get('step_id'), step.get('target_id')
        if isinstance(sid, str):
            step_by_id.setdefault(sid, step)
            if counts[sid] > 1:
                issue(CODE_DUPLICATE_STEP_ID, 'step_id must be unique', step)
        if isinstance(tid, str):
            if tid not in targets:
                issue(CODE_UNKNOWN_TARGET, 'target absent from frozen registry', step)
            elif count_ok:
                kind = targets[tid].get('target_type')
                required = 1 if kind == 'normal' else 2 if kind == 'cooperative' else None
                if required is not None and available_robots < required:
                    issue(CODE_INSUFFICIENT_ROBOTS,
                          f'target requires {required} available robots', step)
        predecessors = step.get('predecessor_step_ids', [])
        for pred in predecessors if isinstance(predecessors, list) else []:
            if not isinstance(pred, str):
                continue
            if pred not in counts:
                issue(CODE_UNKNOWN_PREDECESSOR, 'predecessor absent from plan', step)
            elif isinstance(sid, str):
                graph[sid].append(pred)

    # Iterative DFS avoids Python recursion limits on long, legitimate plans.
    colors = {}
    for root in graph:
        if colors.get(root):
            continue
        colors[root] = 1
        stack = [(root, iter(graph[root]))]
        while stack:
            node, edges = stack[-1]
            pred = next(edges, None)
            if pred is None:
                colors[node] = 2
                stack.pop()
            elif colors.get(pred) == 1:
                issue(CODE_DEPENDENCY_CYCLE, 'dependency graph contains a cycle',
                      step_by_id[node])
            elif not colors.get(pred):
                colors[pred] = 1
                stack.append((pred, iter(graph[pred])))

    # Stable order, independent of hash seed; collapse identical diagnostics.
    unique = {(i['code'], i['step_id'], i['target_id'], i['detail']) for i in issues}
    result = [dict(zip(('code', 'step_id', 'target_id', 'detail'), row))
              for row in sorted(unique)]
    return not result, result
