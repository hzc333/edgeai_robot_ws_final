# Task 02: mission plan validation core

Pure Python library, based on `edgeai_robot_ws_final` main
`95cc29443f643aa8bc0e2013922af2237c5d41b5`. No ROS imports, node, service,
publisher, scheduler or model integration. Runtime dependency: `jsonschema`
4.x, already declared in the repository's `pixi.toml`; no new environment
dependency is required. Package metadata uses a non-personal placeholder
maintainer address pending team ownership confirmation.

## API

```python
from edgeai_rescue_coordinator import validate_mission

accepted, issues = validate_mission(
    candidate, frozen_targets, available_robots,
    run_id, map_id, map_revision, target_revision, phase,
)
```

Inputs are finite JSON-compatible Python dictionaries/lists and scalars.
Candidate fields exactly match `MissionPlan.msg`, with `valid=False`; each
step uses `action_type=1` (`ACTION_RESCUE`). Integer fields reject booleans,
floats and values outside ROS ranges. The candidate schema also enforces
the nonempty identifiers, model metadata, instruction and steps constraints
from the mission log schema. Header uses the installed standard ROS
`std_msgs/Header` / `builtin_interfaces/Time` field definitions.

`frozen_targets` is the full `schemas/target_list.schema.json` format,
including map metadata, revision, frozen time and target records. The caller
supplies the authoritative current context and must ensure the snapshot
actually was frozen; a timestamp alone cannot establish provenance.

Return value is `(bool, list[dict])`. Every issue has exactly `code`,
`step_id`, `target_id`, `detail`, matching `MissionValidationIssue.msg`.
The validator never modifies any input or marks the candidate valid. Issues
are collected, deduplicated and sorted by these fields. Malformed parts are
rejected while checks on independently readable fields continue; no semantic
conclusion is invented for a field that cannot be read safely.

## Checks and explicit limits

| Code | Check |
| --- | --- |
| 100 | Invalid candidate/registry/context shape, types, ranges, duplicate registry IDs, or `valid=True` |
| 101 | Duplicate step ID |
| 102 | Target absent from frozen registry |
| 103 | **待队里澄清**: neither MissionStep nor the JSON step has `target_type`; no trigger is fabricated |
| 104 | Predecessor absent from plan |
| 105 | Dependency cycle, including self-cycle; iterative DFS |
| 106 | Normal target needs at least one available robot; cooperative needs two |
| 107 | Structured forbidden fields in raw candidate, including robot assignment, velocity, path and pose |
| 108 | Candidate or registry map ID / map revision / target revision differs from current context |
| 109 | Candidate or registry run ID differs from current context |
| 110 | Phase other than `CompetitionState.PHASE_TASK2B` (5) |

`valid=True` violates the ValidateMission.srv candidate contract, so it
returns 100 and is never accepted. Other independently detectable problems
are still returned; “direct rejection” does not mean stop after one issue.

107 is meaningful before converting the raw dictionary into a ROS message.
An actual MissionPlan message cannot carry these extra fields. Unknown
extra keys always produce 100; known forbidden keys also produce 107.
This is a structured-key check, not a natural-language classifier.
`original_instruction` remains uninterpreted, including mentions of robot
names or motion. A caller must not silently drop unknown keys before validation.

The architecture's target-type comparison and same-robot/two-zone check
cannot be implemented with the existing candidate fields: **待队里澄清**.
There is also no candidate schema-version field; message shape is checked
against the current contract, while registry schema version is checked as 2.
Mission log JSON is a different, post-validation format and is not accepted
as a candidate without explicit conversion. No shared fields are added.

Robot count does not represent individual capability or assignment. Independent
steps can run in sequence, so demands are not summed across the plan. This
follows the architecture's two distinct robots per cooperative rescue rule;
runtime scheduling and capability checks remain outside this core. Repeated
target IDs with distinct step IDs are not rewritten or deduplicated.

Only Task 2B is allowed, based on the documented mission protocol; acceptance
during FROZEN or earlier phases is not inferred. Mapping a full CompetitionState
snapshot (including emergency-stop state) into this API belongs to later ROS
integration. No collision, reachability or rescue-geometry feasibility checks
are claimed.

## Fixtures and contract checks

`test/fixtures/task1_targets.complete.example.json` copies the original
`task1_targets.example.json` and appends **synthetic** T2/T3 records copied
from the full T1 record with distinct target/marker IDs. Coordinates are
test placeholders, not measurements. Original T1/TC1 and metadata stay intact.
The original registry correctly rejects the example plan's T2/T3; the complete
fixture accepts its four steps. T9 from `mission_intake_rejected.example.json`
is rejected with 102. `candidate_from_log` in the tests is the explicit,
test-only adapter from log `action="rescue"` to message `action_type=1`.

The bundled registry schema is an exact copy with a byte-equality test.
Candidate field sets and action/phase/issue constants are compared to the
repository msg files. Test 103's trigger is explicitly skipped with the
reason above; it must not be reported as passed. There is also a negative
103 test and a test rejecting an attempted `target_type` extension with 100.

## Run tests offline

From the repository root, with the repository Pixi environment already installed:

```bash
pixi run python -m pytest -v -s -rs src/edgeai_rescue_coordinator/test
```

On the card 02 machine the new clone has no installed Pixi environment. Use
the existing interpreter read-only, with optional pytest plugins disabled:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/yueer/edgeai_robot_ws/.pixi/envs/default/bin/python \
  -m pytest -v -s -rs src/edgeai_rescue_coordinator/test
```

Linux network-isolated run (no host network changes):

```bash
unshare --user --map-root-user --net env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/yueer/edgeai_robot_ws/.pixi/envs/default/bin/python \
  -m pytest -v -s -rs src/edgeai_rescue_coordinator/test
```

The timing test measures 100 validations of four steps/four full target
records, reports minimum/mean/maximum latency, and compares the maximum with
`interface_runtime.yaml`'s 5.0-second timeout. It excludes interpreter startup
and schema loading. This is not a worst-case bound for arbitrary input sizes.
Determinism is checked across three separate processes/hash seeds.

Repository-wide `pixi run test` requires a colcon build and tests ROS packages;
it is not the focused command for this pure-core card. Full ROS integration
and the complete colcon suite: **未运行**. No requirement for either is claimed
as passed by the pure Python results.


## Card 03: dependency scheduler

Import `DependencyScheduler` from `edgeai_rescue_coordinator.scheduler`.
This pure standard-library module does not call the validator, create RescueTask,
select robots, infer completion, publish ROS messages, access a clock or use a network.
Baseline is card 02 commit `2889ad2`; validator and its tests are unchanged.

```python
from edgeai_rescue_coordinator.scheduler import DependencyScheduler

scheduler = DependencyScheduler(validated_plan)  # MissionPlan-shaped dict, valid=True
accepted_event = scheduler.apply({
    'kind': 'start',
    'run_id': validated_plan['run_id'],
    'mission_id': validated_plan['mission_id'],
    'step_id': 'step-t2',
    'task_id': 'task-from-allocator',
    'robot_ids': ['robot-from-allocator'],
    'stamp': {'sec': 1, 'nanosec': 0},
})
snapshot = scheduler.snapshot()
ready = scheduler.ready_steps()
```

The constructor copies the plan. Scheduling preconditions (valid=True, nonempty
steps, unique step IDs, legal priorities, known prerequisites, acyclic graph and
usable identities/header) are checked defensively; invalid preconditions raise
ValueError containing PLAN_INVALID (TaskReason 500), not a new wire error code.
This does not replace the upstream target/revision/context validator.
Two steps referencing the same target remain independent steps.

### External events and output

Events are a library input convention, not an added shared ROS interface.
The caller serializes calls on one scheduler instance. All events carry `kind`,
`run_id`, `mission_id`, and `stamp` (ROS sec/nanosec dictionary).
Step events also require `step_id`, nonempty `task_id`, and nonempty unique
`robot_ids`. The library records assignments supplied by the caller; it does not
validate physical robot capability. The same task ID cannot start two steps.

| kind | Accepted transition | Default reason |
| --- | --- | --- |
| start | READY to IN_PROGRESS | NONE (0) |
| complete | IN_PROGRESS to COMPLETED | NONE (0) |
| fail | IN_PROGRESS to FAILED | INTERNAL_ERROR (900) if no nonzero reason supplied |
| cancel | IN_PROGRESS to CANCELLED | USER_REQUEST (1) if no nonzero reason supplied |
| phase_changed | Active mission to ABORTED | PHASE_CHANGED (2) |
| safety_stop | Active mission to ABORTED | SAFETY_STOP (600) |

A step result must have the same task_id and ordered robot_ids as its accepted
start. Optional `reason` has exactly `code` and `detail`, using existing
TaskReason constants. `complete` takes its timestamp as completed_at; other
steps retain zero completed_at. No outcome is inferred from elapsed time.

RescueTask/RescueEvent are read-only references: upstream maps its accepted
assignment/execution start to `start`; authoritative RescueEvent COMPLETED,
FAILED, CANCELLED (6/7/8) map to complete/fail/cancel. Navigation arrival alone
is not mission completion. The future ROS adapter owns this mapping and supplies
phase/estop events; it is not implemented here. Snapshot publication remains
P4's `/edge/task_allocator`, as configured in interface_runtime.yaml.

`snapshot()` is a detached dictionary with exactly MissionExecutionState fields:
header, run_id, mission_id, state, steps, reason. Each step has exactly
MissionStepState fields: step_id, target_id, state, task_id, robot_ids,
completed_at, reason. Reason is {code, detail}. Header frame_id comes from the
plan; stamp is the maximum of the original stamp and accepted event timestamps.
Snapshot step order follows plan order. `ready_steps()` returns detached original
MissionStep dictionaries, sorted by descending priority, then lexical step_id.
Snapshot and ready-list mutation cannot change the scheduler's internal state.

### Dependencies, terminals and dirty events

All prerequisites must be COMPLETED before a step becomes READY. Priority never
bypasses this gate. Failure/cancellation blocks every transitive dependent with
DEPENDENCY_FAILED (400), in topological order. Unrelated branches keep running.

The mission starts IDLE with roots READY, becomes EXECUTING after an accepted
step event, COMPLETED when every step completed, or FAILED when every step is
terminal and at least one did not complete. Overall failure reason is the first
unsuccessful step's reason in plan order. Thus failed branches do not leave a
finished graph stuck EXECUTING, while active independent work may still finish.

Phase change / safety stop cancels only WAITING_FOR_PREDECESSORS, READY and
IN_PROGRESS steps, then marks the active mission ABORTED. Steps already
COMPLETED, FAILED, BLOCKED or CANCELLED retain their original state, reason
and other recorded fields. Entire terminal missions are immutable and all
later events return False.

`apply()` returns False without mutation for unknown identities, invalid fields,
unknown reasons, wrong task/robots, duplicates, illegal transitions or timestamps
older than the last accepted event for that step. Equal timestamps are allowed
for ordered transitions. A delayed interrupt older than the snapshot is ignored.
Results arriving before start are ignored, not buffered; the caller must order
or retransmit events. Calls must be serialized, including obtaining a snapshot
and ready list if an atomic pair is needed.

There is no promise of termination without external outcomes: an IN_PROGRESS
step cannot be declared failed just because no event arrived. The caller must
supply an existing timeout failure reason or interrupt for lost/stalled work.
Timeout enforcement, transport reordering/replay, and ROS adapter integration
are **未验证 / 未运行** here. With external terminal events supplied, valid DAGs
finish; malformed graphs are rejected at construction.

### Card 03 verification

Run from this package directory (not the repository root):

```bash
cd /home/yueer/edgeai_ws_card02/src/edgeai_rescue_coordinator
unshare --user --map-root-user --net env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/yueer/edgeai_robot_ws/.pixi/envs/default/bin/python -m pytest -q -rs test
```

This includes the unchanged validator tests and the package copyright/flake8/
pep257 checks. Pytest may report unknown ament marker warnings because plugin
autoload is disabled; these are not failures. `test/test_scheduler.py` compares
all state and reason constants against msg files, exercises the rulebook example,
priority ties, failure propagation, interruption, duplicate/out-of-order events,
and generates 200 DAGs with seeds 31000 through 31199 (1–35 steps, shuffled plan
order). Each graph checks every completed step's prerequisites, reaches COMPLETED,
and produces identical snapshots when the same events are replayed.
No held-out data, templates, tests or reports are used.
