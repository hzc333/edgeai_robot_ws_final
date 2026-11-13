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
