"""Run the two requested CLI submissions and assert observations by mission_id."""
import json
from pathlib import Path
import re
import subprocess
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_rescue_coordinator.ros_support import to_dict
from edgeai_robot_interfaces.msg import MissionIntakeResult, MissionPlan, MissionExecutionState
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('card04_acceptance')
contract = load_runtime_contract()
records = {'intake': [], 'plan': [], 'state': []}
subs = []
for key, cls, kind in [('intake', MissionIntakeResult, 'event_stream'),
                       ('plan', MissionPlan, 'snapshot'),
                       ('state', MissionExecutionState, 'snapshot')]:
    subs.append(node.create_subscription(
        cls, '/edge/mission_' + key, lambda msg, key=key: records[key].append(to_dict(msg)),
        qos_from_contract(contract, kind)))


def spin_until(predicate, seconds=90):
    end = time.monotonic() + seconds
    while not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert predicate(), 'Timed out waiting for evidence'


def submit(instruction):
    payload = json.dumps(dict(run_id=fixture['run_id'], original_instruction=instruction))
    command = ['ros2', 'service', 'call', '/edge/submit_mission',
               'edgeai_robot_interfaces/srv/SubmitMission', payload]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    spin_until(lambda: proc.poll() is not None, 20)
    output = proc.communicate()[0]
    print('$ ' + ' '.join(command), flush=True)
    print(output, flush=True)
    assert proc.returncode == 0 and 'accepted=True' in output
    mid = re.search(r"mission_id='([^']+)'", output).group(1)
    return mid


fixture = json.loads(Path('src/edgeai_rescue_coordinator/test/fixtures/task1_targets.complete.example.json').read_text())
assert 'T999' not in {t['target_id'] for t in fixture['targets']}
print('FROZEN TARGET IDS:', [t['target_id'] for t in fixture['targets']], flush=True)
try:
    spin_until(lambda: len(node.get_publishers_info_by_topic('/edge/robot_states')) == 1, 20)
    # Allow robot heartbeat and validator context discovery before submitting.
    start = time.monotonic()
    spin_until(lambda: time.monotonic() - start > 3, 5)
    rule = Path('docs/reference/Edge_AI_Robotics_Challenge_Rulebook_V0.md').read_text()
    instruction = re.search(r'Example instruction: “(.*?)”', rule).group(1)
    good = submit(instruction)
    spin_until(lambda: any(x['mission_id'] == good for x in records['intake']))
    intake = [x for x in records['intake'] if x['mission_id'] == good]
    assert len(intake) == 1 and intake[0]['outcome'] == 3, intake
    spin_until(lambda: any(x['mission_id'] == good and x['state'] == 2 for x in records['state']))
    plans = [x for x in records['plan'] if x['mission_id'] == good]
    assert len(plans) == 1 and plans[0]['valid']
    plan = plans[0]
    assert not intake[0]['candidate_plan']['valid']
    assert intake[0]['candidate_plan']['steps'] == plan['steps']
    deps = {s['target_id']: s['predecessor_step_ids'] for s in plan['steps']}
    assert deps == {'T2': [], 'T1': ['step-t2'], 'T3': ['step-t2'],
                    'TC1': ['step-t1', 'step-t3']}, deps
    for snapshot in [x for x in records['state'] if x['mission_id'] == good]:
        states = {s['step_id']: s['state'] for s in snapshot['steps']}
        for step in plan['steps']:
            if states[step['step_id']] in (1, 2, 3):
                assert all(states[p] == 3 for p in step['predecessor_step_ids'])
    bad = submit('Rescue T999.')
    spin_until(lambda: any(x['mission_id'] == bad for x in records['intake']))
    rejected = [x for x in records['intake'] if x['mission_id'] == bad]
    assert len(rejected) == 1 and rejected[0]['outcome'] == 2, rejected
    assert any(i['code'] == 102 and i['target_id'] == 'T999' for i in rejected[0]['issues'])
    start = time.monotonic()
    spin_until(lambda: time.monotonic() - start > 6, 8)
    assert not any(x['mission_id'] == bad for x in records['plan'])
    assert not any(x['mission_id'] == bad for x in records['state'])
    assert len([x for x in records['intake'] if x['mission_id'] == good]) == 1
    assert len([x for x in records['intake'] if x['mission_id'] == bad]) == 1
    publishers = {}
    for key in records:
        publishers[key] = [p.node_namespace.rstrip('/') + '/' + p.node_name for p in
                           node.get_publishers_info_by_topic('/edge/mission_' + key)]
    assert publishers == {'intake': ['/edge/mission_interpreter'],
                           'plan': ['/edge/mission_validator'],
                           'state': ['/edge/task2b_mock_allocator_TEST_ONLY']}, publishers
    print(json.dumps(dict(good_mission_id=good, bad_mission_id=bad,
                          publishers=publishers, records=records), indent=2), flush=True)
    print('PASS: two CLI submissions; DAG transitions; one intake each; invalid ID has no plan/state (6 s observation).', flush=True)
finally:
    Path('log/task2b/observations.json').write_text(json.dumps(records, indent=2))
    node.destroy_node()
    rclpy.try_shutdown()
