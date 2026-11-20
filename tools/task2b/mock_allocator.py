"""TEST ONLY: serialized scheduler adapter; never impersonates Ye's task_allocator.

Uses repository MockRobotAgent actions. Holding feedback triggers a TEST verifier
completion after both participants hold, followed by cancellation/release. This
is not production rescue-zone/dwell verification or a collision planner.
"""
import time

from edgeai_rescue_coordinator.ros_support import ContextNode, message, to_dict
from edgeai_rescue_coordinator.scheduler import DependencyScheduler
from edgeai_robot_interfaces.action import ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState, MissionExecutionState, MissionPlan, RobotState, RobotStateArray)
import rclpy
from rclpy.action import ActionClient


class MockAllocator(ContextNode):
    def __init__(self):
        super().__init__('task2b_mock_allocator_TEST_ONLY')
        self.scheduler = None
        self.active = None
        self.seen = set()
        self.states = {}
        self.targets_by_id = {t['target_id']: t for t in self.targets['targets']}
        self.robot_clients = {}
        for rid in ('robot_1', 'robot_2'):
            self.robot_clients[rid] = ActionClient(self, ExecuteRescueTask,
                                              f'/{rid}/execute_rescue_task')
            self.create_subscription(RobotState, f'/{rid}/state',
                                     lambda m: self.states.update({m.robot_id: (m, time.monotonic())}),
                                     self.qos('robot_state'))
        self.state_pub = self.create_publisher(MissionExecutionState, self.topic('mission_state'),
                                               self.qos('snapshot'))
        self.competition_pub = self.create_publisher(
            CompetitionState, self.topic('competition_state'), self.qos('control_state'))
        self.robots_pub = self.create_publisher(
            RobotStateArray, self.topic('robot_states'), self.qos('robot_state'))
        self.create_subscription(MissionPlan, self.topic('mission_plan'), self.on_plan,
                                 self.qos('snapshot'))
        self.create_timer(0.1, self.tick)
        self.get_logger().warning('TEST ONLY state publisher; Ye integration UNVERIFIED')

    def on_plan(self, plan):
        if not plan.valid or plan.mission_id in self.seen:
            return
        if self.scheduler and self.scheduler.snapshot()['state'] not in (2, 3, 4):
            self.get_logger().error('TEST adapter supports only one active mission')
            return
        self.seen.add(plan.mission_id)
        self.scheduler = DependencyScheduler(to_dict(plan))
        self.publish()

    def publish(self):
        self.state_pub.publish(message(MissionExecutionState, self.scheduler.snapshot()))

    def event(self, kind, reason=None):
        a = self.active
        event = dict(run_id=self.run_context['run_id'],
                     mission_id=self.scheduler.snapshot()['mission_id'],
                     step_id=a['step']['step_id'], task_id=a['task_id'],
                     robot_ids=a['robots'], kind=kind,
                     stamp=to_dict(self.get_clock().now().to_msg()))
        if reason:
            event['reason'] = reason
        if not self.scheduler.apply(event):
            raise RuntimeError(f'scheduler rejected {event}')
        self.publish()

    def tick(self):
        state = CompetitionState(**self.run_context, phase=CompetitionState.PHASE_TASK2B,
                                 phase_sequence=1)
        state.header.stamp = self.get_clock().now().to_msg()
        self.competition_pub.publish(state)
        now = time.monotonic()
        self.robots_pub.publish(RobotStateArray(
            run_id=self.run_context['run_id'],
            robots=[m for m, t in self.states.values() if now - t < 1.0]))
        if not self.scheduler:
            return
        if self.active:
            a = self.active
            if a['ending']:
                if a['pending_goals'] == 0 and all(f.done() for f in a['results']):
                    self.active = None
                return
            if a['error'] or now - a['started'] > 60:
                self.event('fail', dict(code=101, detail=a['error'] or 'TEST action timeout'))
                a['ending'] = True
                for h in a['handles']:
                    h.cancel_goal_async()
            elif len(a['holding']) == len(a['robots']):
                # Explicit test-verifier event, NOT a robot success/result.
                self.event('complete')
                a['ending'] = True
                for h in a['handles']:
                    h.cancel_goal_async()
            return
        ready = self.scheduler.ready_steps()
        if not ready or not all(c.server_is_ready() for c in self.robot_clients.values()):
            return
        step = ready[0]
        target = self.targets_by_id[step['target_id']]
        robots = list(self.robot_clients)[:2 if target['target_type'] == 'cooperative' else 1]
        mid = self.scheduler.snapshot()['mission_id']
        a = dict(step=step, robots=robots, task_id=f'{mid}-{step["step_id"]}',
                 started=now, handles=[], results=[], holding=set(), ending=False, error='',
                 pending_goals=len(robots))
        self.active = a
        self.event('start')
        for i, rid in enumerate(robots):
            goal = ExecuteRescueTask.Goal(
                run_id=self.run_context['run_id'], map_id=self.run_context['map_id'],
                map_revision=self.run_context['map_revision'], task_id=a['task_id'],
                target_id=step['target_id'], target_type=2 if len(robots) == 2 else 1,
                rescue_zone_index=i, arrival_xy_tolerance_m=0.05)
            goal.rescue_pose.header.frame_id = 'map'
            from geometry_msgs.msg import Pose
            goal.rescue_pose.pose = message(Pose, target['rescue_zones'][i]['pose'])
            future = self.robot_clients[rid].send_goal_async(
                goal, feedback_callback=lambda msg, rid=rid, a=a: self.feedback(msg, rid, a))
            future.add_done_callback(lambda f, a=a: self.accepted(f, a))

    def accepted(self, future, a):
        a['pending_goals'] -= 1
        handle = future.result()
        if not handle.accepted:
            a['error'] = 'Mock robot rejected goal'
            return
        a['handles'].append(handle)
        result = handle.get_result_async()
        a['results'].append(result)
        if a['ending']:
            handle.cancel_goal_async()
        result.add_done_callback(lambda f: self.ended(f, a))

    def ended(self, future, a):
        if not a['ending']:
            a['error'] = 'Mock action ended before test verifier completion: ' + str(
                future.result().result.reason)

    def feedback(self, msg, rid, a):
        if msg.feedback.execution_state == ExecuteRescueTask.Feedback.STATE_HOLDING:
            a['holding'].add(rid)
        else:
            a['holding'].discard(rid)


rclpy.init()
node = MockAllocator()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.try_shutdown()
