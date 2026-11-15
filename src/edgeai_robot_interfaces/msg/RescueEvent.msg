# Append-only task transition/event used for audit and scoring evidence.
# 救援区占用、停留计时和完成判定只由/edge/rescue_verifier产生；
# 机器人动作结果不是得分证据。
std_msgs/Header header
string run_id
string event_id
string task_id
string target_id
string mission_id
string step_id

uint8 STAGE_TASK2A=1
uint8 STAGE_TASK2B=2
uint8 scoring_stage

uint8 EVENT_CREATED=0
uint8 EVENT_ASSIGNED=1
uint8 EVENT_NAVIGATION_STARTED=2
uint8 EVENT_ZONE_ENTERED=3
uint8 EVENT_DWELL_STARTED=4
uint8 EVENT_DWELL_RESET=5
uint8 EVENT_COMPLETED=6
uint8 EVENT_FAILED=7
uint8 EVENT_CANCELLED=8
uint8 EVENT_BLOCKED=9
uint8 EVENT_ZONE_EXITED=10
uint8 event_type

# robot_ids、rescue_zone_indices和robot_poses按相同下标对应；robot_poses为
# 事件发生时验证器采用的map坐标系机器人位姿，作为裁判核查证据。
string[] robot_ids
uint8[] rescue_zone_indices
geometry_msgs/Pose[] robot_poses
builtin_interfaces/Duration dwell_elapsed
TaskReason reason
