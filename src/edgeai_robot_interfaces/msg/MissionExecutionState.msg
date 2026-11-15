# 已验证任务计划的执行快照，由救援协调器发布于/edge/mission_state。
std_msgs/Header header
string run_id
string mission_id

uint8 STATE_IDLE=0
uint8 STATE_EXECUTING=1
uint8 STATE_COMPLETED=2
uint8 STATE_FAILED=3
uint8 STATE_ABORTED=4
uint8 state

MissionStepState[] steps
TaskReason reason
