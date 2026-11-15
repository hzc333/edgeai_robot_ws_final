# 一次Task 2B自然语言指令从接收到验证结束的审计记录，由/edge/mission_interpreter
# 发布于/edge/mission_intake。每个mission_id只发布一次终态；日志器负责落盘。
# 规则6.7/6.9要求的“原始指令、模型及版本、解析结果、无效原因”全部在此记录。
std_msgs/Header header
string run_id
string mission_id
string original_instruction
builtin_interfaces/Time received_at

string model_name
string model_version
string prompt_version
# 模型原始文本输出，未经任何修改；解释器在调用模型前失败时为空。
string raw_model_output

uint8 OUTCOME_INTERPRETATION_FAILED=1
uint8 OUTCOME_REJECTED_BY_VALIDATOR=2
uint8 OUTCOME_ACCEPTED=3
uint8 outcome

# 解释器提交给验证器的候选计划；OUTCOME_INTERPRETATION_FAILED时steps为空。
MissionPlan candidate_plan
MissionValidationIssue[] issues
