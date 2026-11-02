**Edge AI Robotics Challenge**

**Competition Rulebook**

**Document Scope and Status**

This rulebook is intended for participating teams and event organizers. It defines the competition tasks, system boundaries, operating rules, scoring framework, and milestone requirements. Parameters marked TBD have not yet been finalized. Provisions explicitly marked as tentative rules are subject to organizer confirmation before formal release. All remaining content in this version is part of the current draft and remains subject to the final official version.

Software installation, ROS 2 interfaces, startup commands, TF configuration, calibration procedures, and example code will be provided in a separate Technical Tutorial. Robot configuration and technical resources are summarized in Chapter 2 and Appendix B.

# 1 Challenge Overview

The Edge AI Robotics Challenge focuses on autonomous search-and-rescue in an unknown indoor environment. It evaluates autonomous exploration, environmental perception, multi-robot collaboration, edge computing, and natural-language-driven high-level mission decision-making.

Teams will develop an autonomous search-and-rescue system using organizer-provided mobile robot platforms, basic interfaces, and a simulation environment. The system first explores the environment, builds a map, and identifies rescue targets, and then uses the acquired information to coordinate multiple robots for rescue operations.

| **Task**                                    | **Objective and Output**                                                                                                                                                                      |
|---------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Task 1 - Autonomous Exploration and Mapping | Use up to two robots to autonomously explore an unknown environment, build a 2D occupancy grid map, identify and localize rescue targets, and output a unified map and target list.           |
| Task 2 - Multi-Robot Rescue                 | Use the map and target information obtained in Task 1 to coordinate multiple robots through the edge node for normal rescue, cooperative rescue, and natural-language-guided rescue missions. |

**Unknown Environment → Exploration & Mapping → Target Detection & Localization → Map & Target List → Edge-Node Coordination → Multi-Robot Rescue**

LLM-guided rescue is an advanced mode within Task 2 and does not constitute a separate Task 3. Rescue is abstracted as reaching a specified rescue zone and remaining there continuously for a required duration; physical transport of rescue targets is not required.

## Reading Guide

Chapter 2 describes the system configuration; Chapters 3-4 define the arena and competition flow; Chapters 5-6 specify the competition tasks; Chapters 7-8 define general rules and scoring; Chapters 9-11 cover milestones, submissions, and rule updates; Appendix A lists parameters still to be finalized; and Appendix B summarizes supporting technical resources.

# 2 System Configuration

## 2.1 Robot Hardware

The competition uses differential-drive wheeled mobile robots. The hardware configuration is listed below. Detailed drivers, calibration parameters, and operating limits will follow the official Robot Setup and Technical Tutorial.

| **Component**    | **Model / Configuration**                                       |
|------------------|-----------------------------------------------------------------|
| Motor controller | Phidgets 4A DC Phidget                                          |
| IMU              | Phidgets PhidgetSpatial Precision 3/3/3                         |
| Interface hub    | Phidgets VINT Hub Phidget                                       |
| Drive motor      | Pololu 50:1 Metal Gearmotor with 64 CPR Encoder, Product \#4753 |
| Wheels           | BaneBots 3 7/8-inch wheels                                      |
| RGB-D camera     | Intel RealSense D435i                                           |
| LiDAR            | SLAMTEC RPLIDAR; the supplied product link corresponds to A1    |
| Onboard GPU      | NVIDIA GeForce GT 1030                                          |
| Robot computer   | Intel NUC11TNKi5                                                |

Encoder note: the listed Pololu motor specifies 64 CPR at the motor shaft. Under the manufacturer counting convention, the 50:1 gearbox corresponds to 3,200 counts per output-shaft revolution. Actual odometry should use calibrated whole-robot parameters.

![Robot configuration](Edge_AI_Robotics_Challenge_Rulebook_V0_assets/media/image1.png)

## 2.2 Number of Robots and Edge Node

Each team may use up to four robots. In Task 1, teams may choose to use either one or two robots. Task 2 may use multiple robots, and cooperative rescue requires at least two robots. The actual number of physical robots available during development and milestone assessments, and the corresponding scheduling arrangements, will be announced separately by the organizers.

Each team will use an organizer-provided edge computing node, primarily for map and state aggregation, task coordination, and local LLM inference. The exact edge-node hardware configuration, memory, and compute/acceleration resources will be specified in later technical documentation.

## 2.3 Communication and Development Support

**Communication Architecture**

Robots, the organizer-provided edge node, and the human operator communicate through the provided local communication infrastructure. This edition of the challenge does not intentionally introduce packet loss, bandwidth limitation, or communication degradation as part of the competition.

Communication may occur between:

- robots and the edge node;

- the edge node and the human operator; and

- robots themselves, where supported by the provided communication infrastructure.

Teams may design their own communication, coordination, and information-sharing strategies within the provided infrastructure. Shared information may include robot state, localization information, target observations, task status, task assignments, coordination information, and other data required by the team system.

The edge node may be used for global state aggregation, task coordination, shared information processing, and local LLM inference, but the competition does not mandate a specific centralized or decentralized coordination architecture.

Human-operator interaction is limited to the functions explicitly permitted by the competition rules, such as system start, monitoring, submission of organizer-provided mission instructions, reset requests, and emergency stop.

In the event of communication loss, robots shall follow a predefined safe behavior. Detailed network configuration, ROS 2 namespaces, message interfaces, and communication setup will be specified in the Technical Tutorial.

**Development Support**

The development environment uses ROS 2 and includes Gazebo simulation support; versions and interfaces will be specified in the Technical Tutorial.

The organizers provide the robot platform, interfaces, simulation resources, a single-robot SLAM baseline, and a basic tutorial. Teams are responsible for exploration strategy, target detection and localization, task allocation, multi-robot rescue, and local LLM integration.

Mentors with expertise in different technical areas, including robotics, communication, control, and AI/computer science, will be available to provide technical guidance during the development period. Teams may consult the relevant mentors when they encounter implementation, integration, or system-level technical issues. Mentors provide guidance and troubleshooting support but do not develop the competition solution on behalf of the team

# 3 Competition Environment

## 3.1 Arena

The arena is an indoor, flat-floor environment with an expected area of approximately 5 m × 5 m; the final dimensions are TBD. It may contain rooms, corridors, walls, simple obstacles, rescue targets, and robot start zones.

The official layout and target positions will not be disclosed in advance. Practice environments may use similar structural elements but will not necessarily match the official arena. Teams must not obtain or use the official arena map or target coordinates before the run.

![Competition arena](Edge_AI_Robotics_Challenge_Rulebook_V0_assets/media/image2.png)

## 3.2 Rescue Targets

Rescue targets are represented by ArUco visual markers. Each ArUco marker has a unique ID identifying the corresponding rescue target. Targets are divided into two types: Normal Targets and Cooperative Targets. The ArUco dictionary, physical marker size, mounting method, and mapping from marker IDs to target types will be specified in the technical documentation.

| **Target Type**    | **Example ID** | **Rescue Requirement**                                                                                 |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------|
| Normal Target      | T1 / T2 / T3   | One robot reaches the rescue zone and remains there continuously.                                      |
| Cooperative Target | TC1            | Two different robots simultaneously reach their respective rescue zones and remain there continuously. |

## 3.3 Target Position and Rescue Position

Marker position is used for target identification and localization evaluation, whereas the rescue zone or rescue position is used for rescue-completion verification. Each ArUco marker corresponds to one or more predefined Rescue Zones. The Rescue Zone position is determined from the marker pose using a unified and publicly specified geometric rule. A Normal Target corresponds to one Rescue Zone; a Cooperative Target corresponds to two Rescue Zones.

A rescue is completed when the robot enters the corresponding Rescue Zone and remains there continuously for the required duration. For a Cooperative Target, two different robots must simultaneously occupy the two corresponding Rescue Zones and satisfy the continuous dwell requirement. Rescue completion is based primarily on robot position and does not require a specific final orientation.

# 4 Competition Flow

| **Stage**                     | **Main Content**                                                            |
|-------------------------------|-----------------------------------------------------------------------------|
| Preparation and Check         | Check robots, edge node, communication, software, and stop mechanisms.      |
| Task 1                        | Autonomous exploration, mapping, target detection, and target localization. |
| Task Handover                 | Save the map and target list and initialize the multi-robot rescue system.  |
| Task 2A                       | Execute normal and cooperative rescue tasks.                                |
| State Freeze and Verification | Keep robot positions fixed and verify Task 2A results.                      |
| Task 2B                       | Execute the LLM-Guided Mission.                                             |
| Final Result Confirmation     | Save required records and verify final results.                             |

Time limits for each task, preparation and handover duration, number of official runs, and the method for selecting the final score are TBD.

Tentative rule: Task 1 and Task 2 use the same arena layout and target positions. The organizers may reposition robots according to a uniform procedure during the handover from Task 1 to Task 2A. No repositioning is performed between Task 2A and Task 2B unless required for safety.

# 5 Task 1 - Autonomous Exploration, Mapping, and Target Identification

## 5.1 Objective and Initial Conditions

Teams may use up to two robots starting from a designated start area to autonomously explore an unknown environment, build a unified 2D occupancy grid map, and identify and localize ArUco rescue targets. At the end of the task, each team shall output one unified map and one unified target list. Software, models, marker dictionaries, and calibration parameters may be preloaded, but the official arena map and target coordinates may not be preloaded.

## 5.2 Autonomous Exploration and Mapping

The robot system shall operate autonomously based on the current map and sensor information, select subsequent exploration regions or exploration targets, and autonomously perform path planning, obstacle avoidance, and motion execution toward the selected target. During the timed run, operators may not guide exploration by teleoperation, repeated navigation-goal input, or intermediate waypoints.

Teams may use existing ROS 2 SLAM, navigation, and other open-source software packages. The organizers provide a single-robot SLAM baseline, while autonomous exploration strategy, navigation-system integration, and multi-robot exploration and coordination remain the responsibility of each team.

## 5.3 Target Identification and Localization

The system shall perform the following pipeline:

observe marker → identify target ID and type → estimate relative position → transform into the map frame → update the target list.

Repeated observations of the same target shall be merged into one unique entry and shall not be scored multiple times.

## 5.4 Required Outputs

A. A 2D occupancy grid map, together with the metadata needed to interpret the map, including resolution, origin, and coordinate-frame information.

B. A target list containing, at minimum, the target ID, target type, and the marker position and orientation in the map coordinate frame. Corresponding Rescue Zone(s) may then be derived using the official rescue-geometry rule.

| **Target ID** | **Type** | **x \[m\]** | **y \[m\]** | **yaw \[rad\]** |
|---------------|----------|-------------|-------------|-----------------|
| T1            | Normal   | 1.30        | 2.10        | 1.57            |

The numerical values above are format examples only and do not represent the official arena. Submission format, coordinate conventions, and logging interfaces will be defined in the Technical Tutorial.

## 5.5 End Conditions

Task 1 ends when the **time limit** is reached, the team declares completion, the team requests termination, or the referee stops the run for safety reasons. At the end of the task, the current valid map and target list shall be saved.

## 5.6 Evaluation Principles

Evaluation includes effective exploration and mapping performance, the number of correctly identified unique targets, and target localization accuracy. Map-alignment method, coverage measurement, localization-error thresholds, and false-detection handling are TBD.

Teams with incomplete Task 1 outputs may still proceed to Task 2. The organizers will not provide missing targets, target coordinates, or a completed reference map.

# 6 Task 2 - Multi-Robot Cooperative Rescue

## 6.1 Task Objective and Coordination

Task 2 uses the unified map and target list generated by the team in Task 1. Multiple robots cooperate to complete rescue tasks. Teams may use up to four robots and use the organizer-provided edge node for global state aggregation, task planning, and multi-robot coordination.

The system shall autonomously perform target selection, robot assignment, and task execution based on available target information and robot states. Teams may design their own task-allocation and coordination strategies, for example distance-based, heuristic, or optimization-based approaches, and are responsible for execution order, robot pairing, path-conflict handling, and reassignment after task failure.

During the time run, operators may not replace autonomous planning and coordination by manually selecting targets, assigning robots, or repeatedly providing navigation goals.

Task 2 primarily uses the team's Task 1 outputs; a reference map or complete target list is not provided by default. Robots may continue using real-time perception for localization, obstacle avoidance, and updates to the positions of already discovered targets.

Basic rescue scoring applies only to targets that were already included in the unified Target List at the end of Task 1. Targets newly discovered during Task 2 are therefore not eligible for basic rescue points.

## 6.2 Normal Target Rescue

Each Normal Target corresponds to a defined Rescue Zone and is rescued by one robot. A Normal Target is successfully rescued when one robot enters the corresponding Rescue Zone and continuously satisfies the specified position condition for the required dwell time.

If the robot leaves the valid zone or no longer satisfies the position condition before completion, the continuous dwell timer is reset. Timing restarts when the condition is satisfied again.

After a target has been successfully rescued, the participating robot is released from that task and may be autonomously assigned to another incomplete rescue task or remain on standby according to the team strategy. A robot is not required to return to the start area after completing an individual target.

## 6.3 Cooperative Target Rescue

Each Cooperative Target corresponds to two defined Rescue Zones. Cooperative rescue must be completed by two different robots, each entering one of the two corresponding Rescue Zones.

The cooperative dwell timer starts when both robots simultaneously satisfy the position condition for their respective Rescue Zones. If both robots continuously satisfy the condition for the required duration, the Cooperative Target is successfully rescued.

If either robot leaves its Rescue Zone or no longer satisfies the condition before completion, the cooperative dwell timer is reset. Timing restarts when both robots again simultaneously satisfy the conditions.

The same robot may not occupy both Rescue Zones. One robot may arrive first and wait for the other robot.

After a Cooperative Target is completed, both participating robots are released from that task and may be autonomously assigned to other incomplete tasks. They are not required to return to the start area.

The exact geometry of the two Rescue Zones, position tolerance, and cooperative dwell time will be published in the final technical parameters.

## 6.4 Rescue Records and Scoring Stage

The system shall record the target ID, target type, participating robot(s), task state, and completion time for referee verification.

Each target may receive basic rescue points at most once within the same scoring stage. Once successfully completed, its state shall be updated to completed, and repeatedly entering the same Rescue Zone does not earn additional basic rescue points.

Because a robot may continue to participate in subsequent tasks after completing its current target, the system shall support further task allocation based on robot and target states.

## 6.5 Task 2A Completion Principle

Task 2A is time-limited. Within the allotted time, each team shall autonomously complete as many valid rescue tasks as possible.

Task 2A ends when one of the following conditions occurs:

1.  The competition time limit is reached.

2.  All currently valid rescue targets have been successfully completed.

3.  The referee terminates the run due to safety, equipment failure, or another necessary reason.

At the end of Task 2A, all robots shall immediately stop autonomous motion and enter Safe Mode while remaining at their current physical positions. Robots are not required to return to the start area. Necessary localization, communication, state logging, and system-monitoring functions may continue to operate.

A result-verification period then begins. The organizers confirm Task 2A rescue results based on actual robot behavior, system records, and the competition rules. During verification, robots may not be moved, and teams may not manually modify the map, target positions, task states, or other information that could influence subsequent task execution.

After Task 2A results have been verified, the organizers will reposition the robots to predefined Task 2B starting positions. The map, target list, and other environment information obtained previously are retained. Task 2B then begins with this standardized physical configuration. This state becomes the initial condition for the subsequent LLM-Guided Mission.

## 6.6 LLM Processing Flow

The LLM-Guided Mission is conducted after the basic multi-robot rescue stage (Task 2A) has ended and its results have been verified. The existing map, localization state, target information are retained for the LLM-Guided Mission.

At the start of the LLM-Guided Mission, the organizers provide the system with a previously undisclosed natural-language mission instruction. The team system shall use a local (**or external cloud-based, TBD**) LLM deployed on the organizer-provided edge node to interpret the instruction and convert its mission objectives and constraints into a structured mission plan executable by the multi-robot coordination system.

Natural-language missions may contain high-level constraints such as target selection, task priority, execution order, prerequisites or dependencies, and parallel relationships. The system shall combine the mission instruction with the current target and robot states to generate an appropriate mission plan and then continue execution autonomously once the LLM-Guided Mission begins.

In Task 2B, targets specified by the LLM mission are treated as new mission actions. Even if a target already received basic rescue points in Task 2A, robots may be required to revisit its Rescue Zone. Such an action contributes only to the LLM mission score and does not earn additional basic rescue points.

**Natural-Language Instruction → LLM → Structured Mission Plan → Multi-Robot Coordinator → Robot Assignment & Coordination → Autonomous Navigation → Rescue Execution**

LLM inference shall run on the organizer-provided edge node. Allowed models, software environments, and compute-resource limits will be specified in the technical documentation.

## 6.7 LLM Responsibilities and Boundaries

In this task, the LLM acts as a high-level mission interpreter and planner. Its primary role is to understand the organizer-provided natural-language mission instruction and convert mission objectives and high-level constraints into a structured mission plan.

The LLM may parse information including, but not limited to:

- rescue targets to be executed;

- target or task priorities;

- execution order between tasks;

- prerequisites or dependency relationships;

- tasks that may be executed in parallel.

The LLM determines what needs to be done and which mission-level constraints must be satisfied; it does not directly determine how the robots perform low-level execution.

Specific robot selection and assignment, robot pairing for cooperative targets, path planning, obstacle avoidance, motion control, and rescue-action execution are performed autonomously by the multi-robot coordination system and the robots. The LLM does not directly perform low-level perception, navigation, or motion control and shall not directly send low-level motion-control commands to the robots.

For example, for the instruction “After T2 is completed, rescue T1 and T3,” the LLM should infer the precedence constraints T2 ≺ T1 and T2 ≺ T3. Which robot executes T1 and which robot executes T3 shall be decided autonomously by the multi-robot coordinator according to current robot positions, states, and availability.

For invalid targets, invalid task structures, or missions that the current system cannot execute, the system shall record the corresponding status and reason and remain in a safe state. During a timed run, operators may not manually correct the LLM parsing result or replace the system by manually assigning tasks. Whether abnormal or intentionally infeasible instructions are included in formal evaluation is TBD.

## 6.8 Example Mission and Interpretation

Example instruction: “First rescue T2. After T2 is completed, rescue T1 and T3. Finally, after both T1 and T3 are completed, use two robots to cooperatively rescue TC1.”

| **Stage** | **Prerequisite and Action**                                                     |
|-----------|---------------------------------------------------------------------------------|
| Stage 1   | Complete rescue of T2.                                                          |
| Stage 2   | After T2 is completed, rescue T1 and T3.                                        |
| Stage 3   | After both T1 and T3 are completed, use two robots to cooperatively rescue TC1. |

In this example, there is no precedence constraint between T1 and T3. The coordination system may therefore execute them sequentially or in parallel depending on the current robot states.

The LLM may encode the constraints using JSON, a task graph, a state machine, a behavior tree, or another structured representation. The Rulebook does not mandate a particular internal representation, but the generated mission plan must be executable and recordable by the downstream multi-robot coordination system. The supported instruction language, constraint types, and complexity range will be announced before the competition.

## 6.9 Records, Verifiability, and Completion Principle

The team system shall save the necessary runtime records for the LLM-Guided Mission, including:

- the original natural-language instruction provided by the organizers;

- the structured mission plan generated by the LLM, or an equivalent parsing result;

- task-execution state and key state transitions;

- actual robot task-execution results;

- the LLM model name and version used.

These records are used for post-run mission verification and scoring. The organizers do not require a uniform internal structured-mission schema, but each system shall provide sufficient information for the parsing result and actual execution process to be traceable.

# 7 General Rules

## 7.1 Autonomous Operation and Operator Behavior

During normal timed operation, operators may start the prescribed process, monitor system status, submit organizer-provided mission instructions through the prescribed interface, request a reset, and trigger an emergency stop.

Operators may not teleoperate robots, manually specify navigation points, alter robot task assignment, correct the map or target coordinates, edit LLM parsing results, or use direct on-site observations to inject unauthorized environmental information into the system.

## 7.2 Computing and Network

LLM inference shall be completed locally on the provided edge platform. Tentative rule: external cloud AI services are prohibited during official timed runs, and models and dependencies shall be prepared in advance. Use of auxiliary computing devices, external Internet access, and model-resource limits are TBD.

## 7.3 Reset and Human Intervention

A limited number of resets may be requested and will incur penalties. The number of resets, penalty values, reset positions, whether timing continues, and whether the map, target list, and completed rescues are retained are TBD. Resets must be approved and supervised by the referee and may not be used to manually complete missing information or directly complete rescue actions.

Emergency stop is always permitted for safety. Subsequent continuation, reset, or termination follows the unified competition procedure.

## 7.4 Safety and Fairness

Teams shall not intentionally collide with robots, move obstacles or target markers, or damage the arena. The system shall include an effective stop mechanism, and referees may stop clearly unsafe operation. Speed limits, hardware modifications, manipulator configuration, and collision penalties are TBD.

Teams shall not obtain undisclosed arena information or falsify detection, rescue, or inference records. Failures of organizer-provided equipment or infrastructure will be reviewed by the referee, who will determine whether recovery or a rerun is appropriate.

# 8 Scoring Framework

| **Category**                           | **Evaluation Content**                                                                    |
|----------------------------------------|-------------------------------------------------------------------------------------------|
| Task 1 - Exploration & Mapping         | Effective coverage, map completeness, and map usability.                                  |
| Task 1 - Identification & Localization | Number of correctly identified unique targets and position error.                         |
| Task 2 - Normal & Cooperative Rescue   | Rescue results that satisfy the corresponding conditions and are verified by the referee. |
| Task 2 - LLM Mission                   | Correct interpretation and actual execution of the mission and its constraints.           |
| Penalties                              | Resets, prohibited human intervention, and applicable safety violations.                  |

**Energy Efficiency (optional / bonus / tie-breaker)**  
Energy consumption may be considered as an additional system-efficiency metric. The final measurement method, normalization rule, and scoring weight will be announced before the competition.

The final scoring specification shall publish category weights, maximum scores, measurement thresholds, false-detection and partial-completion handling, treatment of order violations, whether time contributes to scoring, tie-break rules, penalties, and the method for selecting scores across multiple runs. Numerical values are not frozen in this version.

# 9 Milestones and Timeline

Milestones are used to confirm technical readiness and support equipment and venue-resource planning. Dates for all stages are TBD.

| **Stage**         | **Expected Assessment Content**                                                                                                                                                                                                                                                                                                                                                         |
|-------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Kick-off          | Confirm teams, obtain materials, and set up the development environment.                                                                                                                                                                                                                                                                                                                |
| Milestone         | **End-to-End Autonomous Rescue Baseline：**Demonstrate a functional end-to-end baseline including autonomous exploration and SLAM, ArUco target detection and localization, autonomous map-based navigation, and completion of at least one Normal Rescue. At least two robots should be connectable to the system with their states available for subsequent multi-robot coordination. |
| Final Competition | Complete both tasks, cooperative rescue, and local LLM instruction execution.                                                                                                                                                                                                                                                                                                           |

Assessment may use on-site or simulation demonstrations, logs, or videos. Requirements shall be published before each stage. If participation slots later become limited, the organizers shall announce technical thresholds, submission requirements, and allocation rules in advance and shall not introduce unpublished conditions after assessment.

# 10 Submission and Record Requirements

| **Material**           | **Minimum Content**                                                                  |
|------------------------|--------------------------------------------------------------------------------------|
| System description     | Architecture, division of responsibility between robot and edge node, and task flow. |
| Runnable software      | Dependencies, configuration, and startup instructions.                               |
| Task 1 outputs         | Map, metadata, and target list.                                                      |
| Task 2 records         | Task assignment, participating robots, target state, and completion time.            |
| LLM records            | Model name/version, original instruction, parsing result, and execution result.      |
| Brief technical report | Methods, results, known limitations, and third-party components.                     |

Submission format, deadlines, unified time reference, and whether source code must be submitted are TBD. Records shall be sufficient to support score verification.

# 11 Rule Publication and Interpretation

The organizers will publish rules, technical specifications, and Q&A through the website and email. Changes affecting task difficulty, platform restrictions, scoring, or advancement shall include a version identifier and effective date and shall be communicated to all teams.

Interpretations that apply generally shall be made public. The rule-freeze date, score-review period, required evidence, and handling procedure are TBD.

# Appendix A Parameters to Be Finalized Before Official Release

| **Item**                                                        | **Current Status**                       |
|-----------------------------------------------------------------|------------------------------------------|
| Competition date, venue, and contact channel                    | TBD                                      |
| Simulation and physical-robot arrangements for each stage       | TBD                                      |
| Arena dimensions                                                | Approximately 5 m × 5 m; to be confirmed |
| Maximum number of robots per team                               | 4 robots                                 |
| Whole-robot configuration and permitted hardware modifications  | To be verified and published             |
| Edge-node and model resource limits                             | TBD                                      |
| ArUco dictionary, marker size, quantity, and ID mapping         | TBD                                      |
| Rescue geometry, position tolerance, and dwell-time measurement | TBD                                      |
| Normal rescue dwell time                                        | TBD                                      |
| Cooperative rescue dwell time                                   | TBD                                      |
| Task and handover time limits                                   | TBD                                      |
| Reset count, timing, state retention, and penalties             | TBD                                      |
| Speed limits and manipulator competition configuration          | TBD                                      |
| Scoring weights, thresholds, and tie-break rules                | TBD                                      |
| Milestone dates, assessment, and advancement requirements       | TBD                                      |
| Number of official runs and score-selection method              | TBD                                      |
| Submission format, deadlines, rule freeze, and score review     | TBD                                      |

# Appendix B Robot Configuration and Technical Resources

The supporting technical resources are used to review hardware information and system-configuration guidance. Exact operating parameters follow the whole-robot configuration and Technical Tutorial published by the organizers.

## Supporting Document Roles

- Rulebook: competition tasks, rules, scoring, and milestone requirements.

- Kick-off Slides: help teams understand the overall objective, system architecture, and immediate next steps.

- Technical Tutorial: system installation, interfaces, startup, calibration, simulation, and development examples.
