# Task Planning with PlanSys2: ArUco marker search & capture using ROS 2

**Experimental Robotics Laboratory — Planning Assignment (PlanSys2)**  
Authors: Gian Marco Balia, Christian Negri Ravera, Francesca Amato, Filippo Salterini, Arian Tavousi, Milad Rabiei

**Short description:**  
We control a mobile robot in Gazebo to search an environment until all ArUco markers are found and then visit and “capture” each marker in ascending ID order. The behavior is modeled in PDDL and executed using PlanSys2: a planner generates a valid sequence of actions (e.g., explore → change state → capture), and an executor runs it by activating our ROS 2 action nodes. Captures are produced by annotating the camera image and saving the final frames to disk, while also publishing the annotated image on a topic.

---

## 1) Main idea

The environment contains 4 ArUco markers placed somewhere in the world.  
To make the search systematic (and avoid spinning forever), we define 4 exploration waypoints that cover the map corners:

- `wp1 = (-6.0, -6.0)`
- `wp2 = (-6.0,  6.0)`
- `wp3 = ( 6.0, -6.0)`
- `wp4 = ( 6.0,  6.0)`

**High-level idea:**

1) **Search phase**  
   The robot navigates among the waypoints until it has detected all 4 markers at least once.

2) **Capture phase**  
   Once all markers are known, the robot goes to each marker in ascending marker ID, centers it, then publishes + saves the annotated image.

This two-phase structure is important because planning becomes clean:
- Phase 1 goal: “all markers found”
- Phase 2 goal: “all markers captured (in order)”

---

## 2) Why PlanSys2?

PlanSys2 splits the planning pipeline into components so instead of hard-coding a giant state machine, we:
- encode the logic in PDDL,
- let the planner generate a valid action sequence,
- implement each action as a ROS 2 node.

---

## 3) Repository structure

- `plansys_interface/`
  - `domain/`
    - `domain.pddl` (PDDL domain)
    - `problem.pddl` (PDDL problem)
  - `launch/`
    - `actions_launcher.launch.py` (PlanSys2)
  - `src/`
    - `move_to_photograph_action_node.cpp` (Capture action server)
    - `move_to_detect_action_node.cpp` (Explore action server)
    - `change_state_action_node.cpp` (State change)
---

## 4) PDDL model

### 4.1 Types & objects

- `robot` - single robot
- `marker` - waypoints
- `base` - startnpoint

### 4.2 Predicates

- `(robot_at ?r - robot ?m - marker)` — tracking robots position
- `(robot_not_at ?r - robot ?m - marker)` — tracking robots position
- `(detected ?m - marker)` — tracking visited ArUco marker's during detect mode
- `(undetected ?m - marker)` — tracking remaining ArUco marker's during detect mode
- `(photo_mode)` — state change after sucessful detections
- `(detect_mode)` — initial state of the robot
- `(is_after ?m_prev - marker ?m_post - marker)` — maintaining marker's orders (not used)
- `(photographed ?m - marker)` — tracking photographed ArUco marker's during photo mode
- `(unphotographed ?m - marker)` — final annotated “capture” completed during photo mode
- `(at_base ?r - robot ?b - base)` — position check for state change action
- `(is_base ?b - base ?m - marker)` — marker0 == base
- `(free ?r - robot)` — determining robots status

### 4.3 Actions

1) `move_to_detect (?r - robot ?m - marker ?m0 - marker ?b - base )`  
   Durative action. Start exploring the waypoint:
   - starts at base and goes to the given waypoint
   - changes marker's status to detected
   - stores the detected marker

2) `change_state (?r - robot ?m1 - marker ?m2 - marker ?m3 - marker ?m4 - marker ?m0 - marker ?b - base)`  
   Durative action. Changes robots state:
   - robot returns to base
   - changes operation mode to "photo_mode"

3) `move_to_photograph (?r - robot ?m - marker)`  
   Durative action. Produces the deliverable output:
   - moves towards determined waypoint
   - centers and takes a photo
   - publish annotated image on a topic
---

## 5) Nodes and runtime logic

### 5.1 PlanSys2 orchestration
- Domain Expert loads the PDDL domain (actions, predicates, types).
- Problem Expert holds the current instances + predicates and the goal.
- Planner computes a plan from the domain + current problem.
- Executor dispatches each action in the plan to its corresponding action performer node.

### 5.2 Action nodes
This project implements two PDDL actions, each backed by a ROS 2 performer node:

- **Move to detect action node**:
  - receives the dispatched action parameters,
  - navigates to the waypoint and performs the search behavior (rotate while waiting for detections),
  - updates the Problem Expert with newly discovered marker,
  - returns success/failure to the PlanSys2 executor after performing the scans.

- **Change state action node**:
  - receives the dispatched action parameters,
  - navigates to the base waypoint,
  - sorts all the ArUco marker's ids,
  - returns success/failure to the PlanSys2 executor.

- **Move to phorograph action node**:
  - receives the dispatched action parameters,
  - navigates to marker and then aligns to the marker,
  - “takes the picture” by annotating the camera frame, publishing it, and saving it to disk,
  - returns success/failure to the PlanSys2 executor.

---

## 6) Topics, TF, and outputs (update needed)
### 6.1 Subscribed topics
- `/camera/image/compressed`
- `/aruco_detections`
- `/tf`
### 6.2 Published topics
- `/Nav2`
- `/final_marker_image`
### 6.3 Frames
- `odom`
- `base_footprint`
- `camera_link`
- `marker_<id>`
### 6.4 Saved artifacts
- `output/marker_<id>.png`

---

## 7) Prerequisites (update needed)

- OS: Ubuntu (tested on Ubuntu 22 for ROS 2 Humble and Ubuntu 24 for ROS 2 Jazzy).
- ROS 2: Humble (works) and Jazzy (works)
- Gazebo Harmonic is used — confirm the exact Gazebo version matching the ROS 2 distro.
- Python: system Python3 (version used by ROS 2 distro; typically 3.10+).
- System tools: colcon, vcstool (python3-vcstool), development libraries for ROS2 packages.
- OpenCV + cv_bridge
- PlanSys2 binaries:
  - `ros-<distro>-plansys2-*`

---

## 8) Build & run (update needed)

### 8.1 Build workspace
```bash
mkdir -p ~/assignment_ws/src
cd ~/assignment_ws/src

# clone this repository
git clone https://github.com/ExpRobLab/assignment_planning.git

cd ~/assignment_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/local_setup.bash
```
### 8.2 Launch
**Terminal 1: Gazebo + Robot + Nav2 + Detection + PlanSys2**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 launch assignment1 assignment.launch.py world:=simple_world.sdf
```
**Terminal 2: Terminal client**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/assignment_ws/install/setup.bash
ros2 run plansys2_terminal plansys2_terminal
```
In the second terminal, run `get plan` in order to generate a plan and then run `run` for robot to start.

## 9) Results (update needed)
### 9.1 Output images

### 9.2 RQT graph / node graph

### 9.3 Video demo
