# Kinova Gen3 Trajectory Planning and Execution

ROS Noetic implementation for planning, recording, and executing robotic arm trajectories using the Kinova Gen3 manipulator with MoveIt motion planning framework.

## Overview

This project demonstrates autonomous trajectory planning for the Kinova Gen3 7-DOF robotic arm. It allows you to define multiple arm poses, plan collision-free trajectories between them, save the planned motions, and execute them repeatedly in simulation.

### Key Features

- **MoveIt integration** for motion planning
- **5 distinct trajectories** showcasing different arm configurations
- **Trajectory persistence** using pickle serialization
- **Sequential execution** of pre-recorded motions
- **Gazebo simulation** with RViz visualization

## System Requirements

- **ROS Version:** ROS1 Noetic
- **Operating System:** Ubuntu 20.04 (or Docker with `osrf/ros:noetic-desktop-full`)
- **Simulation:** Gazebo + RViz
- **Robot:** Kinova Gen3 with Robotiq 2F-85 gripper

## Dependencies

Install required packages:

```bash
sudo apt-get install ros-noetic-moveit \
                     ros-noetic-ros-controllers \
                     ros-noetic-effort-controllers \
                     ros-noetic-position-controllers
```

Install Kinova ROS packages:
```bash
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/ros_kortex.git
```

Install Conan (for Kortex dependencies):
```bash
pip3 install conan==1.59.0
conan config set general.revisions_enabled=1
```

## Installation

1. Clone this repository:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ApurvK032/Kinova-Gen3-Trajectory-Planning.git hw_pkg
```

2. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Set environment variables:
```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
```

## Usage

### Step 1: Launch Gazebo Simulation

```bash
roslaunch kortex_gazebo spawn_kortex_robot.launch gripper:=robotiq_2f_85
```

Wait for Gazebo and RViz to fully initialize. You should see the Kinova Gen3 arm in both environments.

### Step 2: Collect Trajectories

In a new terminal:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun hw_pkg trajectory_collection.py __ns:=my_gen3
```

**What happens:**
- Arm moves to home position
- Plans and executes 5 different trajectories
- Saves all trajectories to `/home/catkin_ws/trajectories.pkl`

**Expected output:**
```
[INFO] Trajectory collector initialized successfully
[INFO] I1: Moving to home position...
[INFO] Planning T1: Home to Vertical...
[INFO] T1: Home to Vertical executed successfully
...
[INFO] SUCCESS! Saved 5 trajectories!
```

### Step 3: Execute Saved Trajectories

```bash
rosrun hw_pkg trajectory_execution.py __ns:=my_gen3
```

**What happens:**
- Loads trajectories from disk
- Moves arm to home position
- Executes all 5 trajectories in sequence

**Expected output:**
```
[INFO] Trajectory executor initialized successfully
[INFO] Loaded 5 trajectories
[INFO] Executing trajectory 1/5...
[INFO] Trajectory 1 executed successfully
...
[INFO] All trajectories executed!
```

## Trajectory Definitions

The system plans and executes 5 distinct motions:

| ID | Start Position | End Position | Description |
|----|---------------|--------------|-------------|
| T1 | Home | Vertical | Arm raises to vertical configuration |
| T2 | Vertical | Home | Return to home position |
| T3 | Home | Custom Pose 1 | Small joint adjustments (+0.3, +0.2 rad) |
| T4 | Custom Pose 1 | Custom Pose 2 | Incremental movement (+0.25, +0.2 rad) |
| T5 | Custom Pose 2 | Home | Return to starting position |

## Node Architecture

### trajectory_collection.py

**Functionality:**
- Initializes MoveIt commander for arm group
- Plans trajectories between defined poses
- Executes motions on physical simulation
- Serializes trajectory data to pickle file

**Configuration:**
- Velocity scaling: 0.4 (40% of max)
- Acceleration scaling: 0.3 (30% of max)
- Planning group: "arm"

### trajectory_execution.py

**Functionality:**
- Loads pre-recorded trajectories from file
- Ensures starting conditions match recording
- Executes trajectories sequentially
- Provides execution status feedback

**Safety:**
- Always starts from home position
- Waits for completion before next trajectory
- Stops arm after each motion

## Technical Details

**Robot Namespace:** `/my_gen3`  
**Planning Group:** `arm` (7 joints)  
**End Effector:** Robotiq 2F-85 Gripper  
**Storage Format:** Python pickle (RobotTrajectory objects)  
**Storage Location:** `/home/catkin_ws/trajectories.pkl`  

## Docker Setup (Optional)

For Ubuntu 24.04 or other systems:

```bash
xhost +local:docker
docker run -it \
  --name ros_kinova \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/catkin_ws:/home/catkin_ws \
  osrf/ros:noetic-desktop-full
```

Inside container:
```bash
apt-get update
apt-get install -y ros-noetic-moveit \
                   ros-noetic-ros-controllers \
                   gcc g++
pip3 install conan==1.59.0
conan config set general.revisions_enabled=1
```

## Project Structure

```
hw_pkg/
├── scripts/
│   ├── trajectory_collection.py   # Record trajectories
│   └── trajectory_execution.py    # Playback trajectories
├── CMakeLists.txt
└── package.xml
```

## Troubleshooting

**Issue:** MoveIt initialization fails  
**Solution:** Ensure Gazebo is fully loaded and robot namespace is correct

**Issue:** Trajectory planning fails  
**Solution:** Check for collisions in RViz, adjust joint targets

**Issue:** File not found error  
**Solution:** Run collection script before execution script

## Future Enhancements

- [ ] Cartesian path planning
- [ ] Gripper control integration
- [ ] Obstacle avoidance scenarios
- [ ] Real-time trajectory modification
- [ ] Multiple trajectory files support

## License

MIT License

## Author

Apurv Kushwaha
