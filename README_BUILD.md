# Lab 8 – Maze Race: Build & Run Instructions

## Repository Structure

```
lab8/
├── lab8_interfaces/          # Custom ROS2 action definitions
│   ├── action/
│   │   └── ExploreMaze.action
│   ├── CMakeLists.txt
│   └── package.xml
├── maze_explorer/            # Main Python package
│   ├── launch/
│   │   └── lab8_launch.py
│   ├── maze_explorer/
│   │   ├── __init__.py
│   │   ├── wall_follower_action_server.py   # STATE 1: Explore + ArUco detect
│   │   └── orchestrator.py                 # State machine
│   ├── resource/
│   │   └── maze_explorer
│   ├── package.xml
│   └── setup.py
└── REPORT.md                 # Written deliverables
```

## Prerequisites

- ROS2 Humble
- TurtleBot 4 with Nav2, SLAM Toolbox, AMCL running
- Python packages: `opencv-contrib-python`, `cv_bridge`, `numpy`
- `irobot_create_msgs` installed

## Build

```bash
# Copy both packages into your ROS2 workspace src folder
cp -r lab8_interfaces ~/ros2_ws/src/
cp -r maze_explorer   ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select lab8_interfaces
source install/setup.bash

colcon build --packages-select maze_explorer
source install/setup.bash
```

## Run (single terminal – competition mode)

```bash
ros2 launch maze_explorer lab8_launch.py
```

## Parameters to tune for speed

| Parameter | File | Effect |
|-----------|------|--------|
| `forward_speed` | `wall_follower_action_server.py` | Exploration speed |
| `desired_wall_dist` | `wall_follower_action_server.py` | How close to hug the wall |
| `front_clear_dist` | `wall_follower_action_server.py` | When to start turning |
| `max_vel_x` | Nav2 params | Navigation speed |
| `marker_size` | `wall_follower_action_server.py` | ArUco physical size (meters) |

## Key Design Decisions

1. **Single launch file** – both nodes launched from `lab8_launch.py`, satisfying the "one terminal" rule.
2. **ReentrantCallbackGroup** – allows sensor callbacks to fire during action execution.
3. **State machine in orchestrator** – clean separation of exploration, docking, and navigation states.
4. **TF2 integration (recommended)** – use `tf2_ros.Buffer` to record robot pose at ArUco detection for accurate map-frame coordinates.
