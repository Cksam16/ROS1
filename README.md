# ROS1 Navigation Stack

This repository demonstrates an implementation of **autonomous robot navigation** using the ROS1 Navigation Stack in **ROS1 Melodic** on **Ubuntu 18.04**. It integrates localization and path planning with configuration and tuning of key navigation components: `gmapping`, `amcl` and `move_base`.

---

## Project Overview

This project simulates an indoor robot navigating a static environment using:
- **GMapping** to build a 2D occupancy grid map using SLAM
- **AMCL (Adaptive Monte Carlo Localization)** for probabilistic localization based on a 2D map
- **move_base** for global path planning and local obstacle avoidance using costmaps

---

## Folder Structure

```
ROS1/
├── gmapping/
│ └── gmapping.launch
├── AMCL/
│   ├── amcl.launch
│   └── amcl.yaml
├── move_base/
│   ├── move_base.launch
│   └── config/
│       ├── base_local_planner_params.yaml
│       ├── costmap_common_params.yaml
│       ├── global_costmap_params.yaml
│       └── local_costmap_params.yaml
└── README.md
```

---

## File-Level Features and Roles

### GMapping/

- **`gmapping.launch`**  
  Launches the `slam_gmapping` node for real-time SLAM (Simultaneous Localization and Mapping). It subscribes to laser scans and odometry to generate a 2D occupancy grid map of the environment. Useful in mapping unknown spaces before localization.

---

### AMCL/

- **`amcl.launch`**  
  Launches the AMCL node with parameters defined in `amcl.yaml`. This node uses a particle filter to localize the robot on a pre-built map.

- **`amcl.yaml`**  
  Contains configuration parameters for the AMCL node, including motion model parameters (`odom_alpha1`–`odom_alpha4`), laser model settings, minimum and maximum particles, and convergence criteria.

---

### move_base/

- **`move_base.launch`**  
  Launches the `move_base` node and loads configuration files required for global and local planning, costmaps, and planner behaviors.

#### move_base/config/

- **`base_local_planner_params.yaml`**  
  Defines parameters for the local planner (e.g., DWA or Trajectory Rollout), including maximum velocities, acceleration limits, and goal tolerances.

- **`costmap_common_params.yaml`**  
  Specifies shared parameters for both global and local costmaps, such as the robot footprint, inflation radius, and observation sources.

- **`global_costmap_params.yaml`**  
  Configuration specific to the global costmap, such as map resolution, update frequency, and static map layer settings.

- **`local_costmap_params.yaml`**  
  Settings for the local costmap, typically using a rolling window. Controls obstacle range, inflation behavior, and sensor sources used for dynamic obstacle avoidance.

---

## Requirements

- **Ubuntu 18.04**
- **ROS1 Melodic**
- RViz
- `map_server` + `.pgm` map file and `.yaml` config

---

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/YOUR_USERNAME/ROS1.git
cd ..
catkin_make
source devel/setup.bash
```

---

## Usage

1. **Launch GMapping to build a map**:

```bash
roslaunch gmapping gmapping.launch
```

2. **Launch AMCL for localization**:

```bash
roslaunch AMCL amcl.launch
```

3. **Launch move_base for path planning**:

```bash
roslaunch move_base move_base.launch
```

**Ensure the following:**
- Map server is running
- Robot is publishing `odom`, `scan`
- TFs correctly broadcast between `base_link`, `odom`, and `map`

---


