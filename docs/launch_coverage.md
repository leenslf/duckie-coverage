**Prerequisites:**
- Ensure `Fields2Cover` and `opennav_coverage` are built in the workspace.
- Docker environment must be running with GUI support for Gazebo/RViz.

### **How to Run the Navigation Stack**

This project uses **ROS 2 Humble** and **Gazebo** to perform complete coverage path planning. Follow the steps below in separate terminal windows.

#### **1. Launch the Simulation**

First, start the Gazebo environment. This loads the `gravel_featureless` world and spawns thQe robot at the defined starting corner. 

```bash
source install/setup.bash
ros2 launch moborobo_robot minimal_gazebo.launch.py

```

#### **2. Start Navigation2**

Once Gazebo is running, launch the Nav2 stack. This initializes the controller, planner, and the static transform bridge between the `map` and `odom` frames. 

```bash
source install/setup.bash
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true

```

#### **3. Visualization**

Open RViz to monitor the robot's local costmap and planned coverage path. 

```bash
source install/setup.bash
ros2 run rviz2 rviz2

```

#### **4. Execute Coverage Mission**

Run the demo script to send the polygon coordinates to the `coverage_server`. The robot will begin generating and following swaths. 

```bash
source install/setup.bash
ros2 run opennav_coverage_demo demo_coverage

```

---

### **System Configuration Notes**

| Component | Key Parameter | Description |
| --- | --- | --- |
| **Robot Physics** | `mu1: 0.0` | Caster wheels are frictionless to prevent diagonal drift during Northbound turns. |
| **Coverage** | `operation_width: 0.47` | Distance between swaths matches the physical width of the robot. |
| **Sync** | `static_map_odom_tf` | Synchronizes the Gazebo spawn offset $(2.5, -2.5)$ with the Nav2 map. |

---

### **Troubleshooting**

* **Process Died (Exit Code -6):** This usually indicates a YAML type error. Ensure all coordinates in `nav_params_sim.yaml` are written as doubles (e.g., `2.5` or `1.0` instead of `1`). 

* **Gazebo Port Error:** If Gazebo fails to launch, run `pkill -9 gzserver` to clear "zombie" processes.
