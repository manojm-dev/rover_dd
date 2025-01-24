# Questbot Description 


## Table of Contents

1. [Overview](#overview)  
2. [Launch Files](#launch-files)  
3. [Usage](#usage)  
4. [Customization](#customization)  
5. [File Structure](#file-structure)  

---

## Overview
The Questbot Description package provides a set of Xacro and URDF files to define the robot's configuration, along with launch files for visualization and simulation. Key features include:

- Support for 2WD and 6WD configurations.  
- RViz and RQt integration for data visualization.  
- Gazebo support for simulation with clock synchronization.  

## Launch Files

### 1. **`rsp.launch.py`**
Starts the robot state publisher with the appropriate URDF model and simulation configurations.  

#### Arguments:
- `use_sim_time`: Use simulation (Gazebo) clock. Default: `true`.  
- `use_gazebo`: Use Gazebo classic. Default: `true`.  
- `use_gzsim`: Use Gazebo Sim. Default: `false`.  

```bash
ros2 launch questbot_description rsp.launch.py use_sim_time:=true use_gazebo:=true use_gzsim:=false
```

### 2. **`visualize.launch.py`**
Launches RViz and RQt visualization tools with preset configurations.  

#### Arguments:
- `use_rviz`: Enable RViz visualization. Default: `true`.  
- `rviz_config`: Path to the RViz configuration file.  
- `use_rqt`: Enable RQt visualization. Default: `false`.  
- `rqt_perspective`: Path to the RQt perspective file.  

```bash
ros2 launch questbot_description visualize.launch.py use_rviz:=true use_rqt:=true
```

### 3. **`display.launch.py`**
Launches the robot's state publisher, joint state publisher, RViz, and RQt with configurable parameters.  

#### Arguments:
- `use_jsp`: Enable joint state publisher. Default: `true`.  
- `jsp_gui`: Enable joint state publisher GUI. Default: `false`.  

```bash
ros2 launch questbot_description display.launch.py use_jsp:=true jsp_gui:=false
```



---

## File structure
```
questbot_description
├── CMakeLists.txt
├── LICENSE
├── launch
│   ├── display.launch.py
│   ├── rsp.launch.py
│   └── visualize.launch.py
├── package.xml
├── rqt
│   └── display.perspective
├── rviz
│   └── display.rviz
└── urdf
    ├── 2w_diffdrive
    │   ├── body.urdf.xacro
    │   ├── control
    │   │   ├── gazebo_control.xacro
    │   │   └── gzsim_control.xacro
    │   ├── gazebo.urdf.xacro
    │   ├── macro
    │   │   ├── inertial.xacro
    │   │   └── wheel.xacro
    │   ├── robot.urdf.xacro
    │   └── sensors
    │       ├── gazebo_camera.xacro
    │       ├── gazebo_depth_camera.xacro
    │       ├── gazebo_imu.xacro
    │       ├── gazebo_lidar.xacro
    │       ├── gzsim_imu.xacro
    │       └── gzsim_lidar.xacro
    └── 6w_diffdrive
        ├── body.urdf.xacro
        ├── control
        │   ├── gazebo_control.xacro
        │   └── gzsim_control.xacro
        ├── gazebo.urdf.xacro
        ├── macro
        │   ├── inertial.xacro
        │   └── wheel.xacro
        ├── robot.urdf.xacro
        └── sensors
            ├── gazebo_camera.xacro
            ├── gazebo_depth_camera.xacro
            ├── gazebo_imu.xacro
            ├── gazebo_lidar.xacro
            ├── gzsim_imu.xacro
            └── gzsim_lidar.xacro

```
