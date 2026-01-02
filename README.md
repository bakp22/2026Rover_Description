# 2026 Rover — Robot Description Package

This repository contains the **robot description** for the *2026 Boston University Mars Rover* — a ROS2-compatible model including its URDF/XACRO files, meshes, launch configurations, and RViz visualizations.

> ⚙️ This package provides the core assets needed to visualize, simulate, and integrate the 2026 Rover within ROS 2 workflows.

---

## 📁 Repository Structure

```text
2026Rover_Description/
├── build/              # ROS 2 build output
├── install/            # Workspace install artifacts
├── launch/             # Launch files for robot description & visualization
├── log/                # ROS logging output
├── meshes/             # 3D model files (STL / DAE / OBJ)
├── rviz/               # RViz configuration files
├── urdf/               # URDF and XACRO robot description files
├── CMakeLists.txt      # ROS build instructions
├── package.xml         # ROS package metadata
├── test.xacro          # Example XACRO file
└── LICENSE             # MIT License
```

---

## 🚀 Features

This package enables:

- **Robot visualization** in RViz  
- **URDF/XACRO-based robot modeling** with proper link and joint definitions  
- **Launch files** for quick startup  
- Easy integration as a dependency for **simulation, planning, and control** packages

---

## 📌 Prerequisites

- ROS 2 (Humble, Iron, or later)
- A configured ROS 2 workspace
- `colcon` build system

---

## 🛠️ Installation

### 1️⃣ Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/bakp22/2026Rover_Description.git
```

### 2️⃣ Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 3️⃣ Source the workspace

```bash
source install/setup.bash
```

---

## ▶️ Visualization in RViz

Launch the robot description:

```bash
ros2 launch 2026Rover_Description display.launch.py
```

> Replace `display.launch.py` if your launch file has a different name.

---

## 📄 License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.
