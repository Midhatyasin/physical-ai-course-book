# Chapter 3: Robot Simulation with Gazebo

## 🎯 Learning Objectives
By the end of this chapter, you will:
- Understand the role of simulation in robotics
- Set up Gazebo with ROS 2 integration
- Model robots using URDF/SDF
- Simulate physics (gravity, collisions, sensors)

---

## 3.1 Why Simulation?

Simulation is **crucial** in robotics because:
- **Safety**: Test behaviors without risking hardware
- **Speed**: Iterate faster than real-world trials
- **Repeatability**: Controlled environments for debugging
- **Scalability**: Test multiple scenarios simultaneously

> Think of simulation as a **digital twin** of the real world.

---

## 3.2 What is Gazebo?

**Gazebo** is an open-source robotics simulator that integrates seamlessly with ROS 2. It offers:
- **Physics Engine** (ODE, Bullet)
- **Sensor Simulation** (LiDAR, Cameras, IMUs)
- **3D Visualization**
- **Plugin Architecture** for custom behaviors

---

## 3.3 Setting Up Gazebo with ROS 2

### 🛠️ Installation (Ubuntu 22.04)
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
```

### ▶️ Launching Gazebo
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

## 3.4 Modeling Robots: URDF vs SDF

### 🧱 URDF (Unified Robot Description Format)
- **XML-based**
- Used for **kinematic** and **visual** representation
- Integrated with ROS 2 tools (RViz, MoveIt)

#### 📄 Example URDF Snippet
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### 🧱 SDF (Simulation Description Format)
- **More detailed** than URDF
- Handles **dynamics**, **materials**, **plugins**
- Native to Gazebo

#### 📄 Example SDF Snippet
```xml
<sdf version='1.6'>
  <model name='my_robot'>
    <link name='base_link'>
      <collision>
        <geometry>
          <box>
            <size>0.5 0.3 0.1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

---

## 3.5 Simulating Physics

### 🌍 Gravity
- Enabled by default (`gravity="true"`)
- Modify in SDF:
  ```xml
  <gravity>0 0 -9.8</gravity>
  ```

### 🧱 Collisions
- Defined in `<collision>` tags
- Supports shapes: box, sphere, cylinder, mesh

### 🧲 Friction & Damping
- Tune realism:
  ```xml
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
  ```

---

## 3.6 Sensor Simulation

### 📸 Camera
```xml
<sensor name="camera" type="camera">
  <camera>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
</sensor>
```

### 📡 LiDAR
```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
      </horizontal>
    </scan>
  </ray>
</sensor>
```

### 🧭 IMU
```xml
<sensor name="imu" type="imu">
  <imu>
    <noise>
      <type>gaussian</type>
    </noise>
  </imu>
</sensor>
```

---

## 3.7 Hands-On: Spawn a Robot in Gazebo

### 📁 Project Structure
```
/my_gazebo_sim
├── my_robot.urdf
└── launch
    └── spawn_robot.launch.py
```

### 📄 spawn_robot.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', 'my_robot.urdf'],
            output='screen'
        )
    ])
```

### ▶️ Run Simulation
```bash
ros2 launch my_gazebo_sim spawn_robot.launch.py
```

---

## 3.8 Summary

| Component | Purpose                          | Format Used |
|-----------|----------------------------------|-------------|
| URDF      | Kinematic/Visual modeling        | XML         |
| SDF       | Full physics/sensor simulation   | XML         |
| Plugins   | Extend Gazebo functionality      | C++/Python  |
| Sensors   | Simulate real-world inputs       | SDF Tags    |

---

## 🧪 Quick Quiz
1. What are two benefits of simulation in robotics?
2. What is the difference between URDF and SDF?
3. How do you spawn a robot in Gazebo using ROS 2?

---

## 📚 Key Takeaways
- Simulation accelerates development and testing
- Gazebo integrates deeply with ROS 2
- URDF/SDF bridge digital models with physics
- Sensors and plugins bring realism to simulations