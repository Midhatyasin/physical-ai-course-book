# Final Chapter: Deployment & Hardware

## 🎯 Learning Objectives
By the end of this chapter, you will:
- Set up NVIDIA Jetson Orin for edge AI
- Integrate with Unitree robots
- Deploy cloud-native robotics apps
- Optimize for cost and performance

---

## 7.1 Edge Computing with Jetson Orin

### 🧠 Why Jetson?
- **ARM + CUDA**: Low power, high performance
- **ROS 2 Support**: Native compatibility
- **Form Factor**: Compact, ruggedized

### 🛠️ Setup Steps
1. Flash **JetPack 5.1+** (Ubuntu 20.04 LTS)
2. Install ROS 2 Humble:
   ```bash
   sudo apt install ros-humble-desktop
   ```
3. Clone your capstone repo:
   ```bash
   git clone <your-repo>
   colcon build
   ```

### 🧪 Run Nodes on Jetson
```bash
source install/setup.bash
ros2 run capstone whisper_node
```

---

## 7.2 Unitree Robot Integration

### 🦴 Supported Models
- **Go2 Edu** (Quadruped)
- **G1** (Humanoid)

### 🧱 Integration Steps
1. **SDK Setup**:
   ```bash
   git clone https://github.com/unitreerobotics/unitree_ros2
   colcon build
   ```
2. **Launch Controller**:
   ```bash
   ros2 launch unitree_ros2 ctrl.launch.py
   ```

### 🧠 Tips
- Use **UDP** for low-latency commands
- Calibrate **IMU** for balance
- Test **gait patterns** in simulation first

---

## 7.3 Cloud Deployment Options

### ☁️ Scenarios
| Use Case              | Provider     | Cost Model       |
|-----------------------|--------------|------------------|
| Heavy Simulation      | AWS EC2 G5   | $1.50/hr         |
| Model Training        | Google Colab | Free/Paid        |
| Remote Monitoring     | Render       | Free Tier        |

### 🧪 Example: AWS EC2 Setup
1. Launch **g5.2xlarge** (A10G GPU)
2. Install Docker:
   ```bash
   sudo apt install docker.io
   ```
3. Run Isaac Sim Container:
   ```bash
   docker run --gpus all isaac-sim
   ```

---

## 7.4 Optimization Strategies

### 💰 Cost vs Performance
| Strategy              | Benefit                        |
|-----------------------|--------------------------------|
| Sim-to-Real Transfer  | Reduce cloud compute time      |
| Local Caching         | Minimize API calls             |
| Modular Design        | Reuse components               |

### 🧠 Power Management
- Use **Jetson Clocks**:
  ```bash
  sudo jetson_clocks
  ```
- Monitor temps:
  ```bash
  tegrastats
  ```

---

## 7.5 Final Checklist

### 🧪 Deployment Readiness
- [ ] Jetson flashed + ROS 2 installed
- [ ] Unitree SDK configured
- [ ] Capstone nodes tested
- [ ] Cloud backup plan ready
- [ ] README with troubleshooting

---

## 🧪 Quick Quiz
1. What is the main advantage of Jetson Orin?
2. Name two Unitree robot models.
3. How can you reduce cloud costs?

---

## 📚 Key Takeaways
- Jetson enables portable AI inference
- Unitree provides affordable humanoid platforms
- Cloud complements edge for heavy tasks
- Optimization balances cost/performance