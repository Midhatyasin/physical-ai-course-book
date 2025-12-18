# Chapter 2: ROS 2 Fundamentals

## 🎯 Learning Objectives
By the end of this chapter, you will:
- Understand the role of ROS 2 in robotics
- Learn core concepts: Nodes, Topics, Services, Actions
- Build a simple publisher-subscriber system
- Explore ROS 2 tools (CLI, RViz, rqt)

---

## 2.1 What is ROS 2?

**ROS 2 (Robot Operating System 2)** is an open-source framework for building robot applications. It provides:
- **Communication middleware** (Nodes exchange data)
- **Hardware abstraction** (Drivers for sensors/actuators)
- **Development tools** (Visualization, debugging)

> Think of ROS 2 as the **nervous system** of a robot.

---

## 2.2 Core Concepts

### 🧩 Nodes
- **Definition**: Independent processes that perform computation
- **Example**: A camera node captures images; a navigation node plans paths

### 🔗 Topics
- **Definition**: Channels for asynchronous message passing
- **Pattern**: Publisher → Topic → Subscriber
- **Use Case**: Sensor data streaming

### 📞 Services
- **Definition**: Synchronous request-response communication
- **Example**: Request map data from a server

### 🔄 Actions
- **Definition**: Long-running tasks with feedback
- **Components**: Goal, Feedback, Result
- **Use Case**: Move a robot arm to a position

---

## 2.3 Hello World: Publisher-Subscriber

Let’s create a simple ROS 2 package with Python:

### 📁 Project Structure
```
/my_robot_pkg
├── package.xml
├── setup.py
└── /my_robot_pkg
    ├── publisher.py
    └── subscriber.py
```

### 📄 publisher.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 📄 subscriber.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ▶️ Running the Nodes
1. **Terminal 1**:
   ```bash
   ros2 run my_robot_pkg publisher
   ```
2. **Terminal 2**:
   ```bash
   ros2 run my_robot_pkg subscriber
   ```

---

## 2.4 ROS 2 Tools

### 🧰 CLI Commands
- `ros2 node list` – See active nodes
- `ros2 topic list` – List all topics
- `ros2 topic echo /topic_name` – View messages
- `ros2 run package_name executable_name` – Run a node

### 🖼️ RViz
- **Purpose**: 3D visualization of robot data
- **Use Case**: View sensor data, planned paths

### 🧪 rqt
- **Purpose**: GUI for debugging/ros graphs
- **Plugins**: Plotter, Logger, Graph View

---

## 2.5 Summary

| Concept     | Purpose                          | Example Use Case         |
|-------------|----------------------------------|--------------------------|
| Nodes       | Independent computation units    | Camera driver            |
| Topics      | Asynchronous messaging           | Sensor data streaming    |
| Services    | Synchronous request-response     | Get map data             |
| Actions     | Long-running tasks w/ feedback   | Move robot arm           |

---

## 🧪 Quick Quiz
1. What is the difference between a Topic and a Service?
2. How do you run a ROS 2 node from the terminal?
3. Name two ROS 2 visualization/debugging tools.

---

## 📚 Key Takeaways
- ROS 2 is the backbone of modern robotics
- Nodes communicate via Topics, Services, and Actions
- CLI tools help debug and monitor systems
- Hands-on practice builds intuition