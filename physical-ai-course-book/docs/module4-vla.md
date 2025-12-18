# Chapter 5: Vision-Language-Action (VLA)

## 🎯 Learning Objectives
By the end of this chapter, you will:
- Understand the VLA paradigm in robotics
- Integrate LLMs (e.g., GPT, Gemini) with ROS 2
- Build voice-to-action pipelines
- Implement cognitive planning for task execution

---

## 5.1 What is Vision-Language-Action (VLA)?

**VLA** refers to systems where robots:
1. **See** (Vision: Cameras, LiDAR)
2. **Understand** (Language: Speech/NLP)
3. **Act** (Action: Manipulators, Navigation)

> Think of VLA as the **conversational brain** of a humanoid robot.

---

## 5.2 LLMs in Robotics

### 🧠 Why LLMs?
- **Generalization**: Handle unseen tasks via prompting
- **Planning**: Break down complex goals into steps
- **Interaction**: Enable natural dialogue with users

### 🧪 Example Prompt
```
User: "Clean the room."
LLM Plan:
1. Navigate to the vacuum cleaner
2. Pick up the vacuum
3. Move to each corner of the room
4. Return vacuum to dock
```

---

## 5.3 Voice-to-Action Pipeline

### 🎤 Components
1. **Speech Recognition** (Whisper API)
2. **Intent Parsing** (LLM)
3. **Action Mapping** (ROS 2 Nodes)

### 🧱 Architecture
```
[Microphone] 
    ↓
[Whisper ASR] → [LLM Planner] → [ROS 2 Executor]
    ↑
[Speaker] ← [Feedback Loop]
```

### 📄 Example Code: Whisper Integration
```python
import openai

def transcribe_audio(audio_file):
    with open(audio_file, "rb") as f:
        transcript = openai.Audio.transcribe("whisper-1", f)
    return transcript['text']
```

---

## 5.4 Cognitive Planning with LLMs

### 🧠 Planning Steps
1. **Goal Input**: "Bring me a bottle of water"
2. **Scene Understanding**: Detect objects (bottle, table, fridge)
3. **Path Planning**: Navigate to fridge
4. **Manipulation**: Grasp bottle
5. **Return**: Bring to user

### 🧪 Example LLM Prompt
```
Scene: Fridge at (2,3), Bottle at (2.5,3.2), User at (0,0)
Goal: Bring me a bottle of water
Plan:
1. Navigate to (2.5,3.2)
2. Grasp bottle
3. Navigate to (0,0)
4. Hand over bottle
```

---

## 5.5 Hands-On: VLA Demo

### 📁 Project Structure
```
/vla_demo
├── launch
│   └── vla.launch.py
├── src
│   ├── whisper_node.py
│   ├── planner_node.py
│   └── executor_node.py
└── config
    └── llm_prompt.txt
```

### 📄 whisper_node.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        # Simulate transcription
        msg = String()
        msg.data = "Bring me a bottle of water"
        self.publisher_.publish(msg)
        self.get_logger().info('Published voice command')

def main():
    rclpy.init()
    node = WhisperNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 📄 planner_node.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.plan_callback,
            10
        )

    def plan_callback(self, msg):
        command = msg.data
        plan = self.generate_plan(command)
        self.get_logger().info(f'Plan: {plan}')

    def generate_plan(self, command):
        # Simulate LLM call
        return "1. Navigate to bottle\n2. Grasp bottle\n3. Return to user"

def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5.6 Summary

| Component      | Role                          | Tech Used          |
|----------------|-------------------------------|--------------------|
| Vision         | Perceive environment          | Cameras, Isaac ROS |
| Language       | Understand user intent        | LLMs (Gemini/GPT)  |
| Action         | Execute physical tasks        | ROS 2, Manipulators|

---

## 🧪 Quick Quiz
1. What does VLA stand for?
2. How does an LLM help in task planning?
3. Name two components of a voice-to-action pipeline.

---

## 📚 Key Takeaways
- VLA bridges perception, language, and action
- LLMs enable flexible, general-purpose planning
- Voice interfaces make robots more accessible
- ROS 2 orchestrates the end-to-end pipeline