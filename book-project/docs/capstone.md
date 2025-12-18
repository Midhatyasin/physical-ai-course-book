# Chapter 6: Capstone Project – Autonomous Humanoid

## 🎯 Learning Objectives
By the end of this chapter, you will:
- Design an end-to-end humanoid task pipeline
- Integrate VLA, ROS 2, and Isaac Sim
- Evaluate performance using a judging rubric
- Prepare a demo-ready submission

---

## 6.1 Capstone Overview

### 🎯 Task
**Goal**: *"Clean the room and bring me a bottle of water."*

### 🧱 Components
1. **Voice Command** (Whisper)
2. **Scene Perception** (Isaac ROS + Cameras)
3. **Navigation** (Nav2 + VSLAM)
4. **Manipulation** (Pick-and-place arm)
5. **Feedback** (ROS 2 Topics)

---

## 6.2 System Architecture

### 🧠 Block Diagram
```
[User Voice] 
    ↓
[Whisper ASR] → [LLM Planner] → [Task Executor]
    ↑                ↓              ↓
[Feedback]      [Nav2]       [Manipulator]
                   ↘            ↙
                  [Isaac Sim World]
```

### 🧩 Key Nodes
| Node            | Function                         | Package Used       |
|-----------------|----------------------------------|--------------------|
| `whisper_node`  | Speech-to-text                   | OpenAI SDK         |
| `planner_node`  | Decompose task into steps        | LLM (Gemini)       |
| `nav_node`      | Path planning + execution        | Nav2               |
| `arm_node`      | Grasp/drop objects               | MoveIt             |
| `sim_node`      | Isaac Sim environment sync       | Isaac ROS          |

---

## 6.3 Step-by-Step Execution

### 🧠 1. Voice Input
- **Input**: `"Clean the room and bring me a bottle of water."`
- **Output**: Transcribed text

### 🧠 2. LLM Planning
- **Prompt**:
  ```
  Scene: Vacuum at (1,2), Bottle at (3,4), User at (0,0)
  Goal: Clean room + bring water
  Plan:
  1. Navigate to (1,2)
  2. Activate vacuum
  3. Clean room
  4. Navigate to (3,4)
  5. Grasp bottle
  6. Navigate to (0,0)
  7. Hand over bottle
  ```

### 🧠 3. Task Execution
- **ROS 2 Launch**:
  ```bash
  ros2 launch capstone_project execute.launch.py
  ```

---

## 6.4 Evaluation Rubric

### 🧪 Judges Will Score On:
| Criteria              | Weight (%) | Description                              |
|-----------------------|------------|------------------------------------------|
| Task Completion       | 30%        | Did the robot finish the job?            |
| Planning Accuracy     | 20%        | Were steps logically ordered?            |
| Perception Quality    | 20%        | Did it detect objects correctly?         |
| Code Modularity       | 15%        | Is the system well-structured?           |
| Innovation            | 15%        | Bonus for creative enhancements          |

---

## 6.5 Submission Checklist

### 📦 Deliverables
- [ ] Source code (ROS 2 packages)
- [ ] Simulation world (Isaac Sim `.usd`)
- [ ] README with setup/run instructions
- [ ] Video demo (3 mins max)
- [ ] Report (PDF, <5 pages)

### 🧠 Tips
- Use **comments** in code
- Include **error handling**
- Add **unit tests** for key nodes

---

## 🧪 Quick Quiz
1. What is the final output of the LLM planner?
2. Which ROS 2 package handles navigation?
3. Name two evaluation criteria for judges.

---

## 📚 Key Takeaways
- Capstone integrates all prior modules
- Real-world tasks require multi-system coordination
- Judges value clarity, modularity, and innovation
- Demo readiness is key to success