# Physical AI & Humanoid Robotics Course

This repository contains the complete "Teaching Physical AI & Humanoid Robotics Course" - a comprehensive educational resource on Physical AI and humanoid robotics.

## Course Overview

This capstone course introduces Physical AI—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.

### Key Topics Covered

1. **Robot Operating System 2 (ROS 2)** - The robotic nervous system for controlling humanoid robots
2. **Robot Simulation** - Using Gazebo and Unity to create digital twins of robots
3. **NVIDIA Isaac Platform** - Advanced perception and AI training for robots
4. **Vision-Language-Action (VLA)** - Integrating LLMs with robotic control systems
5. **Humanoid Robot Development** - Designing robots for natural human interactions
6. **Conversational Robotics** - Voice-to-action systems using OpenAI Whisper and GPT models

## Repository Structure

```
├── book/                   # Docusaurus-based book website
│   ├── docs/               # Course content in Markdown format
│   ├── src/                # React components and customizations
│   ├── docusaurus.config.js # Docusaurus configuration
│   └── package.json        # Node.js dependencies
├── chatbot/                # RAG chatbot implementation
│   ├── app/                # FastAPI application
│   ├── models/             # Chatbot models
│   ├── utils/              # Utility functions
│   ├── requirements.txt    # Python dependencies
│   └── Dockerfile          # Container configuration
└── README.md              # This file
```

## Features

### 🤖 AI-Enhanced Learning
- **Integrated RAG Chatbot**: Ask questions about any course content
- **Text Selection Support**: Get answers based on specific text selections
- **Personalized Responses**: Chatbot adapts to your background and preferences

### 🔐 User Authentication
- **Secure Signup/Login**: Using better-auth integration
- **User Profiles**: Store software and hardware background information
- **Progress Tracking**: Save your learning progress

### 🎯 Content Personalization
- **Adaptive Difficulty**: Adjust content complexity to your level
- **Learning Style Preferences**: Visual, hands-on, or theoretical focus
- **Content Depth Control**: Overview to comprehensive coverage

### 🌍 Language Support
- **Urdu Translation**: Enable Urdu translation for chapters
- **Persistent Preferences**: Translation settings saved per user

## Getting Started

### Prerequisites
- Node.js (v16 or higher)
- Python 3.9+
- Docker (for chatbot deployment)

### Running the Book Website Locally

1. Navigate to the book directory:
   ```bash
   cd book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

4. Open your browser to http://localhost:3000

### Running the Chatbot

1. Navigate to the chatbot directory:
   ```bash
   cd chatbot
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your actual credentials
   ```

4. Run the chatbot server:
   ```bash
   python -m app.main
   ```

### Deploying to GitHub Pages

1. Create a new repository on GitHub
2. Push this code to your repository
3. In your repository settings, enable GitHub Pages from the `gh-pages` branch

Alternatively, you can use the included GitHub Actions workflow which will automatically deploy to GitHub Pages on every push to the main branch.

## Course Content

The course is divided into four main modules:

### Module 1: The Robotic Nervous System (ROS 2)
Learn the fundamentals of ROS 2, the middleware that connects all robot components.

### Module 2: The Digital Twin (Gazebo & Unity)
Create realistic simulations of robots and environments for testing and validation.

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Implement advanced perception and training using NVIDIA's AI platform.

### Module 4: Vision-Language-Action (VLA)
Integrate large language models with robotic control for autonomous behavior.

## Hardware Requirements

This course requires specific hardware for the full experience:
- High-performance workstation with NVIDIA RTX GPU
- NVIDIA Jetson Orin Nano for edge computing
- Robot platform (Unitree Go2 Edu recommended for budget)

See the full hardware requirements in the course documentation.

## Contributing

We welcome contributions to improve this course! Please see our contributing guidelines for details.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- NVIDIA for their Isaac platform and educational resources
- Open Robotics for ROS 2 development
- OpenAI for language model APIs
- The broader robotics and AI community