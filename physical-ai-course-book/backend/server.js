const express = require('express');
const cors = require('cors');
const session = require('express-session');
require('dotenv').config();

const app = express();
const PORT = process.env.PORT || 3003;

// Middleware
app.use(cors({
  origin: 'http://localhost:3000',
  credentials: true
}));
app.use(express.json());
app.use(session({
  secret: process.env.AUTH_SECRET || 'fallback-secret-key',
  resave: false,
  saveUninitialized: false,
  cookie: { 
    secure: false, // Set to true in production with HTTPS
    httpOnly: true,
    maxAge: 1000 * 60 * 60 * 24 * 7 // 1 week
  }
}));

// Simple in-memory user store (in production, use a database)
const users = {};

// Authentication routes
const bcrypt = require('bcrypt');

// Register endpoint
app.post('/api/auth/register', async (req, res) => {
  try {
    const { email, password, name } = req.body;
    
    // Check if user already exists
    if (users[email]) {
      return res.status(400).json({ message: 'User already exists' });
    }
    
    // Hash password
    const hashedPassword = await bcrypt.hash(password, 10);
    
    // Store user
    users[email] = {
      id: Date.now().toString(),
      email,
      name,
      password: hashedPassword
    };
    
    // Create session
    req.session.userId = users[email].id;
    req.session.user = {
      id: users[email].id,
      email: users[email].email,
      name: users[email].name
    };
    
    res.status(201).json({
      user: req.session.user
    });
  } catch (error) {
    console.error('Registration error:', error);
    res.status(500).json({ message: 'Internal server error' });
  }
});

// Login endpoint
app.post('/api/auth/login', async (req, res) => {
  try {
    const { email, password } = req.body;
    
    // Check if user exists
    const user = users[email];
    if (!user) {
      return res.status(400).json({ message: 'Invalid credentials' });
    }
    
    // Check password
    const validPassword = await bcrypt.compare(password, user.password);
    if (!validPassword) {
      return res.status(400).json({ message: 'Invalid credentials' });
    }
    
    // Create session
    req.session.userId = user.id;
    req.session.user = {
      id: user.id,
      email: user.email,
      name: user.name
    };
    
    res.json({
      user: req.session.user
    });
  } catch (error) {
    console.error('Login error:', error);
    res.status(500).json({ message: 'Internal server error' });
  }
});

// Session endpoint
app.get('/api/auth/session', (req, res) => {
  if (req.session.user) {
    res.json({
      user: req.session.user
    });
  } else {
    res.status(401).json({ message: 'Not authenticated' });
  }
});

// Logout endpoint
app.post('/api/auth/logout', (req, res) => {
  req.session.destroy(() => {
    res.json({ message: 'Logged out successfully' });
  });
});

// Mock chat endpoint - in a real implementation, this would connect to Gemini API and Qdrant
app.post('/api/chat', async (req, res) => {
  try {
    const { text } = req.body;
    
    // Simulate API processing delay
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    // Enhanced mock responses based on book content - in a real implementation, this would come from Gemini API with RAG
    const keywordResponses = {
      'physical ai': [
        "Physical AI refers to artificial intelligence systems that operate in the real world, interacting with physical environments through sensors, actuators, and mechanical bodies. Unlike digital AI, Physical AI embodies intelligence—it perceives, reasons, and acts in 3D space.Embodied Intelligence = Brain (AI) + Body (Robot) + Environment (World).",
        "The real world is messy with objects having weight, friction, and unpredictable behavior. Physical AI bridges this gap by learning from real sensory inputs, adapting to dynamic environments, and performing tasks like walking, grasping, and navigating.",
        "Applications include domestic helpers (Roomba, kitchen assistants), industrial robots (Amazon Kiva, autonomous forklifts), healthcare (surgical assistants, elderly care robots), and mobility (self-driving cars, delivery drones)."
      ],
      'robot': [
        "Robots are programmable machines that can execute tasks autonomously or semi-autonomously. They typically have sensors, processors, and actuators.",
        "Humanoid robots offer versatility (walk through doors, climb stairs), trust (familiar appearance reduces fear), and social compatibility (eye contact, gestures) which makes them ideal for domestic use.",
        "Industrial robots are used in manufacturing for tasks like welding, painting, and assembly."
      ],
      'ros2': [
        "ROS 2 (Robot Operating System 2) is an open-source framework for building robot applications. Think of ROS 2 as the nervous system of a robot. It provides communication middleware, hardware abstraction, and development tools.",
        "Core concepts include Nodes (independent processes), Topics (asynchronous message passing), Services (synchronous request-response), and Actions (long-running tasks with feedback).",
        "ROS 2 uses a distributed architecture with nodes communicating through topics, services, and actions. It's essential for building complex robotic systems."
      ],
      'simulation': [
        "Simulation is crucial in robotics because it ensures safety (test behaviors without risking hardware), speed (iterate faster than real-world trials), repeatability (controlled environments for debugging), and scalability (test multiple scenarios simultaneously).",
        "Think of simulation as a digital twin of the real world. It's essential for testing robotics algorithms safely.",
        "Popular simulation platforms include Gazebo and Unity 3D."
      ],
      'gazebo': [
        "Gazebo is an open-source robotics simulator that integrates seamlessly with ROS 2. It offers physics engines (ODE, Bullet), sensor simulation (LiDAR, Cameras, IMUs), 3D visualization, and plugin architecture for custom behaviors.",
        "Gazebo is often used with ROS for testing robot algorithms in a safe, simulated environment. It supports various robot models and can simulate complex environments with realistic physics.",
        "You can install Gazebo with ROS 2 using 'sudo apt install ros-humble-gazebo-ros-pkgs' on Ubuntu 22.04."
      ],
      'isaac': [
        "NVIDIA Isaac is a platform for autonomous machines that includes Isaac SDK for development and Isaac Sim for simulation.",
        "Isaac SDK provides optimized algorithms for perception, localization, and mapping.",
        "Isaac Sim is built on Omniverse and offers high-fidelity simulation for robotics."
      ],
      'vla': [
        "Vision-Language-Action (VLA) models combine computer vision, natural language processing, and robotic control.",
        "VLA models enable robots to understand instructions given in natural language and execute appropriate actions.",
        "These models represent the convergence of multimodal AI with robotics."
      ],
      'code': [
        "I can help explain code concepts related to Physical AI and robotics. Please share the specific code you'd like me to explain.",
        "For code explanation, please paste the code snippet you'd like me to analyze.",
        "I can explain code related to ROS2, robotics algorithms, or AI implementations. What code would you like me to review?"
      ]
    };
    
    // Select a response based on keywords in the question
    let selectedResponse = "I can help with topics related to Physical AI, robots, ROS2, simulation tools like Gazebo, NVIDIA Isaac, VLA models, and code explanation. Please ask a specific question about any of these topics!";
    
    // Convert text to lowercase for matching
    const lowerText = text.toLowerCase();
    
    // More flexible keyword matching
    const keywordMap = {
      'physical ai': ['physical ai', 'physicalai', 'embodied intelligence'],
      'robot': ['robot', 'robots', 'robotics', 'humanoid'],
      'ros2': ['ros2', 'ros 2', 'robot operating system 2', 'nodes', 'topics', 'services', 'actions'],
      'simulation': ['simulation', 'simulate', 'simulator', 'digital twin'],
      'gazebo': ['gazebo', 'gazebo simulator', 'physics engine'],
      'isaac': ['isaac', 'nvidia isaac', 'isaac sdk', 'isaac sim'],
      'vla': ['vla', 'vision-language-action', 'vision language action', 'multimodal ai'],
      'code': ['code', 'coding', 'program', 'explain code', 'code explanation']
    };
    
    // Check for keywords and select appropriate response
    for (const [primaryKeyword, variants] of Object.entries(keywordMap)) {
      // Check if any variant matches
      if (variants.some(variant => lowerText.includes(variant))) {
        const responses = keywordResponses[primaryKeyword];
        selectedResponse = responses[Math.floor(Math.random() * responses.length)];
        break;
      }
    }
    
    res.json({
      reply: `${selectedResponse} (This is a simulated response. In a full implementation, this would connect to the Gemini API and Qdrant database.)`
    });
  } catch (error) {
    console.error('Chat error:', error);
    res.status(500).json({ 
      reply: "Sorry, I encountered an error processing your request." 
    });
  }
});

// Health check endpoint
app.get('/api/health', (req, res) => {
  res.json({ status: 'OK', message: 'Chat API is running' });
});

app.listen(PORT, () => {
  console.log(`Chat API server running on port ${PORT}`);
});