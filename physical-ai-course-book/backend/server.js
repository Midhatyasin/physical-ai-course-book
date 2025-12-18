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
    
    // Improved mock responses based on keywords - in a real implementation, this would come from Gemini API
    const keywordResponses = {
      'physical ai': [
        "Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators. These systems can perceive their environment, make decisions, and act upon it.",
        "Physical AI combines machine learning with robotics to create systems that can learn from and interact with the physical world.",
        "Unlike traditional AI that operates in virtual environments, Physical AI systems must deal with real-world physics, uncertainty, and continuous sensor data."
      ],
      'robot': [
        "Robots are programmable machines that can execute tasks autonomously or semi-autonomously. They typically have sensors, processors, and actuators.",
        "Humanoid robots are designed to resemble the human form, which allows them to operate in environments built for humans.",
        "Industrial robots are used in manufacturing for tasks like welding, painting, and assembly."
      ],
      'ros2': [
        "ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications.",
        "ROS 2 provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.",
        "ROS 2 uses a distributed architecture with nodes communicating through topics, services, and actions."
      ],
      'simulation': [
        "Simulation is crucial for robotics development as it allows testing algorithms safely and cost-effectively.",
        "Gazebo is a popular robotics simulator that provides physics engines and realistic sensor models.",
        "Unity 3D is increasingly used for robotics simulation, especially for computer vision and AI training."
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
      ]
    };
    
    // Select a response based on keywords in the question
    let selectedResponse = "I'm not sure about that specific topic. Could you ask about Physical AI, robots, ROS2, simulation, Isaac, or VLA models?";
    
    // Convert text to lowercase for matching
    const lowerText = text.toLowerCase();
    
    // More flexible keyword matching
    const keywordMap = {
      'physical ai': ['physical ai', 'physicalai'],
      'robot': ['robot', 'robots', 'robotics'],
      'ros2': ['ros2', 'ros 2', 'robot operating system 2'],
      'simulation': ['simulation', 'simulate', 'simulator'],
      'isaac': ['isaac', 'nvidia isaac'],
      'vla': ['vla', 'vision-language-action', 'vision language action']
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