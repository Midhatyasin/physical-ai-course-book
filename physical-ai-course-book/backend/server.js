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
    
    // Mock response - in a real implementation, this would come from Gemini API
    const mockResponses = [
      "I understand you're asking about Physical AI. Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators.",
      "That's an interesting question about robotics. Humanoid robots are designed to resemble the human form and can perform tasks in human environments.",
      "ROS 2 is a great framework for robotics development. It provides tools and libraries for building robot applications.",
      "Simulation is crucial for testing robotics algorithms safely. Gazebo and Unity are popular simulation platforms.",
      "NVIDIA Isaac is powerful for AI-powered perception in robotics applications."
    ];
    
    // Select a random response for demonstration
    const randomResponse = mockResponses[Math.floor(Math.random() * mockResponses.length)];
    
    res.json({
      reply: `${randomResponse} (This is a simulated response. In a full implementation, this would connect to the Gemini API and Qdrant database.)`
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