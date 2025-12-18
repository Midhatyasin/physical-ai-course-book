const express = require('express');
const cors = require('cors');
require('dotenv').config();

// Initialize Better Auth
const { betterAuth } = require('better-auth');
const authConfig = require('./auth.config');

const auth = betterAuth(authConfig);

const app = express();
const PORT = process.env.PORT || 3003;

// Middleware
app.use(cors());
app.use(express.json());

// Better Auth routes
app.use('/api/auth', auth.handler);

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