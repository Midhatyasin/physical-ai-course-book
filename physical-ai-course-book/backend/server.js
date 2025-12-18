const express = require('express');
const cors = require('cors');
const session = require('express-session');
require('dotenv').config({ path: __dirname + '/../.env' });

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

// RAG-enabled chat endpoint - connects to Gemini API and book content
app.post('/api/chat', async (req, res) => {
  try {
    const { text } = req.body;
    
    // Simulate API processing delay
    await new Promise(resolve => setTimeout(resolve, 1000));
    
    // Check if we have processed embeddings
    const fs = require('fs');
    const path = require('path');
    const embeddingsPath = path.join(__dirname, 'embeddings.json');
    
    if (fs.existsSync(embeddingsPath)) {
      // Use RAG system with actual book content
      const { GoogleGenerativeAI } = require('@google/generative-ai');
      const { findRelevantDocuments } = require('./vector-search');
      
      // Initialize Gemini API
      const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
      const model = genAI.getGenerativeModel({ model: "gemini-pro"});
      
      // Create embedding for the query
      const embeddingModel = genAI.getGenerativeModel({ model: "embedding-001" });
      const embeddingResult = await embeddingModel.embedContent(text);
      const queryEmbedding = embeddingResult.embedding.values;
      
      // Find relevant documents
      const relevantDocs = await findRelevantDocuments(queryEmbedding, 3);
      
      if (relevantDocs.length > 0) {
        // Create context from relevant documents
        const context = relevantDocs.map(doc => `From ${doc.source}: ${doc.content}`).join('\n\n');
        
        // Generate response using Gemini with context
        const prompt = `You are a helpful assistant explaining concepts from a Physical AI and Humanoid Robotics textbook. 
        Use the following context to answer the question accurately:

${context}

Question: ${text}

Answer:`;
        
        const result = await model.generateContent(prompt);
        const response = await result.response;
        const reply = response.text();
        
        res.json({ reply });
      } else {
        // Fallback to keyword-based responses if no relevant documents found
        res.json({ reply: "I couldn't find specific information about that in the textbook. Please ask about Physical AI, robots, ROS2, simulation, or other topics covered in the course." });
      }
    } else {
      // Fallback to mock responses if embeddings haven't been generated
      res.json({ reply: "The RAG system hasn't been initialized yet. Please run 'npm run process-content' to process the book content first. In the meantime, I can answer general questions about Physical AI, robotics, and related topics." });
    }
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