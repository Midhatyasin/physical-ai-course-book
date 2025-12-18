// Debug script for testing embedding creation

const { GoogleGenerativeAI } = require('@google/generative-ai');
require('dotenv').config({ path: __dirname + '/../.env' });

async function testEmbedding() {
  try {
    console.log('Testing embedding creation...');
    
    // Check if API key is available
    if (!process.env.GEMINI_API_KEY) {
      console.error('GEMINI_API_KEY is not set');
      return;
    }
    
    console.log('API Key found, initializing Gemini...');
    
    // Initialize Gemini API
    const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);
    
    // Test with a simple text
    const testText = "What is Physical AI?";
    
    console.log('Creating embedding for:', testText);
    
    // Get embedding model
    const embeddingModel = genAI.getGenerativeModel({ model: "embedding-001" });
    
    // Create embedding
    console.log('Calling embedContent...');
    const result = await embeddingModel.embedContent(testText);
    console.log('Embedding created successfully!');
    console.log('Embedding length:', result.embedding.values.length);
    console.log('First 5 values:', result.embedding.values.slice(0, 5));
    
  } catch (error) {
    console.error('Error creating embedding:', error.message);
    console.error('Stack trace:', error.stack);
  }
}

testEmbedding();