// Test script for RAG processor

async function testRAGProcessor() {
  try {
    console.log('Testing RAG processor...');
    
    // Import the processor
    const { processBookContent } = require('./rag-processor');
    
    // Run the processor
    await processBookContent();
    
    console.log('RAG processor test completed!');
  } catch (error) {
    console.error('Error testing RAG processor:', error.message);
  }
}

// Run the test
testRAGProcessor();