// Debug script for testing mock embedding creation

function testMockEmbedding() {
  try {
    console.log('Testing mock embedding creation...');
    
    // Test with a simple text
    const testText = "What is Physical AI?";
    
    console.log('Creating mock embedding for:', testText);
    
    // Create mock embedding (random values)
    const mockEmbedding = Array.from({length: 768}, () => Math.random() * 2 - 1);
    
    console.log('Mock embedding created successfully!');
    console.log('Embedding length:', mockEmbedding.length);
    console.log('First 5 values:', mockEmbedding.slice(0, 5));
    
  } catch (error) {
    console.error('Error creating mock embedding:', error.message);
    console.error('Stack trace:', error.stack);
  }
}

testMockEmbedding();