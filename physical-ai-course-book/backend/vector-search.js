// Vector search utility for finding relevant documents

const fs = require('fs');
const path = require('path');

// Function to calculate cosine similarity between two vectors
function cosineSimilarity(vecA, vecB) {
  if (vecA.length !== vecB.length) {
    throw new Error('Vectors must have the same length');
  }
  
  let dotProduct = 0;
  let magnitudeA = 0;
  let magnitudeB = 0;
  
  for (let i = 0; i < vecA.length; i++) {
    dotProduct += vecA[i] * vecB[i];
    magnitudeA += vecA[i] * vecA[i];
    magnitudeB += vecB[i] * vecB[i];
  }
  
  magnitudeA = Math.sqrt(magnitudeA);
  magnitudeB = Math.sqrt(magnitudeB);
  
  if (magnitudeA === 0 || magnitudeB === 0) {
    return 0;
  }
  
  return dotProduct / (magnitudeA * magnitudeB);
}

// Function to find the most relevant documents
async function findRelevantDocuments(queryEmbedding, topK = 3) {
  try {
    // Load embeddings from file
    const embeddingsPath = path.join(__dirname, 'embeddings.json');
    
    if (!fs.existsSync(embeddingsPath)) {
      console.warn('Embeddings file not found. Please run the content processing first.');
      return [];
    }
    
    const embeddingsData = fs.readFileSync(embeddingsPath, 'utf8');
    const embeddedDocuments = JSON.parse(embeddingsData);
    
    // Calculate similarities
    const similarities = embeddedDocuments.map(doc => ({
      document: doc,
      similarity: cosineSimilarity(queryEmbedding, doc.embedding)
    }));
    
    // Sort by similarity and take top K
    similarities.sort((a, b) => b.similarity - a.similarity);
    
    return similarities.slice(0, topK).map(item => item.document);
  } catch (error) {
    console.error('Error finding relevant documents:', error.message);
    return [];
  }
}

module.exports = { cosineSimilarity, findRelevantDocuments };