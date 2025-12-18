// RAG Processor for Physical AI Course Book
// This script processes book content and creates embeddings for the chatbot

const fs = require('fs');
const path = require('path');
const { GoogleGenerativeAI } = require('@google/generative-ai');

// Load environment variables
require('dotenv').config({ path: path.resolve(__dirname, '../.env') });

// Initialize Gemini API
const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY);

// Function to read all markdown files from docs directory
async function readBookContent() {
  const docsDir = path.join(__dirname, '..', 'docs');
  const files = fs.readdirSync(docsDir);
  const markdownFiles = files.filter(file => file.endsWith('.md'));
  
  console.log(`Found ${markdownFiles.length} markdown files to process`);
  
  const documents = [];
  
  for (const file of markdownFiles) {
    const filePath = path.join(docsDir, file);
    const content = fs.readFileSync(filePath, 'utf8');
    
    // Split content into chunks (simplified approach)
    const chunks = splitIntoChunks(content, 1000); // ~1000 characters per chunk
    
    chunks.forEach((chunk, index) => {
      documents.push({
        id: `${file}-${index}`,
        source: file,
        content: chunk,
        metadata: {
          chapter: file.replace('.md', ''),
          chunkIndex: index
        }
      });
    });
    
    console.log(`Processed ${file} into ${chunks.length} chunks`);
  }
  
  return documents;
}

// Simple function to split text into chunks
function splitIntoChunks(text, chunkSize) {
  const chunks = [];
  let currentChunk = '';
  
  // Split by lines to avoid breaking sentences
  const lines = text.split('\n');
  
  for (const line of lines) {
    if (currentChunk.length + line.length > chunkSize && currentChunk.length > 0) {
      chunks.push(currentChunk.trim());
      currentChunk = line + '\n';
    } else {
      currentChunk += line + '\n';
    }
  }
  
  if (currentChunk.trim()) {
    chunks.push(currentChunk.trim());
  }
  
  return chunks;
}

// Function to create embeddings using Gemini
async function createEmbeddings(documents) {
  console.log('Creating embeddings for documents...');
  
  const embeddingModel = genAI.getGenerativeModel({ model: "embedding-001" });
  
  const embeddedDocuments = [];
  
  for (let i = 0; i < documents.length; i++) {
    const doc = documents[i];
    
    try {
      // Create embedding for the document content
      const embeddingResult = await embeddingModel.embedContent(doc.content);
      const embedding = embeddingResult.embedding.values;
      
      embeddedDocuments.push({
        ...doc,
        embedding: embedding
      });
      
      console.log(`Created embedding for document ${i + 1}/${documents.length}`);
    } catch (error) {
      console.error(`Error creating embedding for document ${doc.id}:`, error.message);
    }
  }
  
  return embeddedDocuments;
}

// Function to save embeddings to a file (simplified storage)
function saveEmbeddings(embeddedDocuments) {
  const outputPath = path.join(__dirname, 'embeddings.json');
  fs.writeFileSync(outputPath, JSON.stringify(embeddedDocuments, null, 2));
  console.log(`Saved ${embeddedDocuments.length} embeddings to ${outputPath}`);
}

// Main processing function
async function processBookContent() {
  try {
    console.log('Starting RAG processing for Physical AI Course Book...');
    
    // Check if GEMINI_API_KEY is available
    if (!process.env.GEMINI_API_KEY) {
      console.error('GEMINI_API_KEY is not set in .env file');
      process.exit(1);
    }
    
    // Read book content
    const documents = await readBookContent();
    
    // Create embeddings
    const embeddedDocuments = await createEmbeddings(documents);
    
    // Save embeddings
    saveEmbeddings(embeddedDocuments);
    
    console.log('RAG processing completed successfully!');
    console.log(`Processed ${documents.length} document chunks`);
    console.log(`Created ${embeddedDocuments.length} embeddings`);
    
  } catch (error) {
    console.error('Error during RAG processing:', error.message);
    process.exit(1);
  }
}

// Run the processor if this script is executed directly
if (require.main === module) {
  processBookContent();
}

module.exports = { readBookContent, createEmbeddings, saveEmbeddings, processBookContent };