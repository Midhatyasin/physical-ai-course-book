#!/usr/bin/env node

// Script to start both frontend and backend for development
const { spawn } = require('child_process');
const path = require('path');

console.log('🚀 Starting Physical AI Course Book development environment...\n');

// Start backend server
console.log('1. Starting backend server...');
const backend = spawn('node', ['server.js'], {
  cwd: path.join(__dirname, 'backend'),
  stdio: 'inherit'
});

backend.on('error', (error) => {
  console.error('❌ Failed to start backend server:', error.message);
});

backend.on('close', (code) => {
  console.log(`Backend server exited with code ${code}`);
});

// Wait a moment for backend to start, then start frontend
setTimeout(() => {
  console.log('\n2. Starting frontend development server...');
  const frontend = spawn('npx', ['docusaurus', 'start'], {
    stdio: 'inherit'
  });

  frontend.on('error', (error) => {
    console.error('❌ Failed to start frontend server:', error.message);
  });

  frontend.on('close', (code) => {
    console.log(`Frontend server exited with code ${code}`);
  });
}, 3000);

console.log('\n📝 Notes:');
console.log('   - Backend API will be available at http://localhost:3002');
console.log('   - Frontend will be available at http://localhost:3000');
console.log('   - Press Ctrl+C to stop both servers\n');