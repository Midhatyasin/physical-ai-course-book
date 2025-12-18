#!/usr/bin/env node

// This script helps deploy the Physical AI course book to Vercel
// Prerequisites:
// 1. Node.js installed
// 2. Vercel CLI installed (npm install -g vercel)
// 3. Vercel account

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

console.log('🚀 Starting deployment process to Vercel...\n');

try {
  // Check if vercel CLI is installed
  console.log('1. Checking for Vercel CLI...');
  execSync('vercel --version', { stdio: 'pipe' });
  console.log('   ✅ Vercel CLI is installed\n');
} catch (error) {
  console.log('   ❌ Vercel CLI is not installed');
  console.log('   Installing Vercel CLI...');
  execSync('npm install -g vercel', { stdio: 'inherit' });
  console.log('   ✅ Vercel CLI installed successfully\n');
}

// Check if we're in the correct directory
const packageJsonPath = path.join(process.cwd(), 'package.json');
if (!fs.existsSync(packageJsonPath)) {
  console.error('❌ Error: package.json not found. Please run this script from the project root directory.');
  process.exit(1);
}

// Build the project
console.log('2. Building the project...');
try {
  execSync('npm run build', { stdio: 'inherit' });
  console.log('   ✅ Build completed successfully\n');
} catch (error) {
  console.error('❌ Error: Build failed');
  process.exit(1);
}

// Deploy to Vercel
console.log('3. Deploying to Vercel...');
try {
  console.log('   Running: vercel --prod');
  execSync('vercel --prod', { stdio: 'inherit' });
  console.log('\n   ✅ Deployment completed successfully!\n');
} catch (error) {
  console.log('   ℹ️  If this is your first time deploying, you may need to log in to Vercel.');
  console.log('   Run "vercel login" first, then try again.');
  console.log('   Or visit https://vercel.com/new to set up your project manually.');
}

console.log('📝 Deployment Notes:');
console.log('   - Your site will be available at the URL provided by Vercel');
console.log('   - The first deployment may take a few minutes');
console.log('   - Subsequent deployments will be faster');
console.log('   - You can check your deployments at https://vercel.com/dashboard');