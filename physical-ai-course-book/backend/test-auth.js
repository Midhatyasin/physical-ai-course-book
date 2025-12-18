// Authentication Test Script
// This script will automatically test the registration and login endpoints

// Use built-in https module for compatibility
const https = require('https');
const http = require('http');
const url = require('url');

// Custom fetch-like function using built-in modules
function customFetch(requestUrl, options = {}) {
  return new Promise((resolve, reject) => {
    const parsedUrl = url.parse(requestUrl);
    const lib = parsedUrl.protocol === 'https:' ? https : http;
    
    const requestOptions = {
      hostname: parsedUrl.hostname,
      port: parsedUrl.port,
      path: parsedUrl.path,
      method: options.method || 'GET',
      headers: options.headers || {}
    };
    
    const req = lib.request(requestOptions, (res) => {
      let data = '';
      
      res.on('data', (chunk) => {
        data += chunk;
      });
      
      res.on('end', () => {
        resolve({
          ok: res.statusCode >= 200 && res.statusCode < 300,
          status: res.statusCode,
          statusText: res.statusMessage,
          json: () => Promise.resolve(JSON.parse(data)),
          text: () => Promise.resolve(data)
        });
      });
    });
    
    req.on('error', (error) => {
      reject(error);
    });
    
    if (options.body) {
      req.write(options.body);
    }
    
    req.end();
  });
}

async function testAuthentication() {
  const BASE_URL = 'http://localhost:3003/api/auth';
  
  // Test user credentials
  const testUser = {
    email: 'test@example.com',
    password: 'password123',
    name: 'Test User'
  };
  
  console.log('🧪 Starting Authentication Tests...\n');
  
  try {
    // Test 1: Register a new user
    console.log('1. Testing User Registration...');
    const registerResponse = await customFetch(`${BASE_URL}/register`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(testUser),
    });
    
    if (registerResponse.ok) {
      const registerData = await registerResponse.json();
      console.log('✅ Registration Successful!');
      console.log('   User ID:', registerData.user?.id);
      console.log('   User Email:', registerData.user?.email);
      console.log('   User Name:', registerData.user?.name);
    } else {
      const errorData = await registerResponse.json();
      console.log('❌ Registration Failed:', errorData.message || registerResponse.statusText);
      // Continue with login test even if registration fails (user might already exist)
    }
    
    console.log('');
    
    // Test 2: Login with the user
    console.log('2. Testing User Login...');
    const loginResponse = await customFetch(`${BASE_URL}/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        email: testUser.email,
        password: testUser.password
      }),
    });
    
    if (loginResponse.ok) {
      const loginData = await loginResponse.json();
      console.log('✅ Login Successful!');
      console.log('   User ID:', loginData.user?.id);
      console.log('   User Email:', loginData.user?.email);
      console.log('   User Name:', loginData.user?.name);
    } else {
      const errorData = await loginResponse.json();
      console.log('❌ Login Failed:', errorData.message || loginResponse.statusText);
    }
    
    console.log('');
    
    // Test 3: Check session
    console.log('3. Testing Session Check...');
    const sessionResponse = await customFetch(`${BASE_URL}/session`, {
      method: 'GET',
    });
    
    if (sessionResponse.ok) {
      const sessionData = await sessionResponse.json();
      console.log('✅ Session Check Successful!');
      console.log('   User Authenticated:', !!sessionData.user);
      if (sessionData.user) {
        console.log('   Session User Email:', sessionData.user.email);
      }
    } else {
      console.log('ℹ️  Session Check Failed (This is expected if not logged in):', sessionResponse.status);
    }
    
    console.log('\n🎉 Authentication Tests Completed!');
    
  } catch (error) {
    console.error('💥 Error during authentication tests:', error.message);
    console.log('\n🔧 Troubleshooting Tips:');
    console.log('   1. Make sure the backend server is running on port 3003');
    console.log('   2. Check that the database connection is properly configured');
    console.log('   3. Verify that the Supabase credentials are correct in your .env file');
  }
}

// Run the tests
testAuthentication();