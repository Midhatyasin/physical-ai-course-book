// Authentication Test Script
// This script will automatically test the registration and login endpoints

// Use node-fetch for compatibility
const fetch = require('node-fetch');

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
    const registerResponse = await fetch(`${BASE_URL}/register`, {
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
    const loginResponse = await fetch(`${BASE_URL}/login`, {
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
    const sessionResponse = await fetch(`${BASE_URL}/session`, {
      method: 'GET',
      credentials: 'include',
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