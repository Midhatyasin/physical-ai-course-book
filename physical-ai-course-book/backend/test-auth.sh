#!/bin/bash

echo "🧪 Running Authentication Tests..."
echo ""

# Check if Node.js is installed
if ! command -v node &> /dev/null
then
    echo "❌ Node.js is not installed. Please install Node.js first."
    echo "Download from: https://nodejs.org/"
    exit 1
fi

echo "✅ Node.js is installed"
echo ""

# Run the authentication test script
echo "🚀 Executing authentication tests..."
node test-auth.js

echo ""
echo "📝 Test completed. Check the results above."