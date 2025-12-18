@echo off
echo 🧪 Running Authentication Tests...
echo.

REM Check if Node.js is installed
node --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ❌ Node.js is not installed. Please install Node.js first.
    echo Download from: https://nodejs.org/
    pause
    exit /b 1
)

echo ✅ Node.js is installed
echo.

REM Run the authentication test script
echo 🚀 Executing authentication tests...
node test-auth.js

echo.
echo 📝 Test completed. Check the results above.
pause