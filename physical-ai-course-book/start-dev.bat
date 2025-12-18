@echo off
echo 🚀 Starting Physical AI Course Book development environment...
echo.

REM Start backend server
echo 1. Starting backend server...
cd backend
start "Backend Server" cmd /k "node server.js"
cd ..

REM Wait a moment for backend to start, then start frontend
timeout /t 3 /nobreak >nul
echo.
echo 2. Starting frontend development server...
npx docusaurus start

echo.
echo 📝 Notes:
echo    - Backend API will be available at http://localhost:3003
echo    - Frontend will be available at http://localhost:3000
echo    - Press Ctrl+C to stop the frontend server
echo    - Close the backend server window to stop the backend