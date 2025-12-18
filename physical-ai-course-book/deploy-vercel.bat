@echo off
echo 🚀 Starting deployment process to Vercel...
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

REM Check if Vercel CLI is installed
vercel --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ℹ️  Vercel CLI is not installed. Installing...
    npm install -g vercel
    if %errorlevel% neq 0 (
        echo ❌ Failed to install Vercel CLI
        pause
        exit /b 1
    )
    echo ✅ Vercel CLI installed successfully
) else (
    echo ✅ Vercel CLI is installed
)

echo.
echo 🏗️  Building the project...
npm run build
if %errorlevel% neq 0 (
    echo ❌ Build failed
    pause
    exit /b 1
)
echo ✅ Build completed successfully

echo.
echo ☁️  Deploying to Vercel...
vercel --prod
if %errorlevel% neq 0 (
    echo ℹ️  If this is your first time deploying, you may need to log in to Vercel.
    echo Run "vercel login" first, then try again.
    echo Or visit https://vercel.com/new to set up your project manually.
    pause
    exit /b 1
)

echo.
echo 🎉 Deployment completed successfully!
echo.
echo 📝 Deployment Notes:
echo    - Your site will be available at the URL provided by Vercel
echo    - The first deployment may take a few minutes
echo    - Subsequent deployments will be faster
echo    - You can check your deployments at https://vercel.com/dashboard
echo.
pause