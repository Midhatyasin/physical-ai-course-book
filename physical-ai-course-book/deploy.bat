@echo off
echo Building the project...
npm run build
if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo Deploying to GitHub Pages...
npm run deploy
if %errorlevel% neq 0 (
    echo Deployment failed!
    exit /b %errorlevel%
)

echo Deployment completed!