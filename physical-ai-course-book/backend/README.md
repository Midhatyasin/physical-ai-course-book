# Physical AI Chat API

This is the backend API for the Physical AI & Humanoid Robotics textbook chatbot.

## Setup

1. Install dependencies:
   ```bash
   npm install
   ```

2. Create a `.env` file with your API keys:
   ```env
   PORT=3003
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   ```

3. Start the server:
   ```bash
   npm start
   ```

## Endpoints

- `POST /api/chat` - Chat with the AI assistant
- `GET /api/health` - Health check endpoint
- `POST /api/auth/register` - User registration
- `POST /api/auth/login` - User login
- `GET /api/auth/session` - Get current user session

## Development

For development with auto-reload:
```bash
npm run dev
```

## Testing Authentication

To test the authentication endpoints automatically:

### Windows:
Double-click `test-auth.bat` or run:
```cmd
npm run test-auth
```

### macOS/Linux:
Run:
```bash
./test-auth.sh
```

### Manual Testing:
You can also run the test script directly:
```bash
node test-auth.js
```