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

## Development

For development with auto-reload:
```bash
npm run dev
```