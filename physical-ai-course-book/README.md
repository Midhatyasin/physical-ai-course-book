# Physical AI & Humanoid Robotics Textbook

A Docusaurus-based interactive textbook with RAG chatbot, personalization, and Urdu translation.

## 🚀 Features
- **Interactive RAG Chatbot** (Gemini API + Qdrant)
- **User Authentication** (Better Auth)
- **Content Personalization** (based on user background)
- **Urdu Translation** (per chapter)

## 📦 Setup Instructions

### 1. Clone the Repo
```bash
git clone <repo-url>
cd book-project
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Configure Environment Variables
Copy `.env.example` to `.env` and fill in your keys:
```bash
cp .env.example .env
```

### 4. Run Locally
```bash
npm start
```

### 5. Build for Production
```bash
npm run build
```

### 6. Run the Chatbot Backend (Optional)
For the chatbot to work, you need to run the backend API:

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Create a `.env` file with your API keys (copy from `.env.example` if it exists)

4. Start the backend server:
   ```bash
   npm start
   ```

5. The backend will run on port 3001 by default

## 🧠 Tech Stack
- **Frontend**: Docusaurus + Spec-Kit Plus
- **Backend**: Express.js (for chatbot API)
- **Database**: Supabase (PostgreSQL)
- **Vector DB**: Qdrant Cloud
- **LLM**: Google Gemini API
- **Auth**: Self-hosted Better Auth

## 🌐 Open Source Alternatives
- **NeonDB → Supabase**: [supabase.com](https://supabase.com)
- **Auth → Better Auth**: [github.com/better-auth/better-auth](https://github.com/better-auth/better-auth)

## 📚 Deployment

### GitHub Pages
- GitHub Pages (via `gh-pages` plugin)

### Vercel

#### Option 1: Manual Deployment
1. Push your code to GitHub
2. Create a new project on [Vercel](https://vercel.com/)
3. Import your repository
4. Vercel will automatically detect the Docusaurus project and configure the build settings
5. Click "Deploy" and your site will be live!

#### Option 2: Automated Deployment Script
We've included deployment scripts to make the process easier:

1. Install [Node.js](https://nodejs.org/) if you haven't already
2. Run the deployment script:
   - On Windows: Double-click `deploy-vercel.bat` or run `deploy-vercel.bat` in Command Prompt
   - On macOS/Linux: Run `node deploy-vercel.js` in Terminal

The script will:
- Check for and install Vercel CLI if needed
- Build your project
- Deploy to Vercel

For manual configuration, the `vercel.json` file is already included in this repository.