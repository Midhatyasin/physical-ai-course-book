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

## 🧠 Tech Stack
- **Frontend**: Docusaurus + Spec-Kit Plus
- **Backend**: FastAPI
- **Database**: Supabase (PostgreSQL)
- **Vector DB**: Qdrant Cloud
- **LLM**: Google Gemini API
- **Auth**: Self-hosted Better Auth

## 🌐 Open Source Alternatives
- **NeonDB → Supabase**: [supabase.com](https://supabase.com)
- **Auth → Better Auth**: [github.com/better-auth/better-auth](https://github.com/better-auth/better-auth)

## 📚 Deployment
- GitHub Pages (via `gh-pages` plugin)