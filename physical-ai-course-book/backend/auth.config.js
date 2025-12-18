// Auth configuration for Better Auth
require('dotenv').config();

const authConfig = {
  // Database configuration (using Supabase/PostgreSQL)
  database: process.env.SUPABASE_URL,
  
  // Secret key for JWT signing
  secret: process.env.AUTH_SECRET,
  
  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 1 week
    cookie: {
      secure: false, // Set to true in production with HTTPS
      httpOnly: true,
      sameSite: 'lax',
    },
  },
};

module.exports = authConfig;

module.exports = authConfig;