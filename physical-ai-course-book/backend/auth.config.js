// Auth configuration for Better Auth
require('dotenv').config();

const authConfig = {
  // Database configuration (using Supabase/PostgreSQL)
  database: {
    type: 'postgres',
    url: process.env.SUPABASE_URL,
    authToken: process.env.SUPABASE_ANON_KEY,
  },
  
  // Secret key for JWT signing
  secret: process.env.AUTH_SECRET,
  
  // Social providers (optional)
  socialProviders: {
    // You can add Google, GitHub, etc. here if needed
  },
  
  // Email verification (optional)
  emailVerification: {
    sendEmail: false, // Set to true if you want email verification
  },
  
  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 1 week
    cookie: {
      secure: false, // Set to true in production with HTTPS
      httpOnly: true,
      sameSite: 'lax',
    },
  },
  
  // User configuration
  user: {
    // Fields required during registration
    additionalFields: {
      softwareBackground: {
        type: 'string',
        required: false,
      },
      hardwareBackground: {
        type: 'string',
        required: false,
      },
    },
  },
};

module.exports = authConfig;