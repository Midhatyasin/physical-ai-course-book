import React from 'react';
import { useAuth } from '../hooks/useAuth';

export default function AuthWrapper({ children }) {
  const { user, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  return user ? children : (
    <div style={{ padding: '20px', textAlign: 'center' }}>
      <h2>Authentication Required</h2>
      <p>Please log in to view this content.</p>
      <p>Features like personalization and Urdu translation require a logged-in account.</p>
    </div>
  );
}