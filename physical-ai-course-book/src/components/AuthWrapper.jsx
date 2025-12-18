import React from 'react';
import { useAuth } from '../hooks/useAuth';

export default function AuthWrapper({ children }) {
  const { user } = useAuth();

  return user ? children : <div>Please log in to view this content.</div>;
}