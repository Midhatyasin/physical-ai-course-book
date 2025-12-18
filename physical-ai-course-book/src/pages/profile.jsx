import React from 'react';
import { useAuth } from '../hooks/useAuth';

export default function ProfilePage() {
  const { user } = useAuth();

  return (
    <div>
      <h2>User Profile</h2>
      <p>Name: {user?.name}</p>
      <p>Level: {user?.level}</p>
    </div>
  );
}