import { useState, useEffect } from 'react';

export function useAuth() {
  const [user, setUser] = useState(null);

  useEffect(() => {
    // Simulate fetching user
    setUser({ name: 'Student', level: 'Intermediate' });
  }, []);

  return { user };
}