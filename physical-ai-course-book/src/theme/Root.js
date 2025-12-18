import React, { useState, useEffect } from 'react';
import ChatBot from '@site/src/components/ChatBot';
import AuthForm from '@site/src/components/AuthForm';
import { useAuth } from '@site/src/hooks/useAuth';

// Default implementation, that you can customize
export default function Root({children}) {
  const { user, loading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  
  // Show auth modal if user is not logged in
  useEffect(() => {
    if (!loading && !user) {
      setShowAuthModal(true);
    }
  }, [user, loading]);
  
  const closeAuthModal = () => {
    setShowAuthModal(false);
  };
  
  return (
    <>
      {children}
      <ChatBot />
      
      {/* Authentication Modal */}
      {showAuthModal && (
        <div style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(0, 0, 0, 0.5)',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          zIndex: 10000
        }}>
          <div style={{
            backgroundColor: 'white',
            borderRadius: '8px',
            padding: '20px',
            maxWidth: '500px',
            width: '90%',
            maxHeight: '90vh',
            overflowY: 'auto'
          }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '15px' }}>
              <h3>Welcome to Physical AI & Humanoid Robotics</h3>
              <button 
                onClick={closeAuthModal}
                style={{
                  background: 'none',
                  border: 'none',
                  fontSize: '1.5rem',
                  cursor: 'pointer'
                }}
              >
                ×
              </button>
            </div>
            <p>Sign in or register to unlock personalized features:</p>
            <ul>
              <li>Content personalization based on your background</li>
              <li>Urdu translation of chapters</li>
              <li>Progress tracking</li>
            </ul>
            <AuthForm />
          </div>
        </div>
      )}
    </>
  );
}