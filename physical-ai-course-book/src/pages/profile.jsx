import React, { useState } from 'react';
import { useAuth } from '../hooks/useAuth';

export default function ProfilePage() {
  const { user, logout } = useAuth();
  const [softwareBackground, setSoftwareBackground] = useState(user?.softwareBackground || '');
  const [hardwareBackground, setHardwareBackground] = useState(user?.hardwareBackground || '');

  const handleSave = async () => {
    // In a real implementation, this would save to the backend
    alert('Profile saved!');
  };

  const handleLogout = async () => {
    const result = await logout();
    if (result.success) {
      alert('Logged out successfully!');
    } else {
      alert('Error logging out: ' + result.error);
    }
  };

  return (
    <div style={{ maxWidth: '600px', margin: '0 auto', padding: '20px' }}>
      <h2>User Profile</h2>
      
      <div style={{ marginBottom: '20px' }}>
        <h3>Account Information</h3>
        <p><strong>Email:</strong> {user?.email}</p>
        <p><strong>Name:</strong> {user?.name}</p>
      </div>
      
      <div style={{ marginBottom: '20px' }}>
        <h3>Background Information</h3>
        <p>This information helps personalize your learning experience.</p>
        
        <div style={{ marginBottom: '15px' }}>
          <label htmlFor="software">Software Background:</label>
          <textarea 
            id="software"
            value={softwareBackground}
            onChange={(e) => setSoftwareBackground(e.target.value)}
            style={{ width: '100%', minHeight: '80px', marginTop: '5px' }}
            placeholder="Describe your experience with programming languages, frameworks, etc."
          />
        </div>
        
        <div style={{ marginBottom: '15px' }}>
          <label htmlFor="hardware">Hardware Background:</label>
          <textarea 
            id="hardware"
            value={hardwareBackground}
            onChange={(e) => setHardwareBackground(e.target.value)}
            style={{ width: '100%', minHeight: '80px', marginTop: '5px' }}
            placeholder="Describe your experience with robotics hardware, sensors, etc."
          />
        </div>
        
        <button 
          onClick={handleSave}
          style={{ padding: '10px 20px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
        >
          Save Profile
        </button>
      </div>
      
      <div>
        <button 
          onClick={handleLogout}
          style={{ padding: '10px 20px', backgroundColor: '#dc3545', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
        >
          Logout
        </button>
      </div>
    </div>
  );
}