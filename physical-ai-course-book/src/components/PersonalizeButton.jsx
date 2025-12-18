import React from 'react';

export default function PersonalizeButton() {
  const adjustContent = () => {
    alert('Content adjusted based on your profile!');
  };

  return (
    <button onClick={adjustContent} style={{ margin: '10px' }}>
      🎛️ Personalize Chapter
    </button>
  );
}