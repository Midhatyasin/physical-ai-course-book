import React, { useState } from 'react';

export default function TranslateButton() {
  const [lang, setLang] = useState('en');

  const toggleLanguage = () => {
    const newLang = lang === 'en' ? 'ur' : 'en';
    setLang(newLang);
    document.body.dir = newLang === 'ur' ? 'rtl' : 'ltr';
    alert(`Language switched to ${newLang === 'ur' ? 'Urdu' : 'English'}`);
  };

  return (
    <button onClick={toggleLanguage} style={{ margin: '10px' }}>
      🌍 {lang === 'en' ? 'اردو' : 'English'}
    </button>
  );
}