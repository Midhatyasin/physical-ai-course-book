import React, { useState } from 'react';

export default function ChatBot() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  const sendMessage = async () => {
    try {
      // Add user message immediately
      const userMessage = { user: input, bot: '' };
      setMessages(prev => [...prev, userMessage]);
      setInput('');
      
      // Simulate API call
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text: input })
      });
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      
      const data = await response.json();
      
      // Update with bot response
      setMessages(prev => {
        const updated = [...prev];
        updated[updated.length - 1] = { user: input, bot: data.reply };
        return updated;
      });
    } catch (error) {
      console.error('Error sending message:', error);
      // Show error message in chat
      setMessages(prev => {
        const updated = [...prev];
        updated[updated.length - 1] = { 
          user: input, 
          bot: `Sorry, I encountered an error: ${error.message}. This is expected since we don't have a backend API set up yet.` 
        };
        return updated;
      });
    }
  };

  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', width: '300px', border: '1px solid #ccc', padding: '10px', backgroundColor: 'white', zIndex: 1000 }}>
      <h4>🤖 Chat with Book</h4>
      <div style={{ height: '200px', overflowY: 'scroll', marginBottom: '10px' }}>
        {messages.map((m, i) => (
          <div key={i} style={{ marginBottom: '10px' }}>
            <div><strong>You:</strong> {m.user}</div>
            <div><strong>Bot:</strong> {m.bot}</div>
          </div>
        ))}
        {messages.length === 0 && (
          <div style={{ fontStyle: 'italic', color: '#666' }}>
            Send a message to chat with the book!
          </div>
        )}
      </div>
      <div style={{ display: 'flex' }}>
        <input 
          value={input} 
          onChange={e => setInput(e.target.value)} 
          placeholder="Ask something..." 
          style={{ flex: 1, marginRight: '5px' }}
          onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
        />
        <button onClick={sendMessage}>Send</button>
      </div>
    </div>
  );
}