import React, { useState } from 'react';

export default function ChatBot() {
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  const sendMessage = async () => {
    const res = await fetch('/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text: input })
    });
    const data = await res.json();
    setMessages([...messages, { user: input, bot: data.reply }]);
    setInput('');
  };

  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', width: '300px', border: '1px solid #ccc', padding: '10px' }}>
      <h4>🤖 Chat with Book</h4>
      <div style={{ height: '200px', overflowY: 'scroll' }}>
        {messages.map((m, i) => (
          <div key={i}>
            <strong>You:</strong> {m.user}<br/>
            <strong>Bot:</strong> {m.bot}
          </div>
        ))}
      </div>
      <input value={input} onChange={e => setInput(e.target.value)} placeholder="Ask something..." />
      <button onClick={sendMessage}>Send</button>
    </div>
  );
}