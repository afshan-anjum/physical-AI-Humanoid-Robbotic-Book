import React, { useState } from 'react';

function InteractiveDiagram({ initialValue = 0 }) {
  const [value, setValue] = useState(initialValue);

  const increment = () => setValue(value + 1);
  const decrement = () => setValue(value - 1);

  return (
    <div style={{
      border: '1px solid #ccc',
      padding: '20px',
      borderRadius: '8px',
      textAlign: 'center',
      margin: '20px 0',
      backgroundColor: 'var(--ifm-background-color)'
    }}>
      <p>Interactive Diagram Placeholder</p>
      <p>Current Value: <strong>{value}</strong></p>
      <button onClick={decrement} style={{ margin: '0 10px' }}>Decrement</button>
      <button onClick={increment} style={{ margin: '0 10px' }}>Increment</button>
      <p style={{ marginTop: '15px', fontSize: '0.8em', color: '#666' }}>
        (This is a sample custom React component)
      </p>
    </div>
  );
}

export default InteractiveDiagram;