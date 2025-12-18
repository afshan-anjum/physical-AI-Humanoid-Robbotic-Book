import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import InteractiveDiagram from '../InteractiveDiagram';

describe('InteractiveDiagram', () => {
  test('renders with initial value and increments/decrements', () => {
    render(<InteractiveDiagram initialValue={5} />);

    // Check if initial value is rendered
    expect(screen.getByText(/Current Value: 5/i)).toBeInTheDocument();

    // Increment
    fireEvent.click(screen.getByRole('button', { name: /increment/i }));
    expect(screen.getByText(/Current Value: 6/i)).toBeInTheDocument();

    // Decrement
    fireEvent.click(screen.getByRole('button', { name: /decrement/i }));
    expect(screen.getByText(/Current Value: 5/i)).toBeInTheDocument();
  });

  test('handles default initial value if not provided', () => {
    render(<InteractiveDiagram />);
    expect(screen.getByText(/Current Value: 0/i)).toBeInTheDocument();
  });
});