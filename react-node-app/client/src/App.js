import React, { useState, useEffect } from 'react';
import './App.css';

function App() {
  const [roverCoordinates, setRoverCoordinates] = useState(null);
  const [manualMode, setManualMode] = useState(false);
  const [mode, setMode] = useState('manual');

  useEffect(() => {
    fetchRoverCoordinates().then((data) => {
      setRoverCoordinates(data.roverCoordinates);
    });
  }, []);

  useEffect(() => {
    fetch('http://18.134.98.192:3001/setManualMode', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ mode }),
    })
      .then((response) => {
        // Handle the response from the server if needed
      })
      .catch((error) => {
        console.error('Error:', error);
      });
  }, [mode]);

  const fetchRoverCoordinates = async () => {
    const response = await fetch('http://18.134.98.192:3001/roverCoordinates');
    const data = await response.json();
    return data;
  };

  const handleMvmtClick = async (direction) => {
    await fetch('http://18.134.98.192:3001/mvmtClickPost', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ direction }),
    });
    // Handle the response from the server if needed
  };

  const handleModeChange = () => {
    setMode(manualMode ? 'automatic' : 'manual');
    setManualMode(!manualMode);
  };

  const MazeDisplay = () => {
    const maze = [
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1],
    ];

    return (
      <div className="maze-display">
        {maze.map((row, rowIndex) => (
          <div key={rowIndex} className="maze-row">
            {row.map((cell, columnIndex) => (
              <div
                key={columnIndex}
                className={`maze-cell ${cell === 1 ? "wall" : "path"}`}
              />
            ))}
          </div>
        ))}
      </div>
    );
  };

  return (
    <div className="App">
      <h1 className="title">EE Maze Mapper!</h1>
      <p>Current Rover coordinates: {roverCoordinates}</p>
      <MazeDisplay /> {/* Include the maze display component */}
      <div className="button-container">
        {manualMode && (
          <>
            <button onClick={() => handleMvmtClick('Up')} className="button">
              Up
            </button>
            <button onClick={() => handleMvmtClick('Down')} className="button">
              Down
            </button>
            <button onClick={() => handleMvmtClick('Left')} className="button">
              Left
            </button>
            <button onClick={() => handleMvmtClick('Right')} className="button">
              Right
            </button>
          </>
        )}
      </div>
      <div className="mode-container">
        <button onClick={handleModeChange} style={{ marginTop: '20px' }}>
          {manualMode ? 'Switch to Automatic Mode' : 'Switch to Manual Mode'}
        </button>
      </div>
    </div>
  );
}

export default App;
