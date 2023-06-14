import React from "react";

const MazeDisplay = () => {
  const maze = [
    [1, 1, 1, 1, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1],
    [1, 0, 0, 0, 1],
    [1, 1, 1, 1, 1]
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

export default MazeDisplay;
