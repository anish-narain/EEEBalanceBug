import React, { useState, useEffect } from "react";
import "./App.css";

function App() {
  const [inputValue, setInputValue] = useState("");

  useEffect(() => {
    // Simulating server request to retrieve numerical input
    fetchNumericalInput().then((response) => {
      setInputValue(response);
    });
  }, []);

  const fetchNumericalInput = async () => {
    // Simulating server request
    const response = await fetch("http://localhost:3001/numericalInput");
    const data = await response.json();
    return data.numericalInput;
  };

  const handleInputChange = (event) => {
    setInputValue(event.target.value);
  };

  const handleButtonClick = async (direction) => {
    const response = await fetch("http://localhost:3001/buttonClick", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ direction }),
    });
    // Handle the response from the server if needed
  };

  return (
    <div className="App">
      <p>This text is printed inside the App component</p>
      <div className="input-container">
        <input type="number" value={inputValue} onChange={handleInputChange} />
      </div>
      <div className="button-container">
        <button onClick={() => handleButtonClick("Up")}>Up</button>
        <button onClick={() => handleButtonClick("Down")}>Down</button>
        <button onClick={() => handleButtonClick("Left")}>Left</button>
        <button onClick={() => handleButtonClick("Right")}>Right</button>
      </div>
    </div>
  );
}

export default App;
