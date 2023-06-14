const express = require("express");
const cors = require("cors");

const PORT = process.env.PORT || 3001;

const app = express();

app.use(cors()); // Enable CORS for all routes
app.use(express.json()); // Parse incoming JSON data

let buttonClicks = []; // Array to store button click data

app.get("/numericalInput", (req, res) => {
  var numericalInput = 69; // Replace with your desired numerical input

  res.json({
    numericalInput: numericalInput,
  });
});

app.post("/buttonClick", (req, res) => {
  const { direction } = req.body; // Extract the direction from the request body
  console.log("Button clicked:", direction); // Log the clicked direction

  buttonClicks.push(direction); // Add the clicked direction to the buttonClicks array

  res.sendStatus(200); // Send a success status code (200)
});

app.get("/buttonClicks", (req, res) => {
  res.json({
    buttonClicks: buttonClicks, // Return the buttonClicks array as JSON response
  });
});

app.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`); // Start the server and log the port it's listening on
});
