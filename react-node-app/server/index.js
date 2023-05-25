const express = require("express");
const cors = require("cors");

const PORT = process.env.PORT || 3001;

const app = express();

app.use(cors());

app.get("/numericalInput", (req, res) => {
  var numericalInput = 69; // Replace with your desired numerical input

  res.json({
    numericalInput: numericalInput,
  });
});

app.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`);
});

