# EEEBalanceBug
2nd Year EEE/EIE, Final Term Project

| Folder  | Description |
| ------------- | ------------- |
| Control  | Stores iterations of the control system software. File were originally located in Vision/ESP32 and commit history reflects this. balancing_threaded.ino is the final stable version of the balancing control system. It contains code for cascaded control with an acceleration outer loop but open-loop pitch control proved more effective for moving the rover, so the outer-loop is bypassed. The accelerometer readings were far too noisy, causing large vibrations.  |
| DeadReckoning Trial  | Contains our initial attempt at dead reckoning. The version that was actually used is within the ESP32Codes folder  |
| Energy  | Stores all code used to perform the MPPT algorithm on the Boost SMPS and code used to control the Buck LED Driver SMPS  |
| ShortestPath  | Stores all versions of the Shortest Path code used during the project, to create a lined image from the coordinates and stores the lined image in a file. The lined image can then be image processed to find the shortest path in MatLab.   |
| Vision  | Stores all versions of hardware and software files for FPGA end. (Camera side color detection) |
| LearningFrontEnd | Used to store trials for the front-end web app and server before adding it to a new GitHub repository  |
| ESP32Codes  | Stores all versions of ESP32 Arduino code used during the project, the final code used was Final.ino |

Rover WebServer Repository: https://github.com/Riya050302/rover-webserver

Rover WebApp Repository: https://github.com/anish-narain/rover-webapp

There is a ReadMe within each folder explaining the contents in more detail.
