#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <freertos/task.h>

Adafruit_MPU6050 mpu;
float gyroZe;

#define dirPin1 18
#define stepPin1 19
#define dirPin2 5
#define stepPin2 23
#define stepsPerRevolution 200

#define pinF 33
#define pinB 35
#define pinT 32

//UART STUFF=====================================================================
#define RXD2 16  // FPGA side: (ARDUINO_IO[9]) yellow
#define TXD2 17  // FPGA side: (ARDUINO_IO[8])

//unsigned long start = 0;
//unsigned long period = 500;

unsigned int dist[3] = { 0 };
bool col_detect = true;
int zoom_level = 1;

int r_state = -1;
int t_flag = 0;

unsigned long prev = 0;
unsigned long period = 30000;
float calibrate_x = 1.0;
float calibrate_y = 1.0;  // used for coordinates calibration


int byte2int(byte* buf, int size) {
  int val = 0;

  for (int i = 0; i < size; i++) {
    val += buf[i] << (8 * i);
  }

  return val;
}

int byte2int_signed(byte* buf, int size) {
  int val = 0;

  for (int i = 0; i < (size - 1); i++) {
    val += buf[i] << (8 * i);
  }

  int MSB = (signed char)buf[size - 1];
  val += MSB << (8 * (size - 1));

  return val;
}

void int2byte(int n, byte bytes[4]) {
  for (int i = 0; i < 4; i++) {
    bytes[i] = (n >> 8 * i) & 0xFF;
  }
}

void zoom(int zoom_level) {
  if (Serial1.availableForWrite() >= 4) {
    byte buf[4];
    int2byte(zoom_level, buf);
    Serial1.write(buf, 4);
    Serial.printf("%lu zoom sent\n", zoom_level);
    delay(100);
  }
}

int raw_decode(byte* buf) {

  int i;

  //start condition
  if (byte2int(buf, 4) == 0x00524242) {
    r_state = 0;
  }

  //output
  switch (r_state) {
    case 0:
      break;
    case 1:
      i = byte2int_signed(buf + 3, 1);
      if (i == -1) {
        //Serial.println("no beacons");
        for (int j = 0; j < 3; j++) {
          dist[j] = 0;
        }
      } else {
        for (int j = 0; j < 3; j++) {
          if (i == j) {
            dist[j] = byte2int(buf, 3);
            //Serial.printf("dist to beacon %d = %d mm\n", i, dist[i]);
          } else {
            dist[j] = 0;
          }
        }
      }
      break;
    case 2:
      col_detect = (bool)byte2int(buf, 4);
      break;
    default:
      break;
  }

  //next r_state
  if (r_state > -1 & r_state < 2) {
    r_state++;
  } else if (r_state == 2) {
    r_state = -1;
  }

  return 0;
}

void metrics2string() {
  //Serial.printf("dist_red = %d\n", dist[0]);
  //Serial.printf("dist_yellow = %d\n", dist[1]);
  //Serial.printf("dist_blue = %d\n", dist[2]);

  //Serial.printf("col_detect = %d\n", col_detect);
}

//MOTOR and Server STUFF=====================================================================

#define wheelD 6                                       // Wheel diameter in cm
float stepD = (3.1415 * wheelD / stepsPerRevolution);  // Distance travelled per step
float x, y, bearing;
bool stepFlag;  // High whenever a step occurs

const char* ssid = "Rover27";
const char* password = "123456788";
const char* serverAddress = "18.134.98.192";  // Replace with the IP address or hostname of your Node.js server
const int serverPort = 3001;                  // Replace with the port number your Node.js server is listening on

//Variable declarations for traversal algorithm
int coordinates[2];
int start_coordinates[2];
int current_coordinates[2];
float calibration_coordinates[2];
String left_following;
String wall_detection = "true";  //fixed rn, need to change
String maze_complete = "false";  //fixed rn, need to change
String mvmt_direction;
String server_direction = "Up";
String prev_direction;  //need this for null inputs
String recalibrateFlag;
String stopLeftFlag;
String beaconFlag;
String completeFlag;
int turningRight = -1;

//Function declarations for traversal algorithm
void roverLeftFollow();
String extractDirection(const String& jsonString);
String parseJson(const String& json, const String& key);
void roverMotion(String direction);

class stepperMotor {
public:

  void stop(void) {
    enable = 0;
  }

  void start(void) {
    enable = 1;
  }

  void init(int _pulsePin, int _dirPin, unsigned long _delayTime, bool _direction) {
    pulsePin = _pulsePin;
    dirPin = _dirPin;
    delayTime = _delayTime;
    dir = _direction;

    togglePulse = LOW;
    enable = 0;

    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void control(void) {

    currentTime = micros();
    digitalWrite(dirPin, dir);
    if (enable == 1) {
      if ((currentTime - deltaTime) > delayTime) {

        pulseCount++;

        // Each HIGH or LOW is a "pulse"
        // But each step of the motor requires two "pulses"
        if (pulseCount % 2 == 0) {
          stepCount++;
          stepFlag = true;
        }

        togglePulse = togglePulse == LOW ? HIGH : LOW;
        digitalWrite(pulsePin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }

  void changeDirection(bool _direction) {
    dir = _direction;
  }

  unsigned long steps(void) {
    return stepCount;
  }

  void changeSpeed(unsigned long _speed) {
    delayTime = _speed;
  }

private:
  unsigned long delayTime, deltaTime, currentTime;
  unsigned long pulseCount = 0;
  unsigned long stepCount = 0;
  int pulsePin, dirPin;
  bool dir, togglePulse, enable;
};

stepperMotor s1, s2;

int calibration_coor(float gyroZe) {
  float targetAngle = 6.28;   // Target angle for the turn
  float currentAngle = 0.0;   // Current angle of the turn
  float lastTime = micros();  // Last time recorded



  // dimension of the arena
  int height = 3400;
  int width = 2250;

  int sum_dist[3] = { 0 };  // red, blue, yellow
  int count[3] = { 0 };
  int avg_dist[3] = { 0 };
  float angle[3] = { 0 };



  // zoom in the camera for calibration
  zoom(2);



  // Stop the motors
  s1.stop();
  s2.stop();
  s1.control();
  s2.control();



  while (currentAngle < targetAngle) {

    // Rotate the robot in place, can be replaced by left turn -------------------------
    s1.start();
    s2.start();
    s1.changeDirection(true);
    s2.changeDirection(true);
    s1.control();
    s2.control();



    // Update the current angle based on the gyro reading (not sure if we still need to update current angle)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float Ts = micros() - lastTime;  // Time since last loop
    lastTime = micros();
    bearing += ((float)g.gyro.z - gyroZe) * (Ts / 1000000);  // Integrate angular velocity with respect to time



    // Update the current angle of the turn
    currentAngle += abs((float)g.gyro.z - gyroZe) * (Ts / 1000000);
    //Serial.println(currentAngle);

    // Call the raw_decode function to update the dist array
    if (Serial1.available() >= 4) {
      //Serial.print("MSG RECEIVED");
      byte buf[4];
      Serial1.readBytes(buf, 4);
      //int a = byte2int(buf, 4);
      //Serial.println(a, HEX);
      raw_decode(buf);
      //metrics2string();
    }
    /*
    // print each beacons distance each cycle
    for (int i = 0; i<3; i++){
      Serial.print(dist[i]);
      Serial.print(" ");
    }
    Serial.println("");
    */

    // sum up all positive distance readings
    for (int i = 0; i < 3; i++) {
      if (dist[i] > 0) {
        sum_dist[i] += dist[i];
        count[i] += 1;
        angle[i] = currentAngle;
      }
    }

    // Delay to control the speed of the turn
    delay(10);  // Adjust the delay value to control the speed of the turn
  }



  // Stop the motors after completing the turn
  s1.stop();
  s2.stop();
  s1.control();
  s2.control();



  // zoom out the camera
  zoom(1);
  /*
  for (int i = 0; i<3; i++){
     Serial.print("count");
      Serial.print(count[i]);
      Serial.print(" ");
    }
    Serial.println("");

  for (int i = 0; i<3; i++){
     Serial.print("sum");
      Serial.print(sum_dist[i]);
      Serial.print(" ");
    }
    Serial.println("");
   */

  if ((count[0] != 0 & count[2] != 0)) {

    avg_dist[0] = sum_dist[0] / count[0];
    avg_dist[2] = sum_dist[2] / count[2];

    if (angle[0] > angle[2]) {
      angle[0] = angle[0] - 6.28;
    }

    float angle_w = angle[2] - angle[0];
    /*
    for (int i = 0; i<3; i++){
     Serial.print("avg");
      Serial.print(avg_dist[i]);
      Serial.print(" ");
    }
    Serial.println("");
    */

    // calculate x and y coordinates
    float tmp = sqrt(pow(avg_dist[0], 2) + pow(avg_dist[2], 2) - 2 * avg_dist[0] * avg_dist[2] * abs(cos(angle_w)));
    float beta = asin(avg_dist[2] * sin(angle_w) / tmp);
    float x1 = avg_dist[0] * cos(beta);
    float y1 = height - avg_dist[0] * sin(beta);

    /*
    Serial.println(angle_w,4);
    Serial.println(tmp,4);
    Serial.println(beta,4);
    Serial.println(x1,4);
    Serial.println(y1,4);
    */
    calibrate_x = x1;
    calibrate_y = y1;
  } else if ((count[1] != 0 & count[2] != 0)) {
    avg_dist[1] = sum_dist[1] / count[1];
    avg_dist[2] = sum_dist[2] / count[2];

    if (angle[2] > angle[1]) {
      angle[2] = angle[2] - 6.28;
    }

    float angle_h = angle[1] - angle[2];

    float tmp = sqrt(pow(avg_dist[1], 2) + pow(avg_dist[2], 2) - 2 * avg_dist[1] * avg_dist[2] * abs(cos(angle_h)));
    float gamma = asin(avg_dist[1] * sin(angle_h) / tmp);
    float x2 = width - avg_dist[2] * sin(gamma);
    float y2 = height - avg_dist[2] * cos(gamma);

    calibrate_x = x2;
    calibrate_y = y2;
  } else if ((count[1] != 0 & count[2] != 0 & count[0] != 0)) {
    for (int i = 0; i < 3; i++) {
      avg_dist[i] = sum_dist[i] / count[i];
    }

    if (angle[1] < angle[0] & angle[0] < angle[2]) {
      angle[0] = angle[0] - 6.28;
      angle[2] = angle[2] - 6.28;
    }

    if (angle[2] < angle[1] & angle[1] < angle[0]) {
      angle[0] = angle[0] - 6.28;
    }

    float angle_w = angle[2] - angle[0];
    float angle_h = angle[1] - angle[2];

    // calculate x and y coordinates
    float tmp = sqrt(pow(avg_dist[0], 2) + pow(avg_dist[2], 2) - 2 * avg_dist[0] * avg_dist[2] * abs(cos(angle_w)));
    float beta = asin(avg_dist[2] * sin(angle_w) / tmp);
    float x1 = avg_dist[0] * cos(beta);
    float y1 = height - avg_dist[0] * sin(beta);

    float tmp1 = sqrt(pow(avg_dist[1], 2) + pow(avg_dist[2], 2) - 2 * avg_dist[1] * avg_dist[2] * abs(cos(angle_h)));
    float gamma = asin(avg_dist[1] * sin(angle_h) / tmp1);
    float x2 = width - avg_dist[2] * sin(gamma);
    float y2 = height - avg_dist[2] * cos(gamma);

    calibrate_x = (x1 + x2) / 2;
    calibrate_y = (y1 + y2) / 2;
  }

  else {
    Serial.println("Error detecting enough beacons");
    return 0;
  }
  return 1;
}

int printCount;
int input;  // Serial input data

float errorSum = 0;
float avgError = 0;
float errorF = 0;
float avgF = 0;
float errorB = 0;
float avgB = 0;
float errorT = 0;
float avgT = 0;

//FOR Phototransistor ====================================================
String direction;
int lightSensorReadingFront = 0;
int lightSensorReadingBack = 0;
int lightSensorReadingTop = 0;
//=========================================================================

int lastX;
int lastY;

void httpGetPostTask(void* parameter) {
  while (1) {
    //GET AND POST httpclient setup =================================================
    HTTPClient httpPOST;
    HTTPClient httpGET;

    // POST ENDPOINTS Setup ==========================================================
    if (turningRight == -1) {
      current_coordinates[0] = coordinates[0];
      current_coordinates[1] = coordinates[1];
    } else {
      current_coordinates[0] = lastX;
      current_coordinates[1] = lastY;
    }


    //calibration_coordinates[0] = calibrate_x/10;
    //calibration_coordinates[1] = calibrate_y/10;

    String PostEndpoint = "http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinateAndWallDetectionAndRecalibrationOutput";
    httpPOST.begin(PostEndpoint);  // Specify the server address, port, and endpoint
    httpPOST.addHeader("Content-Type", "application/json");

    // Create the JSON object
    StaticJsonDocument<128> jsonPayload;
    jsonPayload["jsonPacket"]["received_coordinates"][0] = current_coordinates[0];
    jsonPayload["jsonPacket"]["received_coordinates"][1] = current_coordinates[1];
    jsonPayload["jsonPacket"]["received_walldetection"] = wall_detection;
    jsonPayload["jsonPacket"]["recalibrate_output"] = "[" + String(calibrate_x) + "," + String(calibrate_y) + "]";


    // Convert the JSON object to a string
    String jsonString;
    serializeJson(jsonPayload, jsonString);

    // roverCoordinates POST Code ====================================================
    //Serial.print(jsonString);
    
    int httpResponseCodePost = httpPOST.POST(jsonString);

    if (httpResponseCodePost > 0) {
      //Serial.print("Button click request sent. Response code for POST : ");
      //Serial.println(httpResponseCodePost);

      String responseBody = httpPOST.getString();
      //Serial.print("Response body for POST : ");
      //Serial.println(responseBody);
    } else {
      Serial.print("Error sending request for POST. Error code: ");
      Serial.println(httpResponseCodePost);
    }
    

    //GET ENDPOINTS Setup ============================================================
    String GetEndpoint = "http://" + String(serverAddress) + ":" + String(serverPort) + "/nextDirectionAndRecalibrateAndStopLeftAndBeaconAndComplete";
    httpGET.begin(GetEndpoint);  // Specify the server address and endpoint
    
    int httpResponseCodeGet = httpGET.GET();

    if (httpResponseCodeGet > 0) {
      //Serial.print("Response code for GET: ");
      //Serial.println(httpResponseCodeGet);
      String jsonReceived = httpGET.getString();
      direction = parseJson(jsonReceived, "Direction");
      recalibrateFlag = parseJson(jsonReceived, "Recalibrate");
      stopLeftFlag = parseJson(jsonReceived, "StopLeft");
      beaconFlag = parseJson(jsonReceived, "Beacon");
      completeFlag = parseJson(jsonReceived, "Complete");
      //Serial.print("Direction: ");
      //Serial.println(direction);
      //Serial.print("Recalibrate Flag: ");
      //Serial.println(recalibrateFlag);
      Serial.print("Complete Flag: ");
      Serial.println(completeFlag);
    } else {
      Serial.print("Error code for Get: ");
      Serial.println(httpResponseCodeGet);
    }
  
    // Free resources
    httpPOST.end();
    httpGET.end();
  }
}

char dir = 'q';
float lastTime, Ts;

float sum = 0;
float f;
float b;
float t;
float count = 0;
int wallDir = 0;
int wallCount = 100;
bool canChangeDir = false;

int blockLeft = 0;
int blockLeftDir = 0;

float startAngle = 0;
int rightWallCount;

int gyroRecalCount;
bool canGyroRecal = false;

void motorTask(void* parameter) {
  while (1) {

    if (Serial1.available() >= 4) {
      byte buf[4];
      Serial1.readBytes(buf, 4);
      raw_decode(buf);
      //int a = byte2int(buf,4);
      //Serial.println(a, HEX);
    }


    //RECALIBRATION CODE
    if (recalibrateFlag == "true") {
      //int returnedVal = calibration_coor(gyroZe);
      canGyroRecal = !canGyroRecal;
      recalibrateFlag = "false";
    }

    if (beaconFlag == "true") {
      int returnedVal = calibration_coor(gyroZe);
    }

    if (completeFlag == "true"){
      Serial.println("maze traversal end");
      s1.stop();
      s2.stop();
      s1.control();
      s2.control();
      direction = "Stop";
      while (1) {}
    }

    lightSensorReadingFront = analogRead(pinF);
    lightSensorReadingBack = analogRead(pinB);
    lightSensorReadingTop = analogRead(pinT);

    sum += lightSensorReadingFront - lightSensorReadingBack - avgError;
    f += lightSensorReadingFront - avgF;
    b += lightSensorReadingBack - avgB;
    t += lightSensorReadingTop - avgT;

    if (stopLeftFlag == "true") {
      turningRight = 0;
      startAngle = bearing;
      stopLeftFlag = "false";
      rightWallCount = 0;

      lastX = x;
      lastY = y;
    }

    count += 1;
    if (count == 10) {  // 50
      float avg = sum / 10;
      float aF = f / 10;
      float aB = b / 10;
      float aT = t / 10;
      count = 0;
      sum = 0;
      f = 0;
      b = 0;
      t = 0;
      //Serial.println(avg);

      aF = aF - aT;  // Remove ambient light

      gyroRecalCount++;
      if (gyroRecalCount >= 500 && turningRight == -1 && direction == "Up" && canGyroRecal) {
        Serial.println("Gyro recal");
        s1.stop();
        s2.stop();
        s1.control();
        s2.control();
        direction = "Stop";

        delay(500);
        float zs = 0;
        for (int i = 0; i < 10; i++) {
          sensors_event_t a, g, temp;
          mpu.getEvent(&a, &g, &temp);
          zs += g.gyro.z;
          delay(10);
        }

        gyroZe = zs / 10;
        direction = "Up";

        gyroRecalCount = 0;
      }

      
      Serial.print(aF);
      Serial.print(", ");
      Serial.print(aB);
      Serial.print(", ");
      Serial.print(aT);
      Serial.print(", ");
      Serial.print(avg);
      Serial.print(", ");
      Serial.print(direction);
      Serial.print(", ");
      Serial.print(col_detect);
      Serial.print(", ");
      Serial.print(bearing);
      Serial.print(", ");
      Serial.print(x);
      Serial.print(", ");
      Serial.println(y);
      


      if (turningRight == 0) {

        direction = "Right";
        if (bearing < startAngle - 1.57) {
          turningRight = 1;
          //Serial.println(stopLeftFlag);
        }

        /*Serial.print(startAngle);
        Serial.print(", ");
        Serial.println(bearing);*/
      } else if (turningRight == 1) {

        direction = "Up";
        if (col_detect) {
          turningRight = 2;
        }



      } else if (turningRight == 2) {

        rightWallCount += 1;

        if (rightWallCount >= 15) {
          turningRight = 3;
          startAngle = bearing;
        }

      } else if (turningRight == 3) {

        direction = "Right";
        if (bearing < startAngle - 1.57) {
          turningRight = -1;
          //Serial.println(stopLeftFlag);
        }

      } else {
        if (col_detect == true) {
          if (canChangeDir) {
            if (aF > -9999) {
              wallDir = 1;
            } else {
              wallDir = -1;
            }

            canChangeDir = false;
          }

          wallCount = 0;

          if (wallDir == 1) {
            direction = "Right";
          } else {
            direction = "Left";
          }

          if (aF > 800) {
            direction = "Right";
          }


        } else {  // No wall in front
          wallCount++;
          if (wallCount > 40) {  // 8
            canChangeDir = true;
          }

          //Serial.println(aF);

          if (aB < -100 && canChangeDir && blockLeft == 0) {
            direction = "Left";
            //Serial.println("Left");
          } else if ((aB > 100 || aF > 300) && canChangeDir) {
            direction = "Right";
            //Serial.println("Turn away");
          } else if ((aB > 400 || aF > 450) && !canChangeDir && wallDir == 1) {
            direction = "Right";
            //Serial.println("right");
          } else {
            direction = "Up";
            //Serial.println("up");
          }

          if (blockLeft > 0) {

            if (blockLeftDir > 0) {
              direction = "Right";
            } else {
              direction = "Up";
            }

            blockLeft -= 1;
            blockLeftDir -= 1;
          }
          if (aB > 800) {
            //Serial.println("Blocked left");
            blockLeft = 1;
            blockLeftDir = 1;
          }
        }
      }
    }


    if (direction == "Up") {
      s1.start();
      s2.start();
      s1.changeDirection(true);
      s2.changeDirection(false);
    } else if (direction == "Left") {
      s1.start();
      s2.start();
      s1.changeDirection(false);
      s2.changeDirection(false);
    } else if (direction == "Right") {
      s1.start();
      s2.start();
      s1.changeDirection(true);
      s2.changeDirection(true);
    } else {
      s1.stop();
      s2.stop();
    }

    s1.control();
    s2.control();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // Get sensor readings
    Ts = micros() - lastTime;     // Time since last loop
    lastTime = micros();
    bearing += ((float)g.gyro.z - gyroZe) * (Ts / 1000000);  // Integrates angular velocity with respect to time

    if (stepFlag) {
      stepFlag = false;

      // Increment co-ordinates based on direction
      if (direction == "Up") {
        x += sin(bearing) * stepD;
        y += cos(bearing) * stepD;
      } else if (direction == "Down") {
        x += -1 * sin(bearing) * stepD;
        y += -1 * cos(bearing) * stepD;
      }
    }



    coordinates[0] = x;
    coordinates[1] = y;

    printCount++;
    if (printCount == 1000) {
      /*
      Serial.print("X: ");
      Serial.print(x);
      Serial.print(", Y: ");
      Serial.print(y);
      Serial.print(", Dir: ");
      Serial.println(bearing);
      */
      printCount = 0;
    }


    //delay(0);  // Wait for 0.5 seconds before sending the next request
  }
}



void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);  //FPGA

  s1.init(stepPin1, dirPin1, 5000, true);  // left
  s2.init(stepPin2, dirPin2, 5000, true);  // right - inverted direction
  //s1.start();
  //s2.start();

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
  Serial.println("Calibrating");  // Do not move whilst calibrating
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroZe = g.gyro.z;
  Serial.println("Done");

  //calibration_coor(gyroZe);


  for (int i = 0; i < 50; i++) {
    lightSensorReadingFront = analogRead(pinF);
    lightSensorReadingBack = analogRead(pinB);
    lightSensorReadingTop = analogRead(pinT);
    errorSum += lightSensorReadingFront - lightSensorReadingBack;
    errorF += lightSensorReadingFront;
    errorB += lightSensorReadingBack;
    errorT += lightSensorReadingTop;
  }

  avgError = errorSum / 50;
  avgF = errorF / 50;
  avgB = errorB / 50;
  avgT = errorT / 50;

  Serial.println(avgError);
  delay(100);

  //reset buffer
  zoom(6);
  delay(100);

  xTaskCreatePinnedToCore(motorTask, "MotorTask", 8192, NULL, 1, NULL, 1);  // Runs on core 1

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(httpGetPostTask, "HttpGetPostTask", 8192, NULL, 1, NULL, 0);  // Runs on core 0
}



//int counter = 0;
void loop() {
}


void roverLeftFollow() {
  Serial.print("Rover is doing left follow algorithm");
}

String extractDirection(const String& jsonString) {
  // Create a dynamic JSON buffer
  DynamicJsonDocument doc(128);

  // Deserialize the JSON string
  DeserializationError error = deserializeJson(doc, jsonString);

  // Check if parsing succeeded
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return "";
  }

  // Extract the value of the "Direction" key
  String direction = doc["Direction"].as<String>();

  return direction;
}

// Function to parse JSON and retrieve value of a specified key
String parseJson(const String& json, const String& key) {
  // Create a JSON document
  StaticJsonDocument<200> doc;

  // Parse the received JSON
  DeserializationError error = deserializeJson(doc, json);

  // Check for parsing errors
  if (error) {
    Serial.print("Error parsing JSON: ");
    Serial.println(error.c_str());
    return "";  // Return an empty string if there's an error
  }

  // Retrieve the value of the specified key
  const char* value = doc[key.c_str()];

  // Check if the key exists in the JSON
  if (value) {
    return String(value);  // Return the value as a string
  } else {
    //Serial.println("Key not found in JSON");
    return "null";  // Return an empty string if the key is not found
  }
}

void roverMotion(String direction) {
  if (direction == "Up") {
    s1.start();
    s2.start();
    s1.changeDirection(true);
    s2.changeDirection(false);
    s1.control();
    s2.control();
  } else if (direction == "Down") {
    s1.start();
    s2.start();
    s1.changeDirection(false);
    s2.changeDirection(true);
    s1.control();
    s2.control();
  } else if (direction == "Right") {
    s1.start();
    s2.start();
    s1.changeDirection(true);
    s2.changeDirection(true);
    s1.control();
    s2.control();
  } else if (direction == "Left") {
    s1.start();
    s2.start();
    s1.changeDirection(false);
    s2.changeDirection(false);
    s1.control();
    s2.control();
  } else if (direction == "Stop") {
    s1.stop();
    s2.stop();
    s1.control();
    s2.control();
  }
}
