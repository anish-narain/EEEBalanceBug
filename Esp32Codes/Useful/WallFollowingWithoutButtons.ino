#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <freertos/task.h>

Adafruit_MPU6050 mpu;
float gyroZe;

#define dirPin1 12
#define stepPin1 14
#define dirPin2 27
#define stepPin2 26
#define stepsPerRevolution 200

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
float calibrate_x, calibrate_y;  // used for coordinates calibration


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
        Serial.println("no beacons");
        for (int j=0; j<3; j++){
            dist[j] = 0;
        } 
      } else {
        for (int j=0; j<3; j++){
          if (i==j){
            dist[j] = byte2int(buf, 3);
            Serial.printf("dist to beacon %d = %d mm\n", i, dist[i]);
          }
          else{
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
  Serial.printf("dist_red = %d\n", dist[0]);
  Serial.printf("dist_yellow = %d\n", dist[1]);
  Serial.printf("dist_blue = %d\n", dist[2]);

  Serial.printf("col_detect = %d\n", col_detect);
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
String left_following;
String wall_detection = "true";  //fixed rn, need to change
String maze_complete = "false";  //fixed rn, need to change
String mvmt_direction;
String server_direction = "Up";
String prev_direction;  //need this for null inputs
String recalibrateFlag = "false";

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
  float targetAngle = 6.28; // Target angle for the turn
  float currentAngle = 0.0; // Current angle of the turn
  float lastTime = micros(); // Last time recorded

 

  // dimension of the arena
  int height = 340;
  int width = 225;

  int sum_dist[3] = {0};    // red, blue, yellow
  int count[3] = {0};
  int avg_dist[3] = {0};

 

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
    s1.changeDirection(false); // Rotate counter-clockwise
    s2.changeDirection(false); // Rotate counter-clockwise
    s1.control();
    s2.control();

 

    // Update the current angle based on the gyro reading (not sure if we still need to update current angle)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float Ts = micros() - lastTime; // Time since last loop
    lastTime = micros();
    bearing += ((float)g.gyro.z - gyroZe) * (Ts / 1000000); // Integrate angular velocity with respect to time

 

    // Update the current angle of the turn
    currentAngle += abs((float)g.gyro.z - gyroZe) * (Ts / 1000000);
    Serial.println(currentAngle);

    // Call the raw_decode function to update the dist array
    if (Serial1.available() >= 4){
      //Serial.print("MSG RECEIVED");
      byte buf[4];
      Serial1.readBytes(buf, 4);
      int a = byte2int(buf, 4);
      Serial.println(a, HEX);
      raw_decode(buf);
      //metrics2string();
      }

    // print each beacons distance each cycle
    for (int i = 0; i<3; i++){
      Serial.print(dist[i]);
      Serial.print(" ");
    }
    Serial.println("");

    // sum up all positive distance readings
    for (int i=0; i<3; i++){
      if (dist[i] > 0){
        sum_dist[i] += dist[i];
        count[i] += 1;
      }
    }

    // Delay to control the speed of the turn
    delay(10); // Adjust the delay value to control the speed of the turn
  }

 

  // Stop the motors after completing the turn
  s1.stop();
  s2.stop();
  s1.control();
  s2.control();

 

  // zoom out the camera
  zoom(1);

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

  if ((count[0]!=0 & count[2]!=0)){
    
    avg_dist[0] = sum_dist[0] / count[0];
    avg_dist[2] = sum_dist[2] / count[2];

    for (int i = 0; i<3; i++){
     Serial.print("avg");
      Serial.print(avg_dist[i]);
      Serial.print(" ");
    }
    Serial.println("");

    // calculate x and y coordinates
    float beta = acos((pow(avg_dist[0],2)+pow(width,2)-pow(avg_dist[2],2))/(2*avg_dist[0]*width));
    float x1 = avg_dist[0]*cos(beta);
    float y1 = height - avg_dist[0]*sin(beta);

    Serial.println(beta);
    Serial.println(x1);
    Serial.println(y1);

    calibrate_x = x1;
    calibrate_y = y1;

    Serial.printf("calibrate_x %x\n", calibrate_x); 
    Serial.printf("calibrate_x %x\n", calibrate_y); 
  }
  else if ((count[1]!=0 & count[2]!=0)){
    avg_dist[1] = sum_dist[1] / count[1];
    avg_dist[2] = sum_dist[2] / count[2];

    float gamma = acos((pow(avg_dist[2],2)+pow(height,2)-pow(avg_dist[1],2))/(2*avg_dist[2]*width));
    float x2 = width - avg_dist[2]*sin(gamma);
    float y2 = height - avg_dist[2]*cos(gamma);

    calibrate_x = x2;
    calibrate_y = y2;

    Serial.printf("calibrate_x %x\n", calibrate_x); 
    Serial.printf("calibrate_x %x\n", calibrate_y); 
  }
  else if ((count[1]!=0 & count[2]!=0 & count[0]!=0)){
    for (int i=0; i<3; i++){
      avg_dist[i] = sum_dist[i] / count[i];
    }
    // calculate x and y coordinates
    float beta = acos((pow(avg_dist[0],2)+pow(width,2)-pow(avg_dist[2],2))/(2*avg_dist[0]*width));
    float x1 = avg_dist[0]*cos(beta);
    float y1 = height - avg_dist[0]*sin(beta);

    float gamma = acos((pow(avg_dist[2],2)+pow(height,2)-pow(avg_dist[1],2))/(2*avg_dist[2]*width));
    float x2 = width - avg_dist[2]*sin(gamma);
    float y2 = height - avg_dist[2]*cos(gamma);

    calibrate_x = (x1+x2)/2;
    calibrate_y = (y1+y2)/2;

    Serial.printf("calibrate_x %x\n", calibrate_x); 
    Serial.printf("calibrate_x %x\n", calibrate_y); 
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


void httpGetPostTask(void* parameter) {
  while (1) {
    //GET AND POST httpclient setup =================================================
    HTTPClient httpPOST_roverCoordinates;
    HTTPClient httpPOST_wallDetection;
    HTTPClient httpGET_nextDirection;
    HTTPClient httpGET_recalibrate;

    //POST ENDPOINTS Setup ==========================================================
    current_coordinates[0] = coordinates[0];
    current_coordinates[1] = coordinates[1];

    String PostEndpoint_roverCoordinates = "http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinatePost";
    httpPOST_roverCoordinates.begin(PostEndpoint_roverCoordinates);  // Specify the server address, port, and endpoint
    httpPOST_roverCoordinates.addHeader("Content-Type", "application/json");
    String jsonPayload_roverCoordinates = "{\"received_coordinates\":[" + String(current_coordinates[0]) + ", " + String(current_coordinates[1]) + "]}";

    String PostEndpoint_wallDetection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/wallDetection";
    httpPOST_wallDetection.begin(PostEndpoint_wallDetection);  // Specify the server address, port, and endpoint
    httpPOST_wallDetection.addHeader("Content-Type", "application/json");
    String jsonPayload_wallDetection = "{\"received_walldetection\":" + wall_detection + "}";

    //GET ENDPOINTS Setup ============================================================
    String GetEndpoint_nextDirection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/nextDirection";
    httpGET_nextDirection.begin(GetEndpoint_nextDirection);  // Specify the server address and endpoint

    String GetEndpoint_recalibrate = "http://" + String(serverAddress) + ":" + String(serverPort) + "/recalibrate";
    httpGET_recalibrate.begin(GetEndpoint_recalibrate);  // Specify the server address and endpoint


    //roverCoordinates POST Code =====================================================================
    int httpResponseCodePost_roverCoordinates = httpPOST_roverCoordinates.POST(jsonPayload_roverCoordinates);

    if (httpResponseCodePost_roverCoordinates > 0) {
      //Serial.print("Button click request sent. Response code for roverCoordinates : ");
      //Serial.println(httpResponseCodePost_roverCoordinates);

      String responseBody_roverCoordinates = httpPOST_roverCoordinates.getString();
      //Serial.print("Response body for roverCoordinates: ");
      //Serial.println(responseBody_roverCoordinates);
    } else {
      //Serial.print("Error sending request for rover coordinates. Error code: ");
      //Serial.println(httpResponseCodePost_roverCoordinates);
    }

    //wallDetection POST Code =====================================================================
    int httpResponseCodePost_wallDetection = httpPOST_wallDetection.POST(jsonPayload_wallDetection);

    if (httpResponseCodePost_wallDetection > 0) {
      //Serial.print("Button click request sent. Response code for wallDetection : ");
      //Serial.println(httpResponseCodePost_wallDetection);

      String responseBody_wallDetection = httpPOST_wallDetection.getString();
      //Serial.print("Response body for wallDetection: ");
      //Serial.println(responseBody_wallDetection);
    } else {
      //Serial.print("Error sending request for wall detection. Error code: ");
      //Serial.println(httpResponseCodePost_wallDetection);
    }

    if (wall_detection == "true") {
      //mvmt_direction = GET nextDirection ================================================================
      int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();

      if (httpResponseCodeGet_nextDirection > 0) {
        //Serial.print("HTTP Response code: ");
        //Serial.println(httpResponseCodeGet_nextDirection);
        mvmt_direction = httpGET_nextDirection.getString();
        server_direction = extractDirection(mvmt_direction);
      } else {
        //Serial.print("Error code for next direction: ");
        //Serial.println(httpResponseCodeGet_nextDirection);
      }
    }

    if (mvmt_direction == "doleftwall") {
      start_coordinates[0] = current_coordinates[0];
      start_coordinates[1] = current_coordinates[1];
      delay(100);  //let the current coordinates change
      while (start_coordinates != current_coordinates) {
        roverLeftFollow();
      }
      mvmt_direction = "";
    } else {
      //move in mvmt_direction
    }

    //recalibrateFlag = GET recalibrate ================================================================
    int httpResponseCodeGet_recalibrate = httpGET_recalibrate.GET();

    if (httpResponseCodeGet_recalibrate > 0) {
      //Serial.print("HTTP Response code for recalibration: ");
      //Serial.println(httpResponseCodeGet_recalibrate);
      String receivedRecalibration = httpGET_recalibrate.getString();
      recalibrateFlag = parseJson(receivedRecalibration, "Recalibrate");
      //Serial.print("Recalibrate Flag: ");
      //Serial.println(recalibrateFlag);
      if (recalibrateFlag == "true"){
        Serial.println(recalibrateFlag);
      }
    } else {
      Serial.print("Error code for recalibrate: ");
      Serial.println(httpResponseCodeGet_recalibrate);
    }

    // Free resources
    httpPOST_roverCoordinates.end();
    httpPOST_wallDetection.end();
    httpGET_nextDirection.end();
    httpGET_recalibrate.end();

    //delay(500);  // Wait for 0.5 seconds before sending the next request
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

void motorTask(void* parameter) {
  while (1) {
    
    if (Serial1.available() >= 4) {
      byte buf[4];
      Serial1.readBytes(buf, 4);
      raw_decode(buf);
      //int a = byte2int(buf,4);
      //Serial.println(a, HEX);
    }
    
    /*
    //RECALIBRATION CODE
    if (recalibrateFlag == "true") {
      calibration_coor(gyroZe);
    }
    */
    lightSensorReadingFront = analogRead(32);
    lightSensorReadingBack = analogRead(33);
    lightSensorReadingTop = analogRead(35);

    sum += lightSensorReadingFront - lightSensorReadingBack - avgError;
    f += lightSensorReadingFront - avgF;
    b += lightSensorReadingBack - avgB;
    t += lightSensorReadingTop - avgT;

    count += 1;
    if (count == 50) {
      float avg = sum / 50;
      float aF = f / 50;
      float aB = b / 50;
      float aT = t / 50;
      count = 0;
      sum = 0;
      f = 0;
      b = 0;
      t = 0;
      //Serial.println(avg);
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
      Serial.println(col_detect);

      aF = aF - aT;  // Remove ambient light

      if (col_detect == true) {
        if (canChangeDir) {
          if (aF > -150) {
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


      } else {  // No wall in front
        wallCount++;
        if (wallCount > 10) {
          canChangeDir = true;
        }

        if (aF < -100 && canChangeDir && blockLeft == 0) {
          direction = "Left";
        } else if (aF > 150 && canChangeDir) {
          direction = "Right";
        } else if (aF > 300 && !canChangeDir && wallDir == 1) {
          direction = "Right";
        } else {
          direction = "Up";
        }

        if (blockLeft > 0) {
          blockLeft -= 1;
          blockLeftDir -= 1;

          if (blockLeftDir > 0) {
            direction = "Right";
          } else {
            direction = "Up";
          }
        }
        if (avg < -400) {
          Serial.println("Blocked left");
          blockLeft = 6;
          blockLeftDir = 3;
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
      s1.control();
      s2.control();
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

      Serial.print("X: ");
      Serial.print(x);
      Serial.print(", Y: ");
      Serial.print(y);
      Serial.print(", Dir: ");
      Serial.println(bearing);

      printCount = 0;
    }


    //delay(0);  // Wait for 0.5 seconds before sending the next request
  }
}



void setup() {
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

  calibration_coor(gyroZe);


  for (int i = 0; i < 50; i++) {
    lightSensorReadingFront = analogRead(32);
    lightSensorReadingBack = analogRead(33);
    lightSensorReadingTop = analogRead(35);
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


  //xTaskCreatePinnedToCore(motorTask, "MotorTask", 8192, NULL, 1, NULL, 1);  // Runs on core 1

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
