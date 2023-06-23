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

#define wheelD 6 // Wheel diameter in cm
float stepD = (3.1415 * wheelD / stepsPerRevolution); // Distance travelled per step
float x, y, bearing;
bool stepFlag; // High whenever a step occurs

const char* ssid = "Rover27";
const char* password = "123456788";
const char* serverAddress = "18.134.98.192";  // Replace with the IP address or hostname of your Node.js server
const int serverPort = 3001;  // Replace with the port number your Node.js server is listening on

//Variable declarations for traversal algorithm
int coordinates[2];
int start_coordinates[2];
int current_coordinates[2];
String left_following;
String wall_detection = "false"; //fixed rn, need to change
String maze_complete = "false"; //fixed rn, need to change
String mvmt_direction;
String direction = "Up";
String prev_direction; //need this for null inputs


//Function declarations for traversal algorithm
void roverLeftFollow();
String extractDirection(const String& jsonString);
void roverMotion(String direction);

class stepperMotor{
  public:
     
  void stop(void){
    enable = 0;
  }
  
  void start(void){
    enable = 1;
  }

  void init(int _pulsePin, int _dirPin, unsigned long _delayTime, bool _direction){
    pulsePin      = _pulsePin;
    dirPin        = _dirPin;
    delayTime     = _delayTime;
    dir     = _direction;
      
    togglePulse   = LOW;
    enable        = 0;
      
    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin, OUTPUT);
  }

  void control(void){
    
    currentTime = micros();
    digitalWrite(dirPin, dir);
    if(enable == 1){
      if( (currentTime - deltaTime) > delayTime ){
    
        pulseCount++;
 
        // Each HIGH or LOW is a "pulse"
        // But each step of the motor requires two "pulses"
        if(pulseCount % 2 == 0){
          stepCount++;
          stepFlag = true;
        }
  
        togglePulse = togglePulse == LOW ? HIGH : LOW;
        digitalWrite(pulsePin, togglePulse);
        deltaTime = currentTime;
      }
    }
  }

  void changeDirection(bool _direction){
    dir = _direction;
  }

  unsigned long steps(void){
    return stepCount;
  }
  
  void changeSpeed(unsigned long _speed){
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

int printCount;
int input; // Serial input data

void httpGetPostTask(void* parameter) {
  while (1) {
    //GET AND POST httpclient setup =================================================
    HTTPClient httpPOST_roverCoordinates;
    HTTPClient httpPOST_wallDetection;
    HTTPClient httpGET_newWall;
    HTTPClient httpGET_nextDirection;
    HTTPClient httpGET_mvmtStop;
    HTTPClient httpGET_mazeComplete;
    
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
    String GetEndpoint_newWall = "http://" + String(serverAddress) + ":" + String(serverPort) + "/newWall";
    httpGET_newWall.begin(GetEndpoint_newWall);  // Specify the server address and endpoint
    
    String GetEndpoint_nextDirection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/nextDirection";
    httpGET_nextDirection.begin(GetEndpoint_nextDirection);  // Specify the server address and endpoint
    
    String GetEndpoint_mvmtStop = "http://" + String(serverAddress) + ":" + String(serverPort) + "/mvmtStop";
    httpGET_mvmtStop.begin(GetEndpoint_mvmtStop);  // Specify the server address and endpoint
    
    String GetEndpoint_mazeComplete = "http://" + String(serverAddress) + ":" + String(serverPort) + "/mazeComplete";
    httpGET_mazeComplete.begin(GetEndpoint_mazeComplete);  // Specify the server address and endpoint
    
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
     // Serial.print("Button click request sent. Response code for wallDetection : ");
      // Serial.println(httpResponseCodePost_wallDetection);
      
      String responseBody_wallDetection = httpPOST_wallDetection.getString();
      //Serial.print("Response body for wallDetection: ");
      //Serial.println(responseBody_wallDetection);
    } else {
      //Serial.print("Error sending request for wall detection. Error code: ");
      //Serial.println(httpResponseCodePost_wallDetection);
    }

    if (wall_detection == "true"){
      //left_following = GET newWall ================================================================
      int httpResponseCodeGet_newWall = httpGET_newWall.GET();
        
      if (httpResponseCodeGet_newWall>0) {
        //Serial.print("HTTP Response code: ");
        //Serial.println(httpResponseCodeGet_newWall);
        left_following = httpGET_newWall.getString();
        //Serial.println(left_following);
      }
      else {
        //Serial.print("Error code: ");
       // Serial.println(httpResponseCodeGet_newWall);
      }
    }

    if (left_following == "true"){
      start_coordinates[0] = current_coordinates[0];
      start_coordinates[1] = current_coordinates[1];
      delay(100); //let the current coordinates change
      while(start_coordinates != current_coordinates){
        roverLeftFollow();
      }
      left_following == "false";

    }
    else {
      //mvmt_direction = GET nextDirection================================================================
      int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();
        
      if (httpResponseCodeGet_nextDirection>0) {
       // Serial.print("HTTP Response code: ");
       // Serial.println(httpResponseCodeGet_nextDirection);
        mvmt_direction = httpGET_nextDirection.getString();
        direction = extractDirection(mvmt_direction);
      }
      else {
        //Serial.print("Error code for next direction: ");
        //Serial.println(httpResponseCodeGet_nextDirection);
      }
    }
    // Free resources
    httpPOST_roverCoordinates.end();
    httpPOST_wallDetection.end();
    httpGET_newWall.end();
    httpGET_nextDirection.end();
    httpGET_mvmtStop.end();
    httpGET_mazeComplete.end();

    //delay(500);  // Wait for 0.5 seconds before sending the next request  
  }
}

char dir = 'q';
float lastTime, Ts;

void motorTask(void* parameter){
  while(1) {
    //Movement Code
    if (direction == "Up"){
      //Serial.print(direction);
      prev_direction = direction;
      roverMotion("Up");
    }else if (direction == "Down"){
     // Serial.print(direction);
      prev_direction = direction;
      roverMotion("Down");
    }else if (direction == "Right"){
     // Serial.print(direction);
      prev_direction = direction;
      roverMotion("Right");
    }else if (direction == "Left"){
    //  Serial.print(direction);
      prev_direction = direction;
      roverMotion("Left");
    }else if (direction == "Stop"){
    //  Serial.print(direction);
      prev_direction = direction;
      roverMotion("Stop");
    }
    else if (direction == "null"){
    //  Serial.print(direction);
      roverMotion(prev_direction);
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); // Get sensor readings
    Ts = micros() - lastTime; // Time since last loop
    lastTime = micros();
    bearing += ((float)g.gyro.z - gyroZe) * (Ts/1000000); // Integrates angular velocity with respect to time

    if (stepFlag){
      stepFlag = false;

      // Increment co-ordinates based on direction
      if (prev_direction == "Up"){ 
        x += sin(bearing) * stepD;
        y += cos(bearing) * stepD;
      }
      else if (prev_direction == "Down"){
        x += -1 * sin(bearing) * stepD;
        y += -1 * cos(bearing) * stepD;
      }
      
    }

    coordinates[0] = x;
    coordinates[1] = y;

    printCount++;
    if (printCount == 1000){
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
  Serial.begin(9600);

  s1.init(stepPin1, dirPin1, 5000, true); // left
  s2.init(stepPin2, dirPin2, 5000, true); // right - inverted direction
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
  Serial.println("Calibrating"); // Do not move whilst calibrating
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroZe = g.gyro.z;
  Serial.println("Done");

  Serial.println("");
  delay(100);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Create tasks for HTTP GET/POST and motor control
  xTaskCreatePinnedToCore(httpGetPostTask, "HttpGetPostTask", 8192, NULL, 1, NULL, 0);  // Runs on core 0
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 8192, NULL, 1, NULL, 1);  // Runs on core 1
}



//int counter = 0;
void loop() {
    
    /*
    if (counter%1000==0){
      int httpResponseCodePost_roverCoordinates = httpPOST_roverCoordinates.POST(jsonPayload_roverCoordinates);
      int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();
    }
    */
    
    //counter++;
    
  }


void roverLeftFollow(){
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

void roverMotion(String direction){
  if (direction == "Up"){
      s1.start();
      s2.start();
      s1.changeDirection(true);
      s2.changeDirection(false);
      s1.control();
      s2.control();
    }else if (direction == "Down"){
      s1.start();
      s2.start();
      s1.changeDirection(false);
      s2.changeDirection(true);
      s1.control();
      s2.control();
    }else if (direction == "Right"){
      s1.start();
      s2.start();
      s1.changeDirection(true);
      s2.changeDirection(true);
      s1.control();
      s2.control();
    }else if (direction == "Left"){
      s1.start();
      s2.start();
      s1.changeDirection(false);
      s2.changeDirection(false);
      s1.control();
      s2.control();
    }else if (direction == "Stop"){
      s1.stop();
      s2.stop();
      s1.control();
      s2.control();
    }
}
