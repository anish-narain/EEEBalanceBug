#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

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
int coordinates[2] = {10, 20}; //fixed rn, need to change
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
}

char dir = 'q';
float lastTime, Ts;

//int counter = 0;
void loop() {
    HTTPClient httpPOST_roverCoordinates;
    HTTPClient httpGET_nextDirection;

    current_coordinates[0] = coordinates[0];
    current_coordinates[1] = coordinates[1];

    String PostEndpoint_roverCoordinates = "http://" + String(serverAddress) + ":" + String(serverPort) + "/roverCoordinatePost";
    httpPOST_roverCoordinates.begin(PostEndpoint_roverCoordinates);  // Specify the server address, port, and endpoint
    httpPOST_roverCoordinates.addHeader("Content-Type", "application/json");
    String jsonPayload_roverCoordinates = "{\"received_coordinates\":[" + String(current_coordinates[0]) + ", " + String(current_coordinates[1]) + "]}";

    String GetEndpoint_nextDirection = "http://" + String(serverAddress) + ":" + String(serverPort) + "/nextDirection";
    httpGET_nextDirection.begin(GetEndpoint_nextDirection);  // Specify the server address and endpoint

    int httpResponseCodePost_roverCoordinates = httpPOST_roverCoordinates.POST(jsonPayload_roverCoordinates);
    int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();

    httpPOST_roverCoordinates.end();
    httpGET_nextDirection.end();
    /*
    if (counter%1000==0){
      int httpResponseCodePost_roverCoordinates = httpPOST_roverCoordinates.POST(jsonPayload_roverCoordinates);
      int httpResponseCodeGet_nextDirection = httpGET_nextDirection.GET();
    }
    */
    
    //counter++;
    //Movement Code
    if (direction == "Up"){
      Serial.print(direction);
      prev_direction = direction;
      roverMotion("Up");
    }else if (direction == "Down"){
      Serial.print(direction);
      prev_direction = direction;
      roverMotion("Down");
    }else if (direction == "Right"){
      Serial.print(direction);
      prev_direction = direction;
      roverMotion("Right");
    }else if (direction == "Left"){
      Serial.print(direction);
      prev_direction = direction;
      roverMotion("Left");
    }else if (direction == "Stop"){
      Serial.print(direction);
      prev_direction = direction;
      roverMotion("Stop");
    }
    else if (direction == "null"){
      Serial.print(direction);
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
      if (input == 'w'){ 
        x += sin(bearing) * stepD;
        y += cos(bearing) * stepD;
      }
      else if (input == 's'){
        x += -1 * sin(bearing) * stepD;
        y += -1 * cos(bearing) * stepD;
      }
      
    }

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



    delay(0);  // Wait for 0.5 seconds before sending the next request  

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

