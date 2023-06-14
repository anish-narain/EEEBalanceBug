#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <ctype.h>

Adafruit_MPU6050 mpu;
float gyroZe;

#define dirPin1 12
#define stepPin1 14
#define dirPin2 27
#define stepPin2 26
#define stepsPerRevolution 200

//UART STUFF=====================================================================
#define RXD2 16 // FPGA side: (ARDUINO_IO[9]) yellow
#define TXD2 17 // FPGA side: (ARDUINO_IO[8]) 

unsigned long start = 0;
unsigned long period = 500;

unsigned int dist[3] = {0};
bool col_detect = true;
int zoom_level = 1;

int r_state = -1;
int t_flag = 0;

float calibrate_x, calibrate_y; // used for coordinates calibration

void zoom(int zoom_level){
  if (Serial1.availableForWrite() >= 4) {
      byte buf[4]; 
      int2byte(zoom_level, buf);
      Serial1.write(buf, 4);
      Serial.printf("%lu zoom sent\n", zoom_level);
      delay(100);
    }
}

int byte2int(byte* buf, int size) {
  int val=0;

  for (int i=0; i<size; i++) {
    val+= buf[i] << (8*i);
  }

  return val;
}

int byte2int_signed(byte* buf, int size) {
  int val=0;

  for (int i=0; i<(size-1); i++) {
    val += buf[i] << (8*i);
  }

  int MSB = (signed char) buf[size-1];
  val += MSB << (8*(size-1)); 

  return val;
}

void int2byte(int n, byte bytes[4]) {
  for (int i=0; i<4; i++) {
    bytes[i] = (n >> 8*i) & 0xFF;
  }
}


int raw_decode(byte* buf) {

  int i;

  //start condition
  if (byte2int(buf, 4) == 0x00524242) {
    r_state = 0;
  }

  //output
  switch(r_state) {
    case 0:
      break;
    case 1:
      i = byte2int_signed(buf+3, 1);
      if (i == -1) {
        Serial.println("no beacons");
      } else {
        dist[i] = byte2int(buf, 3);
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
  Serial.printf("beacon_x = %d\n", centre[0]);
}

//MOTOR STUFF=====================================================================
#define wheelD 6 // Wheel diameter in cm
float stepD = (3.1415 * wheelD / stepsPerRevolution); // Distance travelled per step

float x, y, bearing;
bool stepFlag; // High whenever a step occurs

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

void calibration_coor(float gyroZe) {
  float targetAngle = 360.0; // Target angle for the turn
  float currentAngle = 0.0; // Current angle of the turn
  float lastTime = micros(); // Last time recorded

  // dimension of the arena
  int height = 500;
  int width = 350;
  
  int sum_dist[3] = {0};    // red, blue, yellow
  int count[3] = {0};
  int avg_dist[3] = {0};

  // zoom in the camera for calibration
  zoom(3);

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

    // Adjust the current angle to be between 0 and 360 degrees
    if (bearing < 0.0) {
      bearing += 360.0;
    } else if (bearing >= 360.0) {
      bearing -= 360.0;
    }

    // Update the current angle of the turn
    currentAngle += abs((float)g.gyro.z - gyroZe) * (Ts / 1000000);

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

  // calculate average distance of all three
  for (int i=0; i<3; i++){
      avg_dist[i] = sum_dist[i] / count[i];
    }

  // zoom out the camera
  zoom(1);
  
  // calculate x and y coordinates
  float beta = acos((pow(avg_dist[0],2)+pow(width,2)-pow(avg_dist[2],2)/(2*avg_dist[0]*width)));
  float x1 = avg_dist[0]*cos(beta);
  float y1 = height - avg_dist[0]*sin(beta);

  float gamma = acos((pow(avg_dist[2],2)+pow(height,2)-pow(avg_dist[1],2)/(2*avg_dist[2]*width)));
  float x2 = width - avg_dist[2]*sin(gamma);
  float y2 = height - avg_dist[2]*cos(gamma);

  calibrate_x = (x1+x2)/2;
  calibrate_y = (y1+y2)/2;
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2); //FPGA

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

}



char dir = 'q';
float lastTime, Ts;

void loop() {

  //RECEIVING
  if (Serial1.available() >= 4){
    byte buf[4];
    Serial1.readBytes(buf, 4);
    raw_decode(buf);
    //metrics2string();
    int hex = byte2int(buf, 4);
    Serial.printf("%x\n", hex);
  }

  //SIM SERVER INPUT
  unsigned long current  = millis();
  if (current - prev == period) {
    Serial.printf("%lu zoom request\n", current);
    prev = current;
    zoom_level = 2;
    t_flag = 1;
  }

  //SENDING
  if (t_flag == 1) {
    if (Serial1.availableForWrite() >= 4) {
      byte buf[4]; 
      int2byte(zoom_level, buf);
      Serial1.write(buf, 4);
      t_flag = 0;
      //Serial.printf("%lu zoom sent\n", current);
    }
  }

  if (col_detect == false){
    s1.start();
    s2.start();
    s1.changeDirection(true);
    s2.changeDirection(false);
    s1.control();
    s2.control();
  }else if (col_detect == true){
    /*
    //stop it
    s1.stop();
    s2.stop();
    s1.control();
    s2.control();
    */
    //move left
    s1.start();
    s2.start();
    s1.changeDirection(false);
    s2.changeDirection(false);
    s1.control();
    s2.control();
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
  /*
  if (printCount == 1000){
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.print(y);
    Serial.print(", Dir: ");
    Serial.println(bearing);
    printCount = 0;
  }
  */

}
