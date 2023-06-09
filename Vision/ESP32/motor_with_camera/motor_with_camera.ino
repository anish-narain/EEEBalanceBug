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
int centre[2] = {0};
bool col_detect = false;

int state = -1;

int byte2int(byte* buf, int size) {
  int val=0;

  for (int i=(size-1); i>=0; i--) {
    val += buf[i] << (8*i);
  }

  return val;
}

int byte2int_signed(byte* buf, int size) {
  int val=0;

  for (int i=(size-1); i>=0; i--) {
    val += buf[i] << (8*i);
  }

  return val;
}

int raw_decode(byte* buf) {

  //start condition
  if (byte2int(buf, 4) == 0x00524242) {
    state = 0;
  }

  //output
  switch(state) {
    case 0:
      break;
    case 1: case 2: case 3:
      dist[state-1] = byte2int(buf, 4);
      break;
    case 4:
      col_detect = (bool)byte2int_signed(buf, 4);
      break;
    case 5:
      for (int i=0; i<2; i++) {
        centre[i] = byte2int(buf+(i*2), 2);
      }
      break;
    default:
      break;
  }

  //next state
  if (state > -1 & state < 5) {
    state++;
  } else if (state == 5) {
    state = -1;
  }

  return 0;
}

void metrics2string() {
  Serial.printf("dist_red = %d\n", dist[0]);
  Serial.printf("dist_yellow = %d\n", dist[1]);
  Serial.printf("dist_blue = %d\n", dist[2]);

  Serial.printf("col_detect = %d\n", col_detect);
  Serial.printf("beacon_x = %d\n", centre[0]);
  Serial.printf("beacon_x to centre = %d\n", centre[1]);
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
  //Serial.printf("cA");
  if (Serial1.available() >= 4){
    byte buf[4];
    Serial1.readBytes(buf, 4);
    raw_decode(buf);
    //metrics2string();
    Serial.printf("B");
    Serial.printf("collision = %d\n", col_detect);
    
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

