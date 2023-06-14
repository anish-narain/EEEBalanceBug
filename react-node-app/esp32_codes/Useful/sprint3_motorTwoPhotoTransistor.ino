#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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

float errorSum = 0;
float avgError = 0;

//FOR Phototransistor ====================================================
String direction;
int lightSensorReadingFront = 0;
int lightSensorReadingBack = 0;
//=========================================================================

void setup() {

  Serial.begin(115200);

  s1.init(stepPin1, dirPin1, 8000, true); // left
  s2.init(stepPin2, dirPin2, 8000, true); // right - inverted direction
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

  for (int i = 0; i < 50; i++){
    lightSensorReadingFront = analogRead(32);
    lightSensorReadingBack = analogRead(33); //NEED TO CHANG
    errorSum += lightSensorReadingFront-lightSensorReadingBack;
  }
  
  avgError = errorSum / 50;

  Serial.println(avgError);
  delay(100);

}

char dir = 'q';
float lastTime, Ts;


float sum = 0;
float count = 0;
void loop() {
  lightSensorReadingFront = analogRead(32);
  lightSensorReadingBack = analogRead(33); //NEED TO CHANGE
  //Serial.print("Front: ");
  //Serial.println(lightSensorReadingFront);
  //Serial.print("Back: ");
  //Serial.println(lightSensorReadingBack);

  sum += lightSensorReadingFront-lightSensorReadingBack - avgError;

  count += 1;
  if (count == 50){
    float avg = sum / 50;
    count = 0;
    sum = 0;
    //Serial.println(avg);
    Serial.print(lightSensorReadingFront);
    Serial.print(", ");
    Serial.print(lightSensorReadingBack);
    Serial.print(", ");
    Serial.println(avg);

    if (avg < -100) {
      direction = "Left";
    }else if (avg > 100){
      direction = "Right";
    }
    else {
      direction = "Up";
    }
    
  }
  
  //delay(100);
  


  //Serial.println(direction);


  if (direction == "Up"){
    s1.start();
    s2.start();
    s1.changeDirection(true);
    s2.changeDirection(false);
  }else if (direction == "Left"){
    s1.start();
    s2.start();
    s1.changeDirection(false);
    s2.changeDirection(false);
  }
  
  
  s1.control();
  s2.control();

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
  

  /*if (s1.steps() == 135){
    s1.stop();
    s2.stop();
  }*/

  /*
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
  */

}

