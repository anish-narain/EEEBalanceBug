#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define dirPin1 23
#define stepPin1 14
#define dirPin2 19
#define stepPin2 18
#define stepsPerRevolution 200

#define OUTMAX 400
#define OUTMIN 0

Adafruit_MPU6050 mpu;

hw_timer_t *msTimer = NULL;

float angle;
float accY, accZ, gyroX;
float accYe, accZe, gyroXe;
volatile float gyroRate;
volatile float accAngle, gyroAngle;

float Ts;
float lastTime;
int printCount = 0;

float alpha = 0.99;

// Controller
float targetAngle=0, targetFreq=OUTMAX; // Outputs to control
float delaySpeed;
float error_angle;
float error_angle_sum=0, error_angle_last=0;
float kp_angle=500, ki_angle=0, kd_angle=200;

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

void setup() {

  Serial.begin(115200);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  s1.init(stepPin1, dirPin1, 3000, true);
  s2.init(stepPin2, dirPin2, 3000, true);
  s1.start();
  s2.start();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Starting timer");

  // Initialise angle
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accY = a.acceleration.x;
  accZ = a.acceleration.z;

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  angle = accAngle;

  accYe = a.acceleration.x;
  accZe = a.acceleration.z - 9.81; // Account for gravity
  gyroXe = g.gyro.y;

  Serial.println("Done calibrating");
  delay(1000);
  
}

String input = "";
int gainChange;

void loop() {


  /*while (Serial.available()){
    char c = Serial.read();
    
    if (c == '\n'){

      float g = input.toFloat();
      if (gainChange == 1){
        kp_angle = g;
      }else if (gainChange == 2){
        ki_angle = g;
      }else if (gainChange == 3){
        kd_angle = g;
      }

      Serial.print(kp_angle);
      Serial.print(", ");
      Serial.print(ki_angle);
      Serial.print(", ");
      Serial.println(kd_angle);

      input = "";
      gainChange = -1;
      error_angle_sum=0, error_angle_last=0;
      
    }else if (gainChange == -1){
      
      if (c == 'p'){
        gainChange = 1;
      }else if (c == 'i'){
        gainChange = 2;
      }else if (c == 'd'){
        gainChange = 3;
      }
        
    }else {
      input += c;
    }
    
  }*/
  
  s1.control();
  s2.control();

  if (targetFreq > 0){
    s1.changeSpeed(delaySpeed);
    s2.changeSpeed(delaySpeed);
    s1.changeDirection(true);
    s2.changeDirection(false);
  }else if (targetFreq < 0){
    s1.changeSpeed(-1 * delaySpeed);
    s2.changeSpeed(-1 * delaySpeed);
    s1.changeDirection(false);
    s2.changeDirection(true);
  }

  /*if (angle > 30 || angle < -30){
    s1.stop();
    s2.stop();
  }else{
    s1.start();
    s2.start();
  }*/

  Ts = micros() - lastTime;
  if (Ts > 1000){
    
    lastTime = micros();
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
  
    accY = a.acceleration.x - accYe;
    accZ = a.acceleration.z - accZe;
    gyroX = g.gyro.y - gyroXe;
  
    accAngle = atan2(accY, accZ); //*RAD_TO_DEG;
    //gyroRate = map(gyroX, -32768, 32767, -500, 500);
    gyroRate = gyroX;
    //gyroRate = gyroX*RAD_TO_DEG;
    gyroAngle = (float)gyroRate*(Ts / 1000000); // Angle according to gyro
    angle = alpha*(angle + gyroAngle) + (1 - alpha)*(accAngle);

    error_angle = angle - targetAngle;
    error_angle_sum += error_angle * (Ts / 1000000);

    // constrain error angle sum
    error_angle_sum = constrain(error_angle_sum, -1 * OUTMAX, OUTMAX);

    targetFreq = (kp_angle * error_angle) + (ki_angle * error_angle_sum) + kd_angle*(error_angle - error_angle_last)/(Ts / 1000000);
    targetFreq = constrain(targetFreq, -1 * OUTMAX, OUTMAX);

    delaySpeed = (1/targetFreq) * (1000000);

    error_angle_last = error_angle;
    
    printCount += 1;
    if (printCount == 1000){
      /*
      Serial.print("Angle: ");
      Serial.print(angle);
      Serial.print(", Ts:");
      Serial.print(Ts);
      Serial.print(", Delay:");
      Serial.println(delaySpeed);*/
      printCount = 0;
    }

  }

}
