#include <AccelStepper.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <Arduino.h>
#include <MPU6050_light.h>
#include <freertos/task.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8])

#define R_STPR 5
#define R_DIRR 23
#define L_STPR 18
#define L_DIRR 19
#define MICROSTEPS /*3200*/ /*SIXTEENTH-STEPPING*/ 800 //QUARTER-STEPPING
#define CIRCUMFERENCE 205 //distance per revolution in mm
#define OUTMAX 1000
#define OUTMIN 0

//STEPPER
AccelStepper r_motor(AccelStepper::DRIVER, R_STPR, R_DIRR);
AccelStepper l_motor(AccelStepper::DRIVER, L_STPR, L_DIRR);

//SAMPLING TIMER
unsigned long prev = 0;
unsigned long t = 0;
unsigned long T_sampling = 1000;

//SENSORS
MPU6050 mpu(Wire);
float pitch;

//CONTROLLER
volatile float control_speed=0; //control input
float setpoint_pitch = 0; //angle setpoint
float e_pitch, e_pitch_prev=0, e_integral=0, e_derivative=0;
float Kp=300, Ki=50, Kd=0; // 1/8 MICROSTEPPING
//float Kp=1000, Ki=500, Kd=0; //1/16 MICROSTEPPING

float revs2steps(float revs) {
  return MICROSTEPS*revs;
}

//TESTING
int state = 0;
unsigned long ttimer;
unsigned long tprev = 0;
unsigned long tperiod = 1000;

//INTERRUPT
hw_timer_t *step_timer = NULL;
void IRAM_ATTR ISR_step(){
  //STEPPING
  l_motor.runSpeed();
  r_motor.runSpeed();
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  //MPU INITIALISATION
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  
  //MOTOR INITIALISATION
  l_motor.setMaxSpeed(revs2steps(20) /*8000*/);
  l_motor.setAcceleration(revs2steps(20) /*8000*/);
  r_motor.setMaxSpeed(revs2steps(20) /*8000*/);
  r_motor.setAcceleration(revs2steps(20) /*8000*/);

  //TIMER & INTERRUPTS
  step_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(step_timer, &ISR_step, true);
  timerAlarmWrite(step_timer, 10000, true);
  timerAlarmEnable(step_timer);
}

void loop() {

    //TIMER REGULATES SAMPLING FREQUENCY
    t = micros() - prev;
    if (t > T_sampling){
      prev = t;

      //READING FROM SENSORS
      mpu.update();
      
      //STORE RELEVANT READINGS
      pitch = mpu.getAngleY();

      //CALCULATE ERRORS
      e_pitch = pitch - setpoint_pitch;
      e_integral += e_pitch; //time in ms, T_sampling = 1ms = 1000micros
      e_derivative = e_pitch - e_pitch_prev;

      //PID controller
      control_speed = (Kp*e_pitch) + (Ki*e_integral) + (Kd*e_derivative);
      control_speed = constrain(control_speed, /*-8000*/ -revs2steps(20), /*8000*/ revs2steps(20));
      control_speed = (abs(control_speed) < 100) ? 0 : control_speed;

      //SHIFT ERROR SAMPLE
      e_pitch_prev = e_pitch;
    }

    l_motor.setSpeed(/*control_speed*/ state*500);
    r_motor.setSpeed(/*control_speed*/ state*500);

    //STEPPING
    l_motor.runSpeed();
    r_motor.runSpeed();
}
