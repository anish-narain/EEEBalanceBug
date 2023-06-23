#include <AccelStepper.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>
#include <Arduino.h>
#include <MPU6050_light.h>
#include <freertos/task.h>

#define LED_BLUE_OUT 32
#define R_STPR 5
#define R_DIRR 23
#define L_STPR 18
#define L_DIRR 19

AccelStepper r_motor(AccelStepper::DRIVER, R_STPR, R_DIRR);
AccelStepper l_motor(AccelStepper::DRIVER, L_STPR, L_DIRR);

//ANGLE CONTROLLER
float pitch;
volatile float control_rate=0; //control input
float setpoint_pitch = 0; //angle setpoint
float e_pitch, e_pitch_prev=0, e_pitch_integral=0, e_pitch_derivative=0;
float pitch_Kp=60, pitch_Ki=3, pitch_Kd=25; // 1/16 MICROSTEPPING


hw_timer_t *My_timer = NULL;
TaskHandle_t control_task;

MPU6050 mpu(Wire);

void IRAM_ATTR onTimer(){
  digitalWrite(LED_BLUE_OUT, !digitalRead(LED_BLUE_OUT));

    //STEPPING
    l_motor.runSpeed();
    r_motor.runSpeed();
}

void control_thread(void* parameter) {

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 1000, true);
  timerAlarmEnable(My_timer);

  while (1) {

    //READING FROM SENSORS
    mpu.update();

    //STORE RELEVANT READINGS
    pitch = mpu.getAngleY();

    //CALCULATE ERRORS
    e_pitch = pitch - setpoint_pitch;
    e_pitch_integral += e_pitch; //time in ms, T_sampling 1 ms
    e_pitch_derivative = e_pitch - e_pitch_prev;

    //PID controller
    control_rate = (pitch_Kp*e_pitch) + (pitch_Ki*e_pitch_integral) + (pitch_Kd*e_pitch_derivative);
    control_rate = constrain(control_rate, -10000, 10000);

    //SHIFT ERROR SAMPLE
    e_pitch_prev = e_pitch;

    r_motor.setSpeed(control_rate);
    l_motor.setSpeed(control_rate);

    //WATCHDOG DELAY
    vTaskDelay(10);
    
  } 
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BLUE_OUT, OUTPUT);

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
  r_motor.setMaxSpeed(2000);
  r_motor.setAcceleration(2000);
  l_motor.setMaxSpeed(2000);
  l_motor.setAcceleration(2000);

  r_motor.setSpeed(1000);
  r_motor.setSpeed(1000);

  xTaskCreatePinnedToCore(
                    control_thread,   /* Function to implement the task */
                    "control_thread", /* Name of the task */
                    4096,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    1,          /* Priority of the task */
                    &control_task,
                    1);  /* Core where the task should run */  
                    
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
