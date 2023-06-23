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
#define LED_BLUE_OUT 32
#define LED_GREEN_OUT 33
#define SIXTEENTH_MICROSTEPS 3200 //SIXTEENTH-STEPPING
#define QUARTER_MICROSTEPS 800 //QUARTER-STEPPING
#define CIRCUMFERENCE 205 //distance per revolution in mm

TaskHandle_t control_task;
TaskHandle_t stepper_task;

//TIMER
unsigned long prev, test_prev;
unsigned long T_move = 2000 * 1000; //500ms
unsigned long T_idle = 3000 * 1000; //1s
int flag = 0;
//float move_accel = 0.1;
float move_pitch = 3;

int control_flag = 0;

//STEPPER
AccelStepper r_motor(AccelStepper::DRIVER, R_STPR, R_DIRR);
AccelStepper l_motor(AccelStepper::DRIVER, L_STPR, L_DIRR);

//SENSORS
MPU6050 mpu(Wire);
int idle = 1;
float accel = 0;
float pitch = 0;

class Timer {
  private:
    unsigned long prev;
    unsigned long T;
  
  public:
    Timer(unsigned long _T) {
      prev = esp_timer_get_time();
      T = _T;
    }

    int check() {
      unsigned long t = esp_timer_get_time();
      if ((t - prev) > T) {
        prev = t;
        return 1;
      } else return 0;
    }

    unsigned long getT() {
      return T;
    }

    void rst() {
      prev = esp_timer_get_time();
    }

};

Timer ramp_timer(100000);
float step;

//ANGLE CONTROLLER
volatile float control_rate=0; //control input
float setpoint_pitch = 0, desired_pitch = 0; //angle setpoint
float e_pitch, e_pitch_prev=0, e_pitch_integral=0, e_pitch_derivative=0;
float pitch_Kp=1000, pitch_Ki=50, pitch_Kd=400; // 1/16 MICROSTEPPING

//ACCELERATION CONTROLLER
volatile float control_pitch=0; //control input
float setpoint_accel = 0; //desired_accel = 0; //speedsetpoint 
float e_accel, e_accel_prev=0, e_accel_integral=0, e_accel_derivative=0;
float accel_Kp=5, accel_Ki=0, accel_Kd=0; // 1/16 MICROSTEPPING

float revs2steps(float revs) {
  return QUARTER_MICROSTEPS*revs;
}

void ramp(Timer timer, float& set, float& current, float step) {
  if (timer.check()) {
    if (current < set) current+=step;
    else if (current > set) current-=step;
  }
}

float calc_step(unsigned long T_total, unsigned long T_step, float set0, float set1) {
  return abs((set1-set0)*T_step/T_total);

}

void stepper_thread(void* parameter) {
  while(1) {
    //CONTROL
    l_motor.setSpeed(control_rate);
    r_motor.setSpeed(control_rate);


    //STEPPING
    l_motor.runSpeed();
    r_motor.runSpeed();

    //WATCHDOG DELAY
    vTaskDelay(1);
  }
}

void control_thread(void* parameter) {
  while (1) {

    //SET DESIRED PITCH
    unsigned long t = esp_timer_get_time();
    if (!idle & ((t - prev) > T_move)) {
      prev = t;
      idle = 1;
      //desired_accel = 0;
      
      step = calc_step(10000000, 100000, desired_pitch, 0);
      //Serial.println(step);
      desired_pitch = 0;

      digitalWrite(LED_BLUE_OUT, 0);
    } else if (idle & ((t - prev) > T_idle)) {
      prev = t;
      idle = 0;

      //move_accel = -move_accel;
      //desired_accel = move_accel;

      move_pitch = -move_pitch;
      step = calc_step(10000000, 100000, desired_pitch, move_pitch);
      //Serial.println(step);
      desired_pitch = move_pitch;

      digitalWrite(LED_BLUE_OUT, 1);
    }

    //RAMP LINEARLY TO SETPOINT_PITCH -> DESIRED_SETPOINT
    ramp(ramp_timer, desired_pitch, setpoint_pitch, step);
    if ((setpoint_pitch > desired_pitch - 0.01) && (setpoint_pitch < desired_pitch +0.01)) digitalWrite(LED_GREEN_OUT, 1);
    else digitalWrite(LED_GREEN_OUT, 0);

    //READING FROM SENSORS
    mpu.update();

    //STORE RELEVANT READINGS
    pitch = mpu.getAngleY();
    float accX = mpu.getAccX();

    accel = pow(pow(accX, 2) + pow(mpu.getAccZ(), 2), 0.5) - 1;
    accel = (accX < 0) ? -accel : accel;

    // if ((t - test_prev) > 200000) {
    //   test_prev = t;
    //   Serial.printf("%f, %f\n", setpoint_pitch, step);
    // }

    //CALCULATE ERRORS
    e_accel = accel - setpoint_accel;
    e_accel_integral += e_accel;
    e_accel_derivative = e_accel - e_accel_prev;

    //PID Controller
    control_pitch = (accel_Kp*e_accel) + (accel_Ki*e_accel_integral) + (accel_Kd*e_accel_derivative);
    control_pitch = constrain(control_pitch, -5, 5);
    
    //CASCADED CONTROL
    //setpoint_pitch = /*idle ? 0 :*/ control_pitch;

    //CALCULATE ERRORS
    e_pitch = pitch - setpoint_pitch;
    e_pitch_integral += e_pitch; //time in ms, T_sampling 1 ms
    e_pitch_derivative = e_pitch - e_pitch_prev;

    //PID controller
    control_rate = (pitch_Kp*e_pitch) + (pitch_Ki*e_pitch_integral) + (pitch_Kd*e_pitch_derivative);
    control_rate = constrain(control_rate, -10000, 10000);

    //SHIFT ERROR SAMPLE
    e_accel_prev = e_accel;
    e_pitch_prev = e_pitch;

    //Serial.println(control_rate);

    //WATCHDOG DELAY
    vTaskDelay(10);
    
  }
  
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BLUE_OUT, OUTPUT);
  pinMode(LED_GREEN_OUT, OUTPUT);

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
  l_motor.setMaxSpeed(32000);
  l_motor.setAcceleration(16000);
  r_motor.setMaxSpeed(32000);
  r_motor.setAcceleration(16000);

  l_motor.setSpeed(10);
  r_motor.setSpeed(10);
  
  // xTaskCreatePinnedToCore(
  //                   stepper_thread,   /* Function to implement the task */
  //                   "stepper_thread", /* Name of the task */
  //                   2048,      /* Stack size in words */
  //                   NULL,       /* Task input parameter */
  //                   1,          /* Priority of the task */
  //                   &stepper_task,       /* Task handle. */
  //                   0);  /* Core where the task should run */

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
  //CONTROL
  l_motor.setSpeed(control_rate);
  r_motor.setSpeed(control_rate);

  //STEPPING
  l_motor.runSpeed();
  r_motor.runSpeed();

}
