#include <Stepper.h>
#include <math.h>
#include <ctype.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8]) 

//STEPPER

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper r_stepper(stepsPerRevolution, 8, 9, 10, 11);

int stepCount = 0;  // number of steps the motor has taken

//COMMUNICATION

unsigned long start = 0;
unsigned long period = 500;

unsigned int dist[3] = {0};
int centre[2] = {0};
int x_error = 0;

int state = -1;

int byte2int(byte* buf, int size) {
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
      x_error = byte2int_signed(buf, 4);
      break;
    case 5:
      for (int i=0; i<2; i++) {
        centre[i] = byte2int(buf+(i*2), 2);
      }
      break;
    default:
      Serial.println("no message");
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

void setup() {
  Serial.begin(115200); //console
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2); //FPGA
  //delay(800);
}

void loop() {
  if (Serial1.available() >= 4){
    byte buf[4];
    Serial1.readBytes(buf, 4);
    raw_decode(buf);
  }

  // read the sensor value:
  int sensorReading = analogRead(A0);
  // map it to a range from 0 to 100:
  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);
  // set the motor speed:
  if (motorSpeed > 0) {
    myStepper.setSpeed(motorSpeed);
    // step 1/100 of a revolution:
    myStepper.step(stepsPerRevolution / 100);
  }
}
