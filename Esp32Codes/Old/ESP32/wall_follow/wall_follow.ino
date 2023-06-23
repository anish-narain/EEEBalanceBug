#include <Stepper.h>
#include <math.h>
#include <ctype.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8])

#define R_STPR 22
#define R_DIRR 23

//STEPPER

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper r_stepper(stepsPerRevolution, R_STPR, R_DIRR);

int stepCount = 0;  // number of steps the motor has taken

//COMMUNICATION

unsigned int dist[3] = {0};
bool col_detect = true;
int zoom_level = 1;

int r_state = -1;
int t_flag = 0;

unsigned long prev = 0;
unsigned long period = 5000;


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
        Serial.printf("dist to beacon %d = %d mm\n", i, dist[i]);
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

  int sensorReading;

  unsigned long current  = millis();
  if (current - prev == period) {
    prev = current;
    sensorReading = (sensorReading >= 1023) ? 0 : sensorReading++;
  }

  int motorSpeed = map(sensorReading, 0, 1023, 0, 100);
  // set the motor speed:
  if (motorSpeed > 0) {
    r_stepper.setSpeed(motorSpeed);
    // step 1/100 of a revolution:
    r_stepper.step(stepsPerRevolution / 100);
  }
}
