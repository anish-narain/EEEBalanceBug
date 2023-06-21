#include <math.h>
#include <ctype.h>
#include <Arduino.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8]) 

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
  
}