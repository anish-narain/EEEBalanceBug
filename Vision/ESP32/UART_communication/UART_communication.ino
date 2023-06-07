#include <math.h>
#include <ctype.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8]) 

unsigned long start = 0;
unsigned long period = 500;

unsigned int dist[3] = {0};
int centre[2] = {0};
bool col_detect = true;

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
      Serial.println("start of message");
      break;
    case 1: case 2: case 3:
      Serial.println("state 1,2,3");
      dist[state-1] = byte2int(buf, 4);
      break;
    case 4:
      Serial.println("state 4");
      col_detect = (bool)byte2int_signed(buf, 4);
      break;
    case 5:
      Serial.println("state 5");
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

void metrics2string() {
  Serial.printf("dist_red = %d\n", dist[0]);
  Serial.printf("dist_yellow = %d\n", dist[1]);
  Serial.printf("dist_blue = %d\n", dist[2]);

  Serial.printf("col_detect = %d\n", col_detect);
  Serial.printf("beacon_x = %d\n", centre[0]);
  Serial.printf("beacon_x to centre = %d\n", centre[1]);
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

    // Convert the raw bytes to a formatted string
    char formattedString[9];
    sprintf(formattedString, "%08x", *((unsigned int*)buf));

    // Print the formatted string
    Serial.print(formattedString);
    Serial.print(" ");
    Serial.println();

    int a = byte2int(buf, 4);
    Serial.println(a, HEX);
    raw_decode(buf);
    //metrics2string();
    
  }
}