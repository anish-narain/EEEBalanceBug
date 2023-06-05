#include <math.h>
#include <ctype.h>

#define RX1 16 // FPGA side: (ARDUINO_IO[9])
#define TX1 17 // FPGA side: (ARDUINO_IO[8]) 

unsigned long start = 0;
unsigned long period = 500;

char buf[4];
//char msg_buf[];
int val = 0;

int bb[3][4];
int centre[2];
int x_error;

int state;

int isNum(char* str, int size) {
  bool isNum =  true;
  for (int i=0; i<size; i++) {
    isNum = isdigit(str[i]);
  }
  return isNum;
}

int hex2int(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    return -1;
}

int hex2int(char* str, int size) {
  int val = 0;

  for (int i=0; i<size; i++) {
    val += hex2int(str[i]) << (size-1-i)*4;
  }

  return val;
}

void string_decode (char* buf) {

  //BOUNDING BOXES
  for (int i=0; i<3; i++) {
    for (int j=0; j<4; j++) {
      bb[i][j] = hex2int(buf+((i+1)*8)+(j*4), 4);
    }
  }

  //WALL-FOLLOWING
  x_error = hex2int(buf+56, 8);

  //BEACON CENTERING
  for (int i=0; i<2; i++) {
    centre[i] = hex2int(buf+64+(i*4), 4);
  }

}

int byte2int(char* buf, int size) {
  int val=0;

  for (int i=0; i<size; i++) {
    val += buf[i] << (8*(size-1-i));
  }

  return val;
}

int byte2int_signed(char* buf, int size) {
  int val = (signed int)(buf[0] << (size-8));

  for (int i=1; i<size; i++) {
    val += buf[i] << (8*(size-1-i));
  }
  
}

int raw_decode(char* buf) {
  if (byte2int(buf, 4) == 0x00524242) {
    state = 0;
  }

  switch(state) {
    case 0:
      Serial.println("start of message");
      break;
    case 1: case 2: case 3: case 4: case 5: case 6:
      for (int i=0; i<2; i++) {
        bb[(state-1)/2][i] = byte2int(buf+(i*2), 2);
      }
      break;
    case (7):
      x_error = byte2int_signed(buf, 4);
      break;
    case (8):
      for (int i=0; i<2; i++) {
        centre[i] = byte2int(buf+(i*2), 2);
      }
      break;
    default:
      Serial.println("out of bounds error");
      break;
  }

  state++;

}

//Display states
void data2string() {
  Serial. println("BOUNDING BOXES:");

  Serial.print("red: x_min =");
  Serial.print(bb[0][0]);
  Serial.print(" y_min =");
  Serial.print(bb[0][1]);
  Serial.print(" x_max =");
  Serial.print(bb[0][2]);
  Serial.print(" y_max =");
  Serial.println(bb[0][3]);

  Serial.print("yellow: x_min =");
  Serial.print(bb[1][0]);
  Serial.print(" y_min =");
  Serial.print(bb[1][1]);
  Serial.print(" x_max =");
  Serial.print(bb[1][2]);
  Serial.print(" y_max =");
  Serial.println(bb[1][3]);

  Serial.print("blue: x_min =");
  Serial.print(bb[2][0]);
  Serial.print(" y_min =");
  Serial.print(bb[2][1]);
  Serial.print(" x_max =");
  Serial.print(bb[2][2]);
  Serial.print(" y_max =");
  Serial.println(bb[2][3]);

  Serial.println("CENTERING: ");
  Serial.print("centre_x=");
  Serial.print(centre[0]);
  Serial.print(" distance=");
  Serial.println(centre[1]);

  Serial.println("WALL-FOLLOWING:");
  Serial.print("x_error=");
  Serial.println(x_error);
}

void setup() {
  Serial.begin(115200); //console
  Serial1.begin(115200, RX1, TX1); //FPGA
  delay(800);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - start >= period) {
    start = currentMillis;

    if (Serial1.available() > 0){

      int size = Serial1.readBytesUntil('\n', buf, 4);
      raw_decode(buf);
      data2string();

    }
  }
}