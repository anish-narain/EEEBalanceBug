#include <math.h>
#include <ctype.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[8])
#define TXD2 17 // FPGA side: (ARDUINO_IO[9]) 


void setup() {
  Serial.begin(115200); //console
  Serial1.begin(115200,SERIAL_8N1, RXD2, TXD2); //FPGA
  //delay(800);
}

int state = -1;

int raw_decode(byte* buf) {

  //start of msg
  if (byte2int(buf, 4) == 0x00524242) {
    state = 0;
  }

  //output
  if (state != NULL) {
    switch(state) {
      case 0:
        Serial.println("start of message");
        break;
      case 1: case 2: case 3: case 4: case 5: case 6:
        Serial.println("state 1,2,3,4,5,6");
        // for (int i=0; i<2; i++) {
        //   bb[(state-1)/2][i] = byte2int(buf+(i*2), 2);
        // }
        break;
      case 7:
        Serial.println("state 7");
        // x_error = byte2int_signed(buf, 4);
        break;
      case 8:
        Serial.println("state 8");
        // for (int i=0; i<2; i++) {
        //   centre[i] = byte2int(buf+(i*2), 2);
        // }
        break;
      default:
        Serial.println("no message");
        break;
    }

    //next state
    if (state >-1 & state<8) {
      state++;
    } else if (state == 8) {
      state = -1;
    }  else {
      state= -1;
    }
    
    return 0;
  }

  

}

int byte2int(byte* buf, int size) {
  int val=0;

  for (int i=size-1; i>=0; i--) {
    val += ((unsigned int)buf[i]) << (8*i);
  }

  return val;
}

void loop() {
  if (Serial1.available() >= 4) { // Check if there are at least 4 bytes available
    byte data[4];
    Serial1.readBytes(data, 4); // Read 4 bytes into the 'data' array

    // Convert the raw bytes to a formatted string
    char formattedString[9];
    sprintf(formattedString, "%08x", *((unsigned int*)data));

    // Print the formatted string
    Serial.print(formattedString);
    Serial.print(" ");
    Serial.println();
    int a = byte2int(data, 4);
    Serial.println(a, HEX);
    Serial.println(state);
    raw_decode(data);
    //Serial.println("end of decode");
  }
}
