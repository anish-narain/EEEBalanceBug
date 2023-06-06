void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void adjustSpeed(int x_error) {
  const float maxSpeedFactor = 2.0;   // Maximum speed factor
  const int maxError = 127;           // Maximum x_error value

  // Calculate the speed factors for each motor based on x_error
  float speedFactorS1 = 1.0;
  float speedFactorS2 = 1.0;

  if (x_error > 0) {
    speedFactorS1 = 1.0; // deviate to the right, increase right motor's speed (max twice of original speed (2))
    speedFactorS2 = 1.0 + (maxSpeedFactor - 1.0) * (static_cast<float>(x_error) / maxError);
  } else if (x_error < 0) { // increase left motor's speed
    speedFactorS1 = 1.0 + (maxSpeedFactor - 1.0) * (static_cast<float>(-x_error) / maxError);
    speedFactorS2 = 1.0;
  }

  // Update the delayTime for each motor
  unsigned long newDelayTimeS1 = delayTime / speedFactorS1;
  unsigned long newDelayTimeS2 = delayTime / speedFactorS2;

  // Update the delayTime for each motor independently
  delayTimeS1 = newDelayTimeS1;
  delayTimeS2 = newDelayTimeS2;
}

--------------------------------------------------------------------------------

// remember to define pins (Serial1)
// something line this 

#define RX1 16 // FPGA side: (ARDUINO_IO[9])
#define TX1 17 // FPGA side: (ARDUINO_IO[8])            

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, RX1, TX1);
}



#include <stdint.h>

const int MAX_MESSAGE_LENGTH = 200;
const int NUM_HEX_NUMBERS = 8;

char VisionMsg[MAX_MESSAGE_LENGTH];
int hexNumbers[NUM_HEX_NUMBERS];
int count = 0;

void receiveVisionMessage() {
  for (int i = 0; i < MAX_MESSAGE_LENGTH; i++) {
    VisionMsg[i] = ' ';
  }
  int i = 0; // Initialize i to 0
  while (Serial1.available()) {
    char Visionbody = Serial1.read();
    if (Visionbody != '\n') {
      VisionMsg[i] = Visionbody;
      i++;
    } else {
      Serial.println("End of message");
      break;
    }
  }
  char* ptr = strtok(VisionMsg, " ");  // Split the message according to space
  count = 0;

  while (ptr != NULL && count < NUM_HEX_NUMBERS) {
    hexNumbers[count] = strtoul(ptr, NULL, 16);  // Convert the token to a hexadecimal number
    ptr = strtok(NULL, " ");  // Move to the next token
    count++;
  }
}

void loop() {
  if (Serial1.available()) {
    receiveVisionMessage();

    // Example: extract 6th number
    int a = hexNumbers[5];
    int16_t value = static_cast<int16_t>(a);    // cast a as a 16 bit signed value
  }
}


---------------------------------------------------------------------------------------------
#include <math.h>
#include <ctype.h>
#define RX1 16 // FPGA side: (ARDUINO_IO[9])
#define TX1 17 // FPGA side: (ARDUINO_IO[8])  

unsigned long start = 0;
unsigned long period = 500;
const int MAX_MESSAGE_LENGTH = 200;


char VisionMsg[MAX_MESSAGE_LENGTH];

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, RX1, TX1);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - start >= period) {
    start = currentMillis;

    if (Serial1.available() > 0){

      int size = Serial1.readBytesUntil('\n', VisionMsg, 200);
      Serial.println("End of message");
      
      }
    

    const int MAX_ITEMS = 10;  // Maximum number of 4-byte items
    const int ITEM_SIZE = 4;   // Size of each item in bytes
    
    unsigned long items[MAX_ITEMS];  // Array to store the separated items
    
    int size = Serial.readBytesUntil('\n', VisionMsg, MAX_ITEMS * ITEM_SIZE);
    
    if (size >= ITEM_SIZE) {
      int itemCount = size / ITEM_SIZE;  // Calculate the number of items
    
      for (int i = 0; i < itemCount; i++) {
        items[i] = (unsigned long)(buf[i * ITEM_SIZE + 3]) << 24 |
                   (unsigned long)(buf[i * ITEM_SIZE + 2]) << 16 |
                   (unsigned long)(buf[i * ITEM_SIZE + 1]) << 8 |
                   (unsigned long)(buf[i * ITEM_SIZE]);
      }
    
      // Now the separated items are stored in the 'items' array
      // You can access them individually as items[0], items[1], etc.
    }

  
    while (ptr != NULL && count < NUM_HEX_NUMBERS) {
      hexNumbers[count] = strtoul(ptr, NULL, 16);  // Convert the token to a hexadecimal number
      ptr = strtok(NULL, " ");  // Move to the next token
      count++;
     }
   }
}
