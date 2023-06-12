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

---------------------------------------------------------------------------------------

#include <math.h>
#include <ctype.h>
#include <Arduino.h>

#define RXD2 16 // FPGA side: (ARDUINO_IO[9])
#define TXD2 17 // FPGA side: (ARDUINO_IO[8]) 

unsigned int dist[3] = {0};
int centre[2] = {0};
bool col_detect = true;
int zoom_level = 1;

int r_state = -1;
int t_flag = 0;

unsigned long prev = 0;
unsigned long period = 5000;

float calibrate_x, calibrate_y; // used for coordinates calibration


int byte2int(byte* buf, int size) {
  int val=0;

  // for (int i=(size-1); i>=0; i--) {
  //   val += buf[i] << (8*i);
  // }

  for (int i=0; i<size; i++) {
    val+= buf[i] << (8*i);
  }

  return val;
}

void zoom(int zoom_level){
  if (Serial1.availableForWrite() >= 4) {
      byte buf[4]; 
      int2byte(zoom_level, buf);
      Serial1.write(buf, 4);
      Serial.printf("%lu zoom sent\n", current);
      delay(100);
    }
}

void calibration_coor(float gyroZe) {
  float targetAngle = 360.0; // Target angle for the turn
  float currentAngle = 0.0; // Current angle of the turn
  float lastTime = micros(); // Last time recorded

  // dimension of the arena
  int height = 500;
  int width = 350;
  
  int sum_dist[3] = {0};    // red, blue, yellow
  int count[3] = {0};
  int avg_dist[3] = {0};

  // zoom in the camera for calibration
  zoom(3);

  // Stop the motors
  s1.stop();
  s2.stop();
  s1.control();
  s2.control();

  while (currentAngle < targetAngle) {
    
    // Rotate the robot in place, can be replaced by left turn -------------------------
    s1.start();
    s2.start();
    s1.changeDirection(false); // Rotate counter-clockwise
    s2.changeDirection(false); // Rotate counter-clockwise
    s1.control();
    s2.control();

    // Update the current angle based on the gyro reading (not sure if we still need to update current angle)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float Ts = micros() - lastTime; // Time since last loop
    lastTime = micros();
    bearing += ((float)g.gyro.z - gyroZe) * (Ts / 1000000); // Integrate angular velocity with respect to time

    // Adjust the current angle to be between 0 and 360 degrees
    if (bearing < 0.0) {
      bearing += 360.0;
    } else if (bearing >= 360.0) {
      bearing -= 360.0;
    }

    // Update the current angle of the turn
    currentAngle += abs((float)g.gyro.z - gyroZe) * (Ts / 1000000);

    // sum up all positive distance readings
    for (int i=0; i<3; i++){
      if (dist[i] > 0){
        sum_dist[i] += dist[i];
        count[i] += 1;
      }
    }
    
    // Delay to control the speed of the turn
    delay(10); // Adjust the delay value to control the speed of the turn
  }

  // Stop the motors after completing the turn
  s1.stop();
  s2.stop();
  s1.control();
  s2.control();

  // calculate average distance of all three
  for (int i=0; i<3; i++){
      avg_dist[i] = sum_dist[i] / count[i];
    }

  // zoom out the camera
  zoom(1);
  
  // calculate x and y coordinates
  float beta = acos((pow(avg_dist[0],2)+pow(width,2)-pow(avg_dist[2],2)/(2*avg_dist[0]*width)));
  float x1 = avg_dist[0]*cos(beta);
  float y1 = height - avg_dist[0]*sin(beta);

  float gamma = acos((pow(avg_dist[2],2)+pow(height,2)-pow(avg_dist[1],2)/(2*avg_dist[2]*width)));
  float x2 = width - avg_dist[2]*sin(gamma);
  float y2 = height - avg_dist[2]*cos(gamma);

  calibrate_x = (x1+x2)/2;
  calibrate_y = (y1+y2)/2;
}

void int2byte(int n, byte bytes[4]) {
  for (int i=0; i<4; i++) {
    bytes[i] = (n >> 8*i) & 0xFF;
  }
}

int raw_decode(byte* buf) {

  //start condition
  if (byte2int(buf, 4) == 0x00524242) {
    r_state = 0;
  }

  //output
  switch(r_state) {
    case 0:
      break;
    case 1: case 2: case 3:
      dist[r_state-1] = byte2int(buf, 4);
      break;
    case 4:
      col_detect = (bool)byte2int(buf, 4);
      break;
    case 5:
      for (int i=0; i<2; i++) {
        centre[i] = byte2int(buf+(i*2), 2);
      }
      break;
    default:
      break;
  }

  //next r_state
  if (r_state > -1 & r_state < 5) {
    r_state++;
  } else if (r_state == 5) {
    r_state = -1;
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

  //RECEIVING
  if (Serial1.available() >= 4){
    byte buf[4];
    Serial1.readBytes(buf, 4);
    raw_decode(buf);
    //metrics2string();
  }

  //SIM SERVER INPUT
  unsigned long current  = millis();
  if (current - prev == period) {
    Serial.printf("%lu zoom request\n", current);
    prev = current;
    zoom_level = 3;
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
