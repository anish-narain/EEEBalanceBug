#include <math.h>
#include <ctype.h>

unsigned long start = 0;
unsigned long period = 500;

char buf[72];
int val = 0;

int isNum(char str*, int size) {
  bool isNum =  true;
  for (int i=0; i<size; i++) {
    isNum = isDigit(buf[i]);
  }
  return isNum;
}

void setup() {
  Serial.begin(115200);       // initialize UART with baud rate of 9600 bps
  delay(800);
  Serial.flush();
  Serial.println(val);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - start >= period) {
    start = currentMillis;

    if (Serial.available() > 0){

      int size = Serial.readBytesUntil('\n', buf, 20);

      if (isNum(buf, size)) {
        val = atoi(buf);
        if (val < 1000) {
          val++;
          Serial.println(val);
        }
      }
    }
  }
}