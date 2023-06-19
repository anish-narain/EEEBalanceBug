/*
 * Based upon code written by Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 2% - 98%
 * Control frequency is set as 1kHz. 
 
 Implements MPPT algorithm using a boost converter
*/

#include <Wire.h>
#include <INA219_WE.h>

INA219_WE ina219; // Instantiate current sensor

float open_loop; // Open loop duty cycle
float va,vb,vref,iL,dutyref,current_mA; // Measurement Variables
unsigned int sensorValue0,sensorValue1,sensorValue2,sensorValue3;  // ADC sample values declaration
float oc=0; // Difference between current and current limit
float current_limit = 1.0;
boolean Boost_mode = 0;
boolean CL_mode = 0;
unsigned int loopTrigger;
unsigned int com_count=0; // A variables to count the interrupts. Used for program debugging.

float pLast, vLast; // Power and voltage at previous sample
float dir = 1; // The direction that duty cycle is currently moving (-1/+1)
int count = 0; // Countrols the dre

void setup() {
  
  noInterrupts(); // Disable all interrupts
  pinMode(13, OUTPUT);  // Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); // Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // Using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  TCA0.SINGLE.PER = 999;
  TCA0.SINGLE.CMP1 = 999;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; // 16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; // 62.5kHz
  analogWrite(6,120); 

  Serial.begin(115200);
  interrupts(); // Enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // Initiates the current sensor
  Wire.setClock(700000); // Set the comms speed for i2c

  dutyref = 0.99; // Begin on high dutyref value. Since it is inverted, it results in an acutal dutycycle of 0.1, resulting in an initial low voltage

  pinMode(12, OUTPUT); // NMOS gate
  
}

void loop() {
  
  if (loopTrigger) { // Runs when the interrupt is triggered
    
    digitalWrite(13, HIGH); // Set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); // Input from the OL_CL switch
    Boost_mode = digitalRead(2); // Input from the Buck_Boost switch

    if (Boost_mode){
      
      if (CL_mode) { //Closed Loop Boost
          pwm_modulate(1); // This disables the Boost as we are not using this mode
          
      } else{ // Open Loop Boost

        count += 1;

        if (count == 100){
        
          float p = iL * vb; // Calculate current power at port B (PV cell side)

          if (p < pLast){ // If the current power is lower than the previous sample
            dir *= -1; // Change the direction that the dutycycle is moving in
          }
          
          dutyref += 0.01 * dir; // Move the dutycycle in the specified direction (either +/- 0.01)
          dutyref = saturation(dutyref, 0.99, 0.33); // Limit the dutycycle so that the output voltage doesn't go beyond the limit

          pLast = p; // Store the values from this sample
          vLast = vb;
          count = 0;
          
        }

        if (va > 17.9){ // If the voltage at port A (connected to capacitor) goes above 18V (the capacitor's voltage limit)
          dutyref += 0.01; // Restrict the duty cycle
        }
          
        current_limit = 2;
        oc = iL-current_limit; // Calculate the difference between current measurement and current limit
        if (oc > 0) {
          open_loop = open_loop + 0.001; // We are above the current limit so less duty cycle
        } else {
          open_loop = open_loop - 0.001; // We are below the current limit so more duty cycle
        }
        
        open_loop = saturation(open_loop, 0.99, dutyref); // Saturate the duty cycle at the reference or a min of 0.01
        pwm_modulate(open_loop); // And send it out
          
      }
      
    } else{  
      
      if (CL_mode) { // Closed Loop Buck
          pwm_modulate(1); // This disables the Buck as we are not using this mode
      } else{ // Open Loop Buck
          pwm_modulate(1); // This disables the Buck as we are not using this mode
      }
      
    }

    com_count++;
    if (com_count >= 500) {  //Send out data every 0.5 seconds
      
      Serial.print("Va: ");
      Serial.print(va);
      Serial.print("\t");

      Serial.print("Vb: ");
      Serial.print(vb);
      Serial.print("\t");

      Serial.print("Inductor Current: ");
      Serial.print(iL);
      Serial.print("\t");

      Serial.print("Power: ");
      Serial.print(iL * vb);
      Serial.print("\t");

      Serial.print("Duty: ");
      Serial.print(dutyref);
      Serial.print("\t");

      Serial.print("Boost Mode: ");
      Serial.print(Boost_mode);
      Serial.print("\t");

      Serial.print("CL Mode: ");
      Serial.print(CL_mode);
      Serial.print("\n");
      
      com_count = 0;
      
    }

    digitalWrite(13, LOW); // Reset pin13.
    loopTrigger = 0;
    
  }
}

// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; // Clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required values for the main loop
void sampling(){

  // Make the initial sampling operations for the circuit measurements
  
  sensorValue0 = analogRead(A0); //sample Vb
  sensorValue2 = analogRead(A2); //sample Vref
  sensorValue3 = analogRead(A3); //sample Va
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the values so they are a bit more usable/readable
  // The analogRead process gives a value between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  vb = sensorValue0 * (12400/2400) * (4.096 / 1023.0); // Convert the Vb sensor reading to volts
  vref = sensorValue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  va = sensorValue3 * (12400/2400) * (4.096 / 1023.0); // Convert the Va sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    //dutyref = saturation(sensorValue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    //dutyref = sensorValue2 * (1.0 / 1023.0);
  }
  
}

float saturation( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}
