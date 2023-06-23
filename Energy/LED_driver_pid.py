import machine
from machine import Pin, ADC, PWM, Timer, deepsleep, lightsleep, RTC
import time

time.sleep(5) # Pauses execution for 5 seconds when powered on, gives time to connect through IDE before code starts running

vret_pin = ADC(Pin(26)) # Current sensor (1.02 ohm resistor)
vout_pin = ADC(Pin(28)) # Output voltage (multiplied by 2.49/12.49 ~ 1/5)
vin_pin = ADC(Pin(27)) # Input voltage (multiplied by 2.49/12.49 ~ 1/5)
led = Pin("LED", Pin.OUT) # Onboard LED pin

pwm = PWM(Pin(0))
pwm.freq(100000)
pwm_en = Pin(1, Pin.OUT)

printCount = 0
pwm_out = 0 # Actual value to output
pwm_ref = 0 # Intended value to output

loopTrigger = False # Flag that is set to true every 1ms
loopTimer = Timer()
def allowLoop(timer):
    global loopTrigger
    loopTrigger = True
    
loopTimer.init(freq=1000, mode=Timer.PERIODIC, callback=allowLoop)
lastTime = 0

# Output limits
voutTarget = 2.6 # LED target voltage
vCutoff = 2.8 # LED absolute voltage limit
currentLimit = 0.25 # Amps

"""
yellow = 1.9 # kp = 200 # 26 mA - 49 mW
blue = 2.6 # kp = 300 # 28 mA - 72.8 mW
red = 1.90 # kp = 300 # 31 mA - 59 mW

Total = 109 mW
Avg = 54.36 mW (Flashing)
    = 10.9 mW (WiFi)
    
Totals:
Constant = 221 mW
Flashing = 166.8 mW
WiFi = 220.9 mW
"""

# Hysteresis
thL, thH = 7, 8

# PID gains
voutError = 0.0
voutErrorSum = 0.0
voutErrorLast = 0.0
change = 0.0
kp = 300 # 300
ki = 0 # 5
kd = 10 # 10
vMax = 0

runLight = False
canSleep = False
flashCounter = 0

def saturate(duty):
    if duty > 62500:
        duty = 62500
    if duty < 100:
        duty = 100
    return duty

while True:
    
    
    if loopTrigger:
        
        now = time.ticks_us()
        Ts = time.ticks_diff(now, lastTime) # Calculates difference in time between loops
        lastTime = now
        
        vin = ((vin_pin.read_u16()/65535) * 3.3) / (2.49 / 12.49) # Convert input voltage ADC to voltage value
        vout = ((vout_pin.read_u16()/65535) * 3.3) / (2.49 / 12.49) # Convert output voltage ADC to voltage value
        iret = ((vret_pin.read_u16()/65535) * 3.3) / 1.02 # Convert current sensing resistor ADC value to current
        
        loopTrigger = False # Resets flag
        
        if vin < thL and canSleep: # If input voltage drops below minimum value
            runLight = False
            pwm_en.value(0) # Disable PWM
            pwm.duty_u16(0)
            print("Stopped")
            
            #time.sleep(1)
            lightsleep(1000) # Enter light sleep for 1s for improved efficiency
            #deepsleep(1000)
                
        elif vin > thH and not runLight: # Once input voltage rises again
            runLight = True
            canSleep = True
            
            pwm_ref = 0
            voutErrorSum = 0 # Reset controller signals
            voutError = 0
            voutErrorLast = 0
            pwm_en.value(1)
            
            for i in range(10): # FLash LED 10 times to indicate wakeup
                led.toggle()
                time.sleep(0.1)
            
        if runLight:
                
            # PID Controller
            voutError = voutTarget - vout # Works out error between target and actual output voltage
            """if (voutError) < 0:
                print(voutError, voutTarget, vout)"""
            voutErrorSum += voutError * (Ts / 1000000) # Use a sum in place of integral
            if voutErrorSum > 5000: # Constrains the sum to stop it growing too large
                voutErrorSum = 5000
            elif voutErrorSum < -5000:
                voutErrorSum = -5000
            
            change = (kp * voutError) + (ki * voutErrorSum) + (kd * (voutErrorLast - voutError) / Ts) # PID equation
            if change > 5000: # Constrains output
                change = 5000
            elif change < -5000:
                change = -5000
            
            pwm_ref += change # Changes duty cycle
            pwm_ref = saturate(pwm_ref)
            voutErrorLast = voutError
            
            if iret > currentLimit: # If above current limit,
                pwm_ref -= change # Restrict duty cycle
                print("Current Limit")
                
            if vout > vCutoff: # If above voltage limit
                pwm_ref -= change # Restrict duty cycle
                print("Voltage Limit")
            
            pwm_out = saturate(pwm_ref) # Limit duty cycle
            pwm.duty_u16(int(pwm_out)) # Output duty cycle
            
            if vout > vMax: # Records the highest output voltage achieved
                vMax = vout
                
            flashCounter += 1 # Flashes LED beacons every 1s
            if flashCounter >= 1000:
                pwm_en.value(0)
                pwm.duty_u16(0)
                print("Flash off")
                lightsleep(1000)
                print("Flash on")
                pwm_ref = 0
                voutErrorSum = 0
                voutError = 0
                voutErrorLast = 0
                pwm_en.value(1)
                flashCounter = 0
        
        printCount += 1
        if printCount > 1000:        
            print(f"V_in: {vin},\t V_out: {vout},\t I_out: {iret},\t Duty: {pwm_out/65535},\t pwm_ref: {pwm_ref},\t Vmax: {vMax}, E: {voutError}")
            #print(vout, pwm_ref, change, voutError, voutErrorSum, (voutErrorLast - voutError), Ts)
            led.toggle()
            printCount = 0
