import time
import VL53L0X
import RPi.GPIO as GPIO
import math
# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the servo pin (replace with your actual pin number)
servo_pin = 21

# Set up the servo pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM object with a frequency of 50Hz
default_duty_cycle = 7.5
servo_pwm = GPIO.PWM(servo_pin, 50)
servo_pwm.start(default_duty_cycle)
# Start the PWM with an initial duty cycle (0 degrees position)
#servo_pwm.start(2.5)  # Adjust duty cycle for your servo
tmp = False 

tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)
# I2C Address can change before tof.open()
# tof.change_address(0x32)
tof.open()
# Start ranging
tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)

timing = tof.get_timing()


class PIDController:
    def __init__(self, Kp, Ki, Kd):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def calculate(self, current_value, target_value):
        error = target_value - current_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error
        I = self.Ki * self.integral

        # Derivative term
        derivative = error - self.last_error
        D = self.Kd * derivative

        # Calculate the output
        output = P + I + D

        self.last_error = error

        return output

# Example usage
target = 200  # Target position
current = 0  # Current position
min_duty_cycle = 7.30  # Minimum duty cycle
max_duty_cycle = 7.850 #maximum duty cycle

#servo_pin = 21
#servo_pin.StartPwm(servo_pin , 50)

# Initialize the PID controller
pid = PIDController(Kp=1.200, Ki=0.018, Kd=12.000)
servo_pwm.ChangeDutyCycle(default_duty_cycle)
start_kick = False 

count = 0


while True:
    
    if not start_kick :
        

        
        """
        servo_pwm.ChangeDutyCycle(min_duty_cycle-0.1)
        print("STARTED")
        time.sleep(1.521)
        """
        servo_pwm.ChangeDutyCycle(max_duty_cycle)
        time.sleep(2.5)
        start_kick = not start_kick
        
    try:


        current = tof.get_distance()

        """
        if current <= 40:
            servo_pwm.ChangeDutyCycle(min_duty_cycle-1)
            time.sleep(0.21)
            servo_pwm.ChangeDutyCycle(max_duty_cycle+1)
            time.sleep(0.21)
        """
        # Calculate the control signal
        control_signal = pid.calculate(current, target)

        control_new = abs(control_signal/10)
        print(f"{control_new}")
        print(f"{current} in mm")
        print(f"PID = {control_signal}")

        duty_cycle = max(min_duty_cycle, min(max_duty_cycle, control_new))
   

        duty_cycle = default_duty_cycle + control_new / 13
        if duty_cycle <= min_duty_cycle :

            duty_cycle = min_duty_cycle

        if duty_cycle >= max_duty_cycle:

            duty_cycle = max_duty_cycle

        print(f"Applying duty cycle: {duty_cycle}")
        servo_pwm.ChangeDutyCycle(duty_cycle)
        print(f"Diff: {target - current}")
    
    # Simulate a delay (replace this with the actual delay required for your system)
        time.sleep(0.05)

        #print("Target position reached!") if abs(target - current) < 1.0 else print("")
        

        if abs(target-current) < 0.705 : 

            print("target psotion reached")
            count += 1 

            if count == 8:

                #servo_pwm.ChangeDutyCycle(default_duty_cycle)
                servo_pwm.stop()
                GPIO.cleanup()
                exit()

    except KeyboardInterrupt:
        servo_pwm.ChangeDutyCycle(default_duty_cycle)
        servo_pwm.stop()
        GPIO.cleanup()

