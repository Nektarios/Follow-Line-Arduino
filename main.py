from machine import Pin, PWM
import utime

# Using GP0 ... GP4 for the sensors
sensors = [Pin(pin, Pin.IN) for pin in range(5)]

# Pins 5 & 6 for Left Motor and 8 & 9 for the Right one
motorLeft1 = PWM(Pin(5, Pin.OUT))
motorLeft2 = PWM(Pin(6, Pin.OUT))
motorRight1 = PWM(Pin(8, Pin.OUT))
motorRight2 = PWM(Pin(9, Pin.OUT))

# Setting the frequency for each direction of the motor to 50 to get them moving
motorLeft1.freq(50)
motorLeft2.freq(50)
motorRight1.freq(50)
motorRight2.freq(50)

# PID constants
Kp = 20000
Ki = 0.0
Kd = 0

# Initialize variables for PID control
lastError = 0
integral = 0


# Reading the sensors values and returning them as an array with 0 for white empty space and 1 for the black lines
def readSensors():
    return [1 - sensor.value() for sensor in sensors]


# Calculating error to fix the distance the middle sensor has from the black line using the weights array and sumarizing it based on the readings of the sensors
def calculate_error(sensors):
    weights = [2, 1, 0, -1, -2]
    error = sum([weights[i] * sensors[i] for i in range(len(sensors))])
    return error


# Control the motor speeds accordingly to ensure the car can follow the lines in the most optimal way
def runMotors(leftSpeed, rightSpeed):
    # Make sure the left and right speed never exceed the value of 65000
    if leftSpeed > 65000:
        leftSpeed = 65000
    if rightSpeed > 65000:
        rightSpeed = 65000

    # Set speeds for each motor based on the corrections that come from the pid algorithm
    if leftSpeed >= 0:
        motorLeft1.duty_u16(leftSpeed)
        motorLeft2.duty_u16(0)
    else:
        motorLeft1.duty_u16(0)
        motorLeft2.duty_u16(-leftSpeed)

    if rightSpeed >= 0:
        motorRight1.duty_u16(rightSpeed)
        motorRight2.duty_u16(0)
    else:
        motorRight1.duty_u16(0)
        motorRight2.duty_u16(-rightSpeed)


def lineFollowing():
    global lastError, integral

    while True:
        sensors = readSensors()
        error = calculate_error(sensors)

        integral += error
        derivative = error - lastError
        correction = Kp * error + Ki * integral + Kd * derivative
        baseSpeed = 50000

        if sum(sensors) >= 3:
            runMotors(0, 0)
            return

        if correction > 0:
            leftSpeed = baseSpeed + correction
            rightSpeed = baseSpeed - correction
        else:
            leftSpeed = baseSpeed + correction
            rightSpeed = baseSpeed - correction

        leftSpeed = int(leftSpeed)
        rightSpeed = int(rightSpeed)
        runMotors(leftSpeed, rightSpeed)

        lastError = error

        utime.sleep(0.01)
        print(readSensors())

try:
    lineFollowing()
except KeyboardInterrupt:
    runMotors(0, 0)
    print("Stopped")