# import Adafruit_BBIO.GPIO as GPIO
# import Adafruit_BBIO.PWM as PWM

def setupPin(pinName:str, isOut:bool):
    # dir = GPIO.OUT if isOut else GPIO.IN
    # GPIO.setup(str, dir)
    return 0 # remove the return after this is implemented

def cleanup():
    # GPIO.cleanup()
    return 0 # remove the return after this is implemented

def startPWM(pinName:str, dutyCycle:int, freq:int, isFallingEdge:bool):
    # PWM.start(pinName, dutyCycle, freq, isFallingEdge)
    return 0

def stopPWM(pinName:str):
    # PWM.stop(pinName)
    return 0

def setDutyCycle(pinName:str, dutyCycle:int):
    # PWM.set_duty_cycle(pinName, dutyCycle)
    return 0
