# import Adafruit_BBIO.GPIO as GPIO
# import Adafruit_BBIO.PWM as PWM

def setup_pin(pin_name:str, is_out:bool):
    # dir = GPIO.OUT if is_out else GPIO.IN
    # GPIO.setup(str, dir)
    return 0

def start_pwm(pin_name:str, duty_cycle:int, freq:int, is_falling_edge:bool):
    # PWM.start(pin_name, duty_cycle, freq, is_falling_edge)
    return 0

def stop_pwm(pin_name:str):
    # PWM.stop(pin_name)
    return 0

def set_duty_cycle(pin_name:str, duty_cycle:int):

    # PWM.set_duty_cycle(pin_name, duty_cycle)
    return 0

