import RPi.GPIO as GPIO
import time

GPIO_TRIGGER = 31
GPIO_ECHO = 33
def distance():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
    GPIO.setup(GPIO_ECHO,GPIO.IN)
    GPIO.output(GPIO_TRIGGER, False)
    time.sleep(0.02)
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER,False)
    count = time.time()
    StartTime = time.time()
    StopTime = time.time()
    while GPIO.input(GPIO_ECHO) == 0 and time.time() - count < 0.1:
        StartTime = time.time()
    while GPIO.input(GPIO_ECHO) == 1 and time.time() - count < 0.1:
        StopTime = time.time()

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300)/2.0
    GPIO.cleanup()
    return distance

while True:
    dist = distance()
    print('Distance:',dist,'cm')