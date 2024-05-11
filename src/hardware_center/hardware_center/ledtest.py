import RPi.GPIO as GPIO
import time

red_led_signal_pin = 17
green_led_signal_pin = 22
blue_led_signal_pin = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(red_led_signal_pin, GPIO.OUT) 
GPIO.setup(green_led_signal_pin, GPIO.OUT)
GPIO.setup(blue_led_signal_pin, GPIO.OUT)

try:
    while True:
        print('Currently in the loop')
        GPIO.output(red_led_signal_pin, GPIO.LOW)
        time.sleep(2)
except KeyboardInterrupt:
    GPIO.cleanup()



    
