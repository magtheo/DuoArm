import RPi.GPIO as GPIO
import time


joystick_button_pin = 23
button_pressed = False
GPIO.setmode(GPIO.BCM)
GPIO.setup(joystick_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.input(joystick_button_pin)
prev_button_state = GPIO.LOW
count = 0
debounce_duration = 0.5
last_button_press_time = 0
current_time = time.time()

try:
    while True:
        time.sleep(0.1)
        current_button_state = GPIO.input(joystick_button_pin)
        print(button_pressed)
        
        # Check for rising edge (HIGH to LOW transition)
        if current_button_state == GPIO.HIGH and prev_button_state == GPIO.LOW:

            if (time.time() - last_button_press_time >= debounce_duration):
                button_pressed = True

                last_button_press_time = time.time()
        else:
            button_pressed = False
        
        prev_button_state = current_button_state


except KeyboardInterrupt:

    GPIO.cleanup()