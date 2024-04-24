import RPi.GPIO as GPIO
import time

# Setup GPIO mode and pins
GPIO.setmode(GPIO.BCM)
button_pin = 18  # Example GPIO pin for the button
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Use internal pull-up resistor

# Initialize variables
button_state = GPIO.input(button_pin)
last_button_state = button_state
last_button_time = time.time()

# Main loop
try:
    while True:
        # Read button state
        button_state = GPIO.input(button_pin)

        # Check for button press
        if button_state != last_button_state:
            if button_state == GPIO.LOW:  # Button pressed
                current_time = time.time()
                if current_time - last_button_time > 0.2:  # Debounce time (adjust as needed)
                    print("Button pressed")
                    last_button_time = current_time

        # Update last button state
        last_button_state = button_state

        # Add a small delay to reduce CPU usage
        time.sleep(0.01)

finally:
    # Cleanup GPIO
    GPIO.cleanup()