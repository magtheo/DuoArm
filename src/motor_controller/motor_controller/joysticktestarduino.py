import serial
from lss import *
import numpy as np

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

x_analog_value = 0
z_analog_value = 0

mid_threshold = 30
diagonal_threshold_cardinal = 255
diagonal_threshold_ordinal = diagonal_threshold_cardinal + 1
top_limit_lss1 = 230
bottom_limit_lss1 = -1600
top_limit_lss0 = -1060
bottom_limit_lss0 = 1400

lss0 = LSS(0)
lss1 = LSS(1)
lss2 = LSS(2)


def lss1_within_limits():
    return bottom_limit_lss1 <= int(lss1.getPosition()) <= top_limit_lss1

def lss0_within_limits():
    return top_limit_lss0 <= int(lss0.getPosition()) <= bottom_limit_lss0

def lss1_beyond_bottom_limit():
    return int(lss1.getPosition()) < bottom_limit_lss1

def lss1_beyond_top_limit():
    return int(lss1.getPosition()) > top_limit_lss1

def lss0_beyond_bottom_limit():
    return int(lss0.getPosition()) > bottom_limit_lss0

def lss0_beyond_top_limit():
    return int(lss0.getPosition()) < top_limit_lss0

def down_step(self):
    self.wheelRPM(10)

def up_step(self):
    self.wheelRPM(-10)

def stop_wheel(self):
    self.wheelRPM(0)


def read_analog_values(ser_obj):
    received_vals = ser_obj.read(4)
    if (len(received_vals) == 4):
        x_analog_val = int.from_bytes(received_vals[0:2], byteorder='little')
        z_analog_val = int.from_bytes(received_vals[2:4], byteorder='little')
        return x_analog_val, z_analog_val
    else:
        return None, None

if __name__ == '__main__':

    ser_obj = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    while True:

        x_analog_value, z_analog_value = read_analog_values(ser_obj)

        if (x_analog_value is not None and z_analog_value is not None):
            print(x_analog_value)
            print(z_analog_value)
            print(f'LSS1 -> position: {lss1.getPosition()}')
            print(f'LSS0 -> position: {lss0.getPosition()}')
            print('---------------------------------')

            # Joystick at North (UP) placement
            if ((512 - diagonal_threshold_cardinal < x_analog_value < 512 + diagonal_threshold_cardinal) and \
                  (z_analog_value == 0)):
                
                if (not lss1_beyond_top_limit() and not lss0_beyond_top_limit()):
                    print("Moving upwards")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    down_step(lss1)
                    up_step(lss0)
                else:
                    print("Maximum upwards position in both or either of the servos has been reached: Upwards motion not allowed")

            # Joystick at South (DOWN) placement 
            elif ((512 - diagonal_threshold_cardinal < x_analog_value < 512 + diagonal_threshold_cardinal) and \
                  (z_analog_value == 1023)):
                
                if (not lss1_beyond_bottom_limit() and not lss0_beyond_bottom_limit()):
                    print("Moving downwards")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    up_step(lss1)
                    down_step(lss0)
                else:
                    print("Maximum downwards position in both or either of the servos has been reached: Downwards motion not allowed")
                
            # Joystick at East (RIGHT) placement 
            elif ((x_analog_value == 1023) and \
                  (512 - diagonal_threshold_cardinal < z_analog_value < 512 + diagonal_threshold_cardinal)):

                if (not lss1_beyond_bottom_limit() and not lss0_beyond_top_limit()):
                    print("Moving to the right")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    up_step(lss1)
                    up_step(lss0)
                else:
                    print("One of the servos has reached one of their max positions: Right motion not allowed")

                
            # Joystick at West (LEFT) placement
            elif ((x_analog_value == 0) and \
                  (512 - diagonal_threshold_cardinal < z_analog_value < 512 + diagonal_threshold_cardinal)):
                
                if (not lss1_beyond_top_limit() and not lss0_beyond_bottom_limit()):
                    print("Moving to the left")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    down_step(lss1)
                    down_step(lss0)
                else:
                    print("One of the servos has reached one of their max positions: Left motion not allowed")

            # Joystick at Northeast (RIGHT-UP DIAGONAL) placement
            elif ((1023 - diagonal_threshold_ordinal <= x_analog_value <= 1023) and \
                  (0 <= z_analog_value <= 512 - diagonal_threshold_cardinal)):
                
                if (not lss0_beyond_top_limit()):
                    print("Moving towards Northeast")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    up_step(lss0)
                else:
                    print("LSS0 has reached its max top position: Right-up motion not allowed")

            # Joystick at Southeast (RIGHT-DOWM DIAGONAL) placement
            elif ((1023 - diagonal_threshold_ordinal <= x_analog_value <= 1023) and \
                  (1023 - diagonal_threshold_ordinal <= z_analog_value <= 1023)):
                
                if (not lss1_beyond_bottom_limit()):
                    print("Moving towards Southeast")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    up_step(lss1)
                else:
                    print("LSS1 has reached its max bottom position: Right-down motion not allowed")

            # Joystick at Northwest (LEFT-UP DIAGONAL) placement
            elif (( 0 <= x_analog_value <= 512 - diagonal_threshold_cardinal) and \
                  (0 <= z_analog_value <= 512 - diagonal_threshold_cardinal)):

                if (not lss1_beyond_top_limit()):
                    print("Moving towards Northwest")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    down_step(lss1)
                else:
                    print("LSS1 has reached its max top position: Left-up motion not allowed")

            # Joystick at Southwest (LEFT-DOWN DIAGONAL) placement
            elif (( 0 <= x_analog_value <= 512 - diagonal_threshold_cardinal) and \
                  (512 + diagonal_threshold_cardinal <= z_analog_value <= 1023)):

                if (not lss0_beyond_bottom_limit()):
                    print("Moving towards Southwest")
                    print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                    down_step(lss0)
                else:
                    print("LSS0 has reached its max bottom position: Left-down motion not allowed")

            # Joystick at middle placement
            # elif ((512 - mid_threshold <= x_analog_value <= 512 + mid_threshold) and \
            #       (512 - mid_threshold <= z_analog_value <= 512 + mid_threshold)):
            #     stop_wheel(lss1)
            #     stop_wheel(lss0)
            # Joystick at middle placement
            else:
                stop_wheel(lss1)
                stop_wheel(lss0)

            
                

                
            
        
 


        
        






        