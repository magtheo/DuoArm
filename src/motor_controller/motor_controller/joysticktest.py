import serial
import time
from lss import *

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

x_analog_value = 0
z_analog_value = 0
diagonal_threshold_cardinal = 255
diagonal_threshold_ordinal = diagonal_threshold_cardinal + 1

lss0 = LSS(0)
lss1 = LSS(1)
lss2 = LSS(2)


def down_step(self):
    self.wheelRPM(10)

def up_step(self):
    self.wheelRPM(-10)

def stop_wheel(self):
    self.wheelRPM(0)

def read_analog_values(ser_obj):
    analog_vals = ser_obj.read(4)
    if (len(analog_vals) == 4):
        x_analog_val = int.from_bytes(analog_vals[0:2], byteorder='little')
        z_analog_val = int.from_bytes(analog_vals[2:4], byteorder='little')
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

            # Joystick at North (UP) placement +- diagonal threshold cardinal
            if ((512 - diagonal_threshold_cardinal <= x_analog_value <= 512 + diagonal_threshold_cardinal) and \
                  (0 - diagonal_threshold_cardinal <= z_analog_value <= 0 + diagonal_threshold_cardinal)):
                print("Moving upwards")
                print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                down_step(lss1)
                up_step(lss0)


            # Joystick at South (DOWN) placement +- diagonal threshold cardinal 
            elif ((512 - diagonal_threshold_cardinal <= x_analog_value <= 512 + diagonal_threshold_cardinal) and \
                  (1023 - diagonal_threshold_cardinal <= z_analog_value <= 1023)):
                print("Moving downwards")
                print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                up_step(lss1)
                down_step(lss0)
                
            
            # Joystick at East (RIGHT) placement +- diagonal treshold cardinal
            elif ((1023 - diagonal_threshold_cardinal <= x_analog_value <= 1023) and \
                  (512 - diagonal_threshold_cardinal <= z_analog_value <= 512 + diagonal_threshold_cardinal)):
                print("Moving to the right")
                print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                up_step(lss1)
                up_step(lss0)
                
            
            # Joystick at West (LEFT) placement +- diagonal treshold cardinal
            elif ((0 - diagonal_threshold_cardinal <= x_analog_value <= 0 + diagonal_threshold_cardinal) and \
                  (512 - diagonal_threshold_cardinal <= z_analog_value <= 512 + diagonal_threshold_cardinal)):
                print("Moving to the left")
                print(f'analog_values -> (x_analog_value, z_analog_value) : {(x_analog_value, z_analog_value)}')
                down_step(lss1)
                down_step(lss0)

            # Joystick at middle placement
            else:
                stop_wheel(lss1)
                stop_wheel(lss0)
            
        
 


        
        






        