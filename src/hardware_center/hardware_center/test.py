

from lss import *
import time

lss2 = LSS(2)
CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
CST_LSS_Baud = LSS_DefaultBaud
initBus(CST_LSS_Port, CST_LSS_Baud)

while True:
    
    lss2.wheelRPM(-100)
    time.sleep(1)
    lss2.wheelRPM(0)
    break
    
    