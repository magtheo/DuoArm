

from lss import *
import time

lss2 = LSS(2)
lss2.setOriginOffset(0)

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
CST_LSS_Baud = LSS_DefaultBaud
initBus(CST_LSS_Port, CST_LSS_Baud) 

while True:
    
    time.sleep(2)
    lss2.wheelRPM(10)
    time.sleep(6)
    lss2.wheelRPM(0)
    print(lss2.getPosition())
    time.sleep(2)
    lss2.wheelRPM(-10)
    time.sleep(6)
    lss2.wheelRPM(0)
    print(lss2.getPosition())