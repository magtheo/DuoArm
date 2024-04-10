###############################################################################
#	Author:			Sebastien Parent-Charette (support@robotshop.com)
#	Version:		1.0.0
#	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
#	
#	Description:	A library that makes using the LSS simple.
#					Offers support for most Python platforms.
#					Uses the Python serial library (pySerial).
###############################################################################

### Import required liraries
import re
import serial
from math import sqrt, atan, acos, fabs

### Import constants
# import lss_const as lssc
from .lss_const import *

### Class functions
def initBus(portName, portBaud):
	LSS.bus = serial.Serial(portName, portBaud)
	LSS.bus.timeout = 0.1

def closeBus():
	if LSS.bus is not None:
		LSS.bus.close()
		del LSS.bus

# Write with a an optional parameter
def genericWrite(id, cmd, param = None):
	if LSS.bus is None:
		return False
	if param is None:
		LSS.bus.write((LSS_CommandStart + str(id) + cmd + LSS_CommandEnd).encode())
	else:
		LSS.bus.write((LSS_CommandStart + str(id) + cmd + str(param) + LSS_CommandEnd).encode())
	return True

# Read an integer result
def genericRead_Blocking_int(id, cmd):
	if LSS.bus is None:
		return None
	try:
		# Get start of packet and discard header and everything before
		c = LSS.bus.read()
		while (c.decode("utf-8") != LSS_CommandReplyStart):
			c = LSS.bus.read()
			if(c.decode("utf-8") == ""):
				break
		# Get packet
		data = LSS.bus.read_until(LSS_CommandEnd.encode('utf-8')) #Otherwise (without ".encode('utf-8')") the received LSS_CommandEnd is not recognized by read_until, making it wait until timeout.
		# Parse packet
		matches = re.match("(\d{1,3})([A-Z]{1,4})(-?\d{1,18})", data.decode("utf-8"), re.I)
		# Check if matches are found
		if(matches is None):
			return(None)
		if((matches.group(1) is None) or (matches.group(2) is None) or (matches.group(3) is None)):
			return(None)
		# Get values from match
		readID = matches.group(1)
		readIdent = matches.group(2)
		readValue = matches.group(3)
		# Check id
		if(readID != str(id)):
			return(None)
		# Check identifier
		if(readIdent != cmd):
			return(None)
	except:
		return(None)
	# return value
	return(readValue)

# Read a string result
#@classmethod
def genericRead_Blocking_str(id, cmd, numChars):
	if LSS.bus is None:
		return None
	if LSS.bus is None:
		return None
	try:
		# Get start of packet and discard header and everything before
		c = LSS.bus.read()
		while (c.decode("utf-8") != LSS_CommandReplyStart):
			c = LSS.bus.read()
			if(c.decode("utf-8") == ""):
				break
		# Get packet
		data = LSS.bus.read_until(LSS_CommandEnd.encode('utf-8')) #Otherwise (without ".encode('utf-8')") the received LSS_CommandEnd is not recognized by read_until, making it wait until timeout.
		data = (data[:-1])
		# Parse packet
		matches = re.match("(\d{1,3})([A-Z]{1,4})(.{" + str(numChars) + "})", data.decode("utf-8"), re.I)
		# Check if matches are found
		if(matches is None):
			return(None)
		if((matches.group(1) is None) or (matches.group(2) is None) or (matches.group(3) is None)):
			return(None)
		# Get values from match
		readID = matches.group(1)
		readIdent = matches.group(2)
		readValue = matches.group(3)
		# Check id
		if(readID != str(id)):
			return(None)
		# Check identifier
		if(readIdent != cmd):
			return(None)
	except:
		return(None)
	# return value
	return(readValue)

class LSS:
	# Class attribute
	bus = None
	
	### Constructor
	def __init__(self, id = 0):
		self.servoID = id
	
	### Attributes
	servoID = 0
	
	### Functions
	#> Actions
	def reset(self):
		return (genericWrite(self.servoID, LSS_ActionReset))
	
	def limp(self):
		return (genericWrite(self.servoID, LSS_ActionLimp))
	
	def hold(self):
		return (genericWrite(self.servoID, LSS_ActionHold))
	
	def move(self, pos):
		return (genericWrite(self.servoID, LSS_ActionMove, pos))
	
	def moveRelative(self, delta):
		return (genericWrite(self.servoID, LSS_ActionMoveRelative, delta))
	
	def wheel(self, speed):
		return (genericWrite(self.servoID, LSS_ActionWheel, speed))
	
	def wheelRPM(self, rpm):
		return (genericWrite(self.servoID, LSS_ActionWheelRPM, rpm))
	
	#> Queries
	#def getID(self):
	#def getBaud(self):
	
	def getStatus(self):
		genericWrite(self.servoID, LSS_QueryStatus)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryStatus))
	
	def getOriginOffset(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryOriginOffset, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryOriginOffset))
	
	def getAngularRange(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryAngularRange, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryAngularRange))
	
	def getPositionPulse(self):
		genericWrite(self.servoID, LSS_QueryPositionPulse)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryPositionPulse))
	
	def getPosition(self):
		genericWrite(self.servoID, LSS_QueryPosition)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryPosition))
	
	def getSpeed(self):
		genericWrite(self.servoID, LSS_QuerySpeed)
		return (genericRead_Blocking_int(self.servoID, LSS_QuerySpeed))
	
	def getSpeedRPM(self):
		genericWrite(self.servoID, LSS_QuerySpeedRPM)
		return (genericRead_Blocking_int(self.servoID, LSS_QuerySpeedRPM))
	
	def getSpeedPulse(self):
		genericWrite(self.servoID, LSS_QuerySpeedPulse)
		return (genericRead_Blocking_int(self.servoID, LSS_QuerySpeedPulse))
	
	def getMaxSpeed(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryMaxSpeed, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryMaxSpeed))
	
	def getMaxSpeedRPM(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryMaxSpeedRPM, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryMaxSpeedRPM))
	
	def getColorLED(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryColorLED, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryColorLED))
	
	def getGyre(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryGyre, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryGyre))
	
	# returns 0 if "DIS"
	def getFirstPosition(self):
		genericWrite(self.servoID, LSS_QueryFirstPosition)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryFirstPosition))
	
	# returns true/false based on if QFD returns "DIS" (= False)
	def getIsFirstPositionEnabled(self):
		genericWrite(self.servoID, LSS_QueryFirstPosition)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryFirstPosition) is not None)
	
	def getModel(self):
		genericWrite(self.servoID, LSS_QueryModelString)
		return (genericRead_Blocking_str(self.servoID, LSS_QueryModelString, 7))
	
	def getSerialNumber(self):
		genericWrite(self.servoID, LSS_QuerySerialNumber)
		return (genericRead_Blocking_int(self.servoID, LSS_QuerySerialNumber))
	
	def getFirmwareVersion(self):
		genericWrite(self.servoID, LSS_QueryFirmwareVersion)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryFirmwareVersion))
	
	def getVoltage(self):
		genericWrite(self.servoID, LSS_QueryVoltage)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryVoltage))
	
	def getTemperature(self):
		genericWrite(self.servoID, LSS_QueryTemperature)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryTemperature))
	
	def getCurrent(self):
		genericWrite(self.servoID, LSS_QueryCurrent)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryCurrent))
	
	#> Queries (advanced)
	def getAngularStiffness(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryAngularStiffness, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryAngularStiffness))
	
	def getAngularHoldingStiffness(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryAngularHoldingStiffness, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryAngularHoldingStiffness))
	
	def getAngularAcceleration(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryAngularAcceleration, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryAngularAcceleration))
	
	def getAngularDeceleration(self, queryType = LSS_QuerySession):
		genericWrite(self.servoID, LSS_QueryAngularDeceleration, queryType)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryAngularDeceleration))
	
	def getIsMotionControlEnabled(self):
		genericWrite(self.servoID, LSS_QueryEnableMotionControl)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryEnableMotionControl))
	
	def getBlinkingLED(self):
		genericWrite(self.servoID, LSS_QueryBlinkingLED)
		return (genericRead_Blocking_int(self.servoID, LSS_QueryBlinkingLED))
	
	#> Configs
	def setOriginOffset(self, pos, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionOriginOffset, pos))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigOriginOffset, pos))
	
	def setAngularRange(self, delta, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionAngularRange, delta))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigAngularRange, delta))
	
	def setMaxSpeed(self, speed, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionMaxSpeed, speed))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigMaxSpeed, speed))
	
	def setMaxSpeedRPM(self, rpm, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionMaxSpeedRPM, rpm))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigMaxSpeedRPM, rpm))
	
	def setColorLED(self, color, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionColorLED, color))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigColorLED, color))
	
	def setGyre(self, gyre, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionGyreDirection, gyre))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigGyreDirection, gyre))
	
	def setFirstPosition(self, pos):
		return (genericWrite(self.servoID, LSS_ConfigFirstPosition, pos))
	
	def clearFirstPosition(self):
		return (genericWrite(self.servoID, LSS_ConfigFirstPosition))
	
	def setMode(self, mode):
		return (genericWrite(self.servoID, LSS_ConfigMode, mode))
	
	#> Configs (advanced)
	def setAngularStiffness(self, value, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionAngularStiffness, value))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigAngularStiffness, value))
	
	def setAngularHoldingStiffness(self, value, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionAngularHoldingStiffness, value))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigAngularHoldingStiffness, value))
	
	def setAngularAcceleration(self, value, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionAngularAcceleration, value))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigAngularAcceleration, value))
	
	def setAngularDeceleration(self, value, setType = LSS_SetSession):
		if setType == LSS_SetSession:
			return (genericWrite(self.servoID, LSS_ActionAngularDeceleration, value))
		elif setType == LSS_SetConfig:
			return (genericWrite(self.servoID, LSS_ConfigAngularDeceleration, value))
	
	def setMotionControlEnabled(self, value):
		return (genericWrite(self.servoID, LSS_ActionEnableMotionControl, value))
	
	def setBlinkingLED(self, state):
		return (genericWrite(self.servoID, LSS_ConfigBlinkingLED, state))
	
### EOF #######################################################################
