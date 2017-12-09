import serial
import time
from wixel import *


## main ##

master = wixelMasterController()

for i in range(0, 6):
	master.setServoPosition(0, i, 1000)
	master.sendSlaveData(0)
	time.sleep(4)
	master.setServoPosition(0, i, 2000)
	master.sendSlaveData(0)
	time.sleep(4)
	master.setServoPosition(0, i, 0)
	master.sendSlaveData(0)          
master.closeMaster()