import serial
import time
from wixel import *


## main ##

master = wixelMasterController()

while(True):
	for i in range(0, 6):
		master.setServoPosition(1, i, 45)
	master.sendSlaveData(0)
		time.sleep(2)
		master.setServoPosition(1, i, -45)
		master.sendSlaveData(0)
		time.sleep(2)
		master.setServoPosition(1, i, 0)
	master.sendSlaveData(0)          
master.closeMaster()

