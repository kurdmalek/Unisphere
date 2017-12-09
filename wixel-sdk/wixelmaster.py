import serial
import time
from wixel import *


## main ##

master = wixelMasterController()

while (true):
	for i in range(0, 6):
		master.setServoPosition(0, i, 1000)
		master.setServoPosition(0, i, 2000)
		master.setServoPosition(0, i, 0)
 
	master.sendSlaveData(0)
	time.sleep(4)
	master.sendSlaveData(0)
	time.sleep(4)
	master.sendSlaveData(0)
	time.sleep(4)

master.closeMaster()
