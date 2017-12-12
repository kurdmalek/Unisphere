import serial
import time
from wixel import *
## main ##
master = wixelMasterController()

while (True):
	for i in range(0, 6):
		master.setServoPosition(0, i, 1545)
	master.sendSlaveData(0)
	time.sleep(4)
	
	for i in range(0,6):
		master.setServoPosition(0, i, 1455)
	master.sendSlaveData(0)
	time.sleep(4)

	for i in range(0,6):
		master.setServoPosition(0, i, 0)
	master.sendSlaveData(0)
	time.sleep(4)
master.closeMaster()
