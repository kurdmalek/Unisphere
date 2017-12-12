import serial
import time
from wixel import *
## main ##
master = wixelMasterController()

while (True):
	for i in range(0, 6):
		master.setServoPosition(1, i, 1800)
	master.sendSlaveData(1)
	time.sleep(4)
	
	for i in range(0,6):
		master.setServoPosition(1, i, 1200)
	master.sendSlaveData(1)
	time.sleep(4)

	for i in range(0,6):
		master.setServoPosition(1, i, 0)
	master.sendSlaveData(1)
	time.sleep(4)
master.closeMaster()
