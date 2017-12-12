import serial
import time
from wixel import *
## main ##
master = wixelMasterController()

while (True):
	for i in range(0, 6):
		master.setServoPosition(0, i, 45)
		master.sendSlaveData(0)
		time.sleep(4)
		master.setServoPosition(0, i, -45)
		master.sendSlaveData(0)
		time.sleep(4)
		master.setServoPosition(0, i, -45)
		master.sendSlaveData(0)
		time.sleep(4)

master.closeMaster()
