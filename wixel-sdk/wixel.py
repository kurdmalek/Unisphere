import serial 

# Wixel Master Controller interface
#
# useage:
# wix = new.wixelMasterController()
# wix.setServoPosition(slave, servo, position)
# data = wix.getSlaveData(slave)
# data is 16 bytes (8 words) 3 analog inputs are 0-6

class wixelMasterController:
    def __init__(self):
        usbPort = '/dev/cu.usbmodem1421'
        self.sc = serial.Serial(usbPort, timeout=1)
        self.initializeTables()

    def initializeTables(self):
        self.masterTxTable = []				# create the transmit and
        self.masterRxTable = []				# receive tables
        for i in range(0,64):				# as lists
            self.masterTxTable.append([])
            for j in range(0,6):
                self.masterTxTable[i].append(0)
           
    def closeMaster(self):
        self.sc.close()					# close serial port

    def setServoPosition(self, slave, servo, data):
        self.masterTxTable[slave][servo] = data
        self.sendSlaveData(slave)

    def getSlaveData(self, slave):
        datapacket = []                         # init list
        datapacket.append(chr(0x81))            # command byte, write tx table
        datapacket.append(chr(slave))           # slave to control goes here

        for d in datapacket:  	                # send command to the serial port
            self.sc.write(d)

        returndata = []                         # now read data returned by master
        for d in range(0,16):                   # for this slave
            returndata.append(self.sc.read(1))
        return returndata                       # returned filled container
    
    def sendSlaveData(self, slave):
        datapacket = []                         # init list
        datapacket.append(chr(0x82))            # command byte, write tx table
        datapacket.append(chr(slave))           # slave to control goes here
        
        for i in range(0,6):                    # queue up the master data for tx
            db = self.masterTxTable[slave][i]
            d0 = db & 0x7f                      # data must be adj for 7 bit chars
            d1 = (db & 0x7f80) >> 7             # or reception gets confused
            datapacket.append(chr(d0)) 		# note the convolutions here?
            datapacket.append(chr(d1)) 		# don't like it but it works 

        datapacket.append(chr(0))    		# only have six servos, 12 bytes plus the command and slave
        datapacket.append(chr(0))    		#  so add last two for 16 total packet size

        for d in datapacket:    		# send them to the serial port
            self.sc.write(d)			# note that it HAS to be exactly 16 bytes total
