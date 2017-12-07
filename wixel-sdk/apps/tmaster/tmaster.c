
/**************************************************************
  Master code. This relies on the hardware feature of the radio
  to address packets to individual nodes based on their h/w id.
  
  
  This code simply round robins all the enabled slaves. We wait
  for a response from the slave, or a timeout, before transmitting 
  to the next. 
  
  Slaves never speak unless spoken to, so we limit collisions 
  and maximize throughput with a very low weight, very fast i/o layer. 
  
  Or that's the idea anyhow.
  
  
  Martan 01/2013
	
**************************************************************/

#include <cc2511_map.h>
#include <board.h>
#include <usb.h>
#include <usb_com.h>
#include <random.h>
#include <time.h>
#include <gpio.h>
#include <string.h>
#include <stdio.h>
#include <radio_queue.h>

#define MAX_TX_INTERVAL 0 	// maximum time between transmissions (ms)
#define TX_MAX_TIMEOUT 3	// time we wait for an rx from a slave before we move to next one
#define PACKET_LENGTH 16	// we are moving 16 bytes of data per transmission
#define ROUNDROBIN_TIME 128	// ms for LEDS
#define NUM_SLAVES 64		// 64 total slaves, this give a max of (6*64) 384 servos we can control and 192 analog inputs we can read

// commands to get and set the data
#define COMMAND_GETDATA             0x81
#define COMMAND_PUTDATA             0x82
#define COMMAND_CLEAR_ERROR         0x83
#define COMMAND_SETSLAVES           0x84

uint32 CODE param_controller_addr = 254;
uint32 CODE param_number_slaves = 16;

static BIT txEnabled = 1;
static BIT rxEnabled = 1;

uint8 XDATA debug[128];

// The i/o tables
static volatile XDATA uint8 txtable[NUM_SLAVES][PACKET_LENGTH];
static volatile XDATA uint8 rxtable[NUM_SLAVES][PACKET_LENGTH];

// this is used to select the device to talk to
static int8 deviceIndex = 0;
static int8 pd = 0;

// USB interface
BIT serialProtocolError = 0;
uint8 commandByte;
uint8 dataBytes[32];
uint8 dataBytesLeft = 0;
uint8 dataBytesReceived;
uint8 XDATA response[32];
int16 numSlaves;

void initializeDataTables(void)
{
	int i;
	
	for (i=0;i<NUM_SLAVES;i++)
	{
	    memset(txtable[i], 0, PACKET_LENGTH);
	    memset(rxtable[i], 0, PACKET_LENGTH);
	}
	if (param_number_slaves < NUM_SLAVES)
		numSlaves = param_number_slaves;
	else
		numSlaves = NUM_SLAVES;
}

void setSlaves(int8 slaves)
{
	if (slaves < NUM_SLAVES)
	{
		numSlaves = slaves;
	}
}

// USB control here
void executeCommand()
{
    uint8 slave;
	//uint8 index;
    uint16 worddata;
	
    switch(commandByte)
    {
    case COMMAND_GETDATA:
	    slave = dataBytes[0];		 // get slave address
        memcpy(response, rxtable[slave], PACKET_LENGTH);
		//index = usbComTxAvailable();
		usbComTxSend(response, PACKET_LENGTH);   // Assumption: usbComTxAvailable() returned >= 2 recently.
    	//sprintf(debug, "%x %x : %x %x : %d %d : %d %d : %d %d : %d %d : %d %d", rxtable[slave][0],rxtable[slave][1], rxtable[slave][2], rxtable[slave][3], rxtable[slave][4], rxtable[slave][5], rxtable[slave][6], rxtable[slave][7]);
		//sprintf(debug, "%x %x : %x %x : %d %d : %d %d : %d %d : %d %d : %d %d", response[0], response[1], response[2], response[3], response[4], response[5], response[6], response[7]);
        break;
        
    case COMMAND_PUTDATA:
	    slave = dataBytes[0];		 // get slave address
        memcpy(txtable[slave], dataBytes, PACKET_LENGTH);
    	//sprintf(debug, "%x %x  %d %d : %d %d : %d %d : %d %d *\n", txtable[slave][0],txtable[slave][1], txtable[slave][2], txtable[slave][3], txtable[slave][4], txtable[slave][5], txtable[slave][6], txtable[slave][7], txtable[slave][8], txtable[slave][9]);

        
        // reconstruct buffer passed by python class
        txtable[slave][0] = PACKET_LENGTH;
        txtable[slave][1] = slave;

        // servo 0        
        worddata = dataBytes[2] << 7;
        worddata = worddata | dataBytes[1];
        
        txtable[slave][2] = (worddata & 0xff00) >> 8;
        txtable[slave][3] = worddata & 0xff;

        // servo 1
        worddata = dataBytes[4] << 7;
        worddata = worddata | dataBytes[3];
        
        txtable[slave][4] = (worddata & 0xff00) >> 8;
        txtable[slave][5] = worddata & 0xff;

        // servo 2
        worddata = dataBytes[6] << 7;
        worddata = worddata | dataBytes[5];
        
        txtable[slave][6] = (worddata & 0xff00) >> 8;
        txtable[slave][7] = worddata & 0xff;

        // servo 3
        worddata = dataBytes[8] << 7;
        worddata = worddata | dataBytes[7];
        
        txtable[slave][8] = (worddata & 0xff00) >> 8;
        txtable[slave][9] = worddata & 0xff;

        // servo 4
        worddata = dataBytes[10] << 7;
        worddata = worddata | dataBytes[9];
        
        txtable[slave][10] = (worddata & 0xff00) >> 8;
        txtable[slave][11] = worddata & 0xff;

        // servo 5
        worddata = dataBytes[12] << 7;
        worddata = worddata | dataBytes[11];
        
        txtable[slave][12] = (worddata & 0xff00) >> 8;
        txtable[slave][13] = worddata & 0xff;

        // speed
        worddata = dataBytes[14] << 7;
        worddata = worddata | dataBytes[13];
        
        txtable[slave][14] = (worddata & 0xff00) >> 8;
        txtable[slave][15] = worddata & 0xff;
		
    	//sprintf(debug, "%x %x  %d %d : %d %d : %d %d : %d %d", txtable[slave][0],txtable[slave][1], txtable[slave][2], txtable[slave][3], txtable[slave][4], txtable[slave][5], txtable[slave][6], txtable[slave][7], txtable[slave][8], txtable[slave][9]);

    break;

    case COMMAND_SETSLAVES:
	    slave = dataBytes[0];		 // get slave address
        setSlaves(slave);
        commandByte = 0;
        dataBytesReceived = 0;
        dataBytesLeft = 0;
        break;

    case COMMAND_CLEAR_ERROR:
        serialProtocolError = 0;
        commandByte = 0;
        dataBytesReceived = 0;
        dataBytesLeft = 0;
        break;
    }
}

void processByte(uint8 byteReceived)
{
    if (byteReceived & 0x80)
    {
        // We received a command byte.
        
        if (dataBytesLeft > 0)
        {
            serialProtocolError = 1;
        }
        
        commandByte = byteReceived;
        dataBytesReceived = 0;
        dataBytesLeft = 0;
        
        // Look at the command byte to figure out if it is valid, and
        // determine how many data bytes to expect.
        switch(commandByte)
        {
            case COMMAND_GETDATA:							// get data from the slaves
                dataBytesLeft = 1;
                break;

			case COMMAND_CLEAR_ERROR:
                dataBytesLeft = 0;
                break;

            case COMMAND_PUTDATA:							// write data to slave
                dataBytesLeft = 15;
                break;

            case COMMAND_SETSLAVES:
                dataBytesLeft = 1;
                break;

            default:
                // Received an invalid command byte.
                serialProtocolError = 1;
                return;
        }
    
        if (dataBytesLeft==0)
        {
            // We have received a single-byte command.
            executeCommand();
        }
    }
    else if (dataBytesLeft > 0)
    {
        // We received a data byte for a binary command.

        dataBytes[dataBytesReceived] = byteReceived;
        dataBytesLeft--;
        dataBytesReceived++;
        
        if (dataBytesLeft==0)
        {
            // We have received the last byte of a multi-byte command.
            executeCommand();
        }
    }
}

/** Checks for new bytes available on the USB virtual COM port
 * and processes all that are available. */
void processBytesFromUsb()
{
    uint8 bytesLeft = usbComRxAvailable();
    while(bytesLeft && usbComTxAvailable() >= sizeof(response))
    {
        processByte(usbComRxReceiveByte());
        bytesLeft--;
    }
}

uint8 getNextSlave(void)
{
    deviceIndex++;
	if (deviceIndex > numSlaves)
	    deviceIndex = 0;
	return deviceIndex;
}

void reportResults()
{
    static uint16 lastReportSentTime;

        if ((uint16)(getMs() - lastReportSentTime) >= 500)
           {
            uint8 XDATA report[128];
            uint8 reportLength = sprintf(report, "%s\r\n", debug);
            usbComTxSend(report, reportLength);
            lastReportSentTime = (uint16)getMs();
           }
}


void setupInputPorts()
{
	// insure all of port 1 is general purpose I/O
	P1SEL = 0;

	// we want all inputs for possible buttons/switches
	P1DIR = 0;
}

uint8 readInputPorts()
{
	uint8 retByte;

	retByte = 0;
	
	return retByte;
}

/* Main loop */

void main(void)
{
    // pointers to link packets
    uint8 XDATA * txBuf;
    uint8 XDATA * rxBuf;					// pointers to buffers in radio_queue
    
	uint32 lastTx = 0;						// transmit time
	uint32 lastRx = 0;						// receive time
	uint8 slaveAddr = 0;

	deviceIndex = 0;

	initializeDataTables();
	setupInputPorts();
    systemInit();							// Whole system intialize
    usbInit();								// Initialize USB communications
    radioQueueInit();						// Pololu library, works well for this

	ADDR = param_controller_addr;			// this is our address, it's programmable but best set to 254

	LED_YELLOW(0);							// turn off the indicator LEDs
	LED_RED(0);
	
	
    while(1)								// start our main loop
    {
        boardService();						// board services
        usbComService();					// keep USB running
        processBytesFromUsb();
		//reportResults();
		
		readInputPorts();

        // Transmit data to slave
        if (txEnabled && (uint8)(getMs() - lastTx) > MAX_TX_INTERVAL && (txBuf = radioQueueTxCurrentPacket()))
        {
			slaveAddr = getNextSlave();		// address of slave to transmit (0-63)
											// move data from tx table to slave
			memcpy(txBuf, &txtable[slaveAddr], PACKET_LENGTH);

		    txBuf[0] = PACKET_LENGTH;		// set packet length for transmit
			txBuf[1] = slaveAddr;			// insert after length
			
			LED_RED(1);						// set transmit active (red) LED

//			sprintf(debug, "%x %x  %d %d : %d %d : %d %d : %d %d", txBuf[0],txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7], txBuf[8], txBuf[9]);

            radioQueueTxSendPacket();		// send the data
            lastTx = getMs();				// snag the transmit time
			txEnabled = 0;					// no more transmits until slave responds or we timeout.
        }
		
		// check timeout, if we go over, slave didn't talk back, or we missed it, move on
		if ( ((getMs() - lastTx) > TX_MAX_TIMEOUT) )
		     txEnabled = 1;

        // if nothing for a while, turn off the leds
		if ( ((getMs() - lastTx) > ROUNDROBIN_TIME ) )
        {
			LED_RED(0);
        }

		if ( ((getMs() - lastRx) > ROUNDROBIN_TIME ) )
        {
			LED_YELLOW(0);
        }

        // receive data from slave
        if (rxEnabled && (rxBuf = radioQueueRxCurrentPacket()))
        {
			LED_YELLOW(1);                  // rx data, turn it on

            slaveAddr = (uint8 XDATA) rxBuf[2];			// don't assume packet came from above transmit
			
			//sprintf(debug, "%x %x : %x %x : %x %x : %x %x : %x %x", rxBuf[0],rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7], rxBuf[8], rxBuf[9]);

			memcpy(&rxtable[slaveAddr], rxBuf, PACKET_LENGTH);
            lastRx = getMs();
            radioQueueRxDoneWithPacket();
			txEnabled = 1;
        }
    }
}
