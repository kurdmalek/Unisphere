/** 
	Slave collects analog inputs, outputs servo positions
	
	Only talks to the master, never to other slaves
	
*********/

#include <cc2511_map.h>
#include <wixel.h>
#include <board.h>
#include <usb.h>
#include <usb_com.h>
#include <random.h>
#include <time.h>
#include <gpio.h>
#include <stdio.h>
#include <radio_queue.h>
#include <servo.h>

#define MAX_TX_INTERVAL 0 // maximum time between transmissions (ms)
#define PACKET_LENGTH 16
#define ROUNDROBIN_TIME 250	// ms for LEDS

int32 CODE param_master_addr = 254;
int32 CODE param_device_addr = 1;
int32 CODE param_input_mode = 0;

int32 CODE param_servo_speed0 = 0;
int32 CODE param_servo_speed1 = 0;
int32 CODE param_servo_speed2 = 0;
int32 CODE param_servo_speed3 = 0;
int32 CODE param_servo_speed4 = 0;
int32 CODE param_servo_speed5 = 0;

static BIT txEnabled = 1;
static BIT rxEnabled = 1;

uint8 XDATA debug[128];

// Here we define what pins we will be using for servos.  Our choice is:
// Servo 0 = P0_2
// Servo 1 = P0_3
// Servo 2 = P0_4
// Servo 3 = P1_2
// Servo 4 = P1_1
// Servo 5 = P1_0
// Enable ALL servos

uint8 CODE pins[] = {2, 3, 4, 12, 11, 10};

void myServosInit()
{
    // Start the servo library.
    servosStart((uint8 XDATA *)pins, sizeof(pins));
    // Set the speeds of servos
    servoSetSpeed(0, param_servo_speed0);
    servoSetSpeed(1, param_servo_speed1);
    servoSetSpeed(2, param_servo_speed2);
    servoSetSpeed(3, param_servo_speed3);
    servoSetSpeed(4, param_servo_speed4);
    servoSetSpeed(5, param_servo_speed5);
	
    servoSetTarget(0, 1200);
    servoSetTarget(1, 1200);
    servoSetTarget(2, 1200);
    servoSetTarget(3, 1200);
    servoSetTarget(4, 1200);
    servoSetTarget(5, 1200);
}

void analogInputsInit()
{
    switch(param_input_mode)
    {
    case 1: // Enable pull-up resistors for all pins on Port 0.
        // This shouldn't be necessary because the pull-ups are enabled by default.
        P2INP &= ~(1<<5);  // PDUP0 = 0: Pull-ups on Port 0.
        P0INP = 0;
        break;

    case -1: // Enable pull-down resistors for all pins on Port 0.
        P2INP |= (1<<5);   // PDUP0 = 1: Pull-downs on Port 0.
        P0INP = 0;         // This line should not be necessary because P0SEL is 0 on reset.
        break;

    default: // Disable pull-ups and pull-downs for all pins on Port 0.
        P0INP = 0x3F;
        break;
    }
}

// for debugging only
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


void main(void)
{
    // pointers to link packets
    uint8 XDATA * txBuf;
    uint8 XDATA * rxBuf;
    uint32 lastTx = 0;
    uint32 lastRx = 0;
    uint8 adcIndex = 0;
	uint16 a,b,c,d,e,f,g;
    uint16 vddMillivolts;
	uint16 adcValues[6];

	uint8 slaveAddr = param_device_addr;
	uint8 masterAddr = param_master_addr;

	ADCCFG = 0x23;

    systemInit();
    usbInit();
	analogInputsInit();
    radioQueueInit();
	myServosInit();
	
	ADDR = slaveAddr;
	
	LED_YELLOW(0);
	LED_RED(0);
	
	// start main code
	
    while(1)
    {
        boardService();
        usbComService();
		reportResults();
		
		// take care of the ADC inputs
	
	    if(adcIndex == 0)
		{
		 vddMillivolts = adcReadVddMillivolts();
         adcSetMillivoltCalibration(vddMillivolts);
		}
	
		adcIndex++;
		if(adcIndex > 2)
		   adcIndex = 0;

		switch(adcIndex)
		{
		 case 0:
			adcValues[adcIndex] = adcRead(0);
			break;
		 case 1:
			adcValues[adcIndex] = adcRead(1);
			break;
		 case 2:
			adcValues[adcIndex] = adcRead(5);
			break;
		}
			
        // get servo positions from the master and apply them
        if (rxEnabled && (rxBuf = radioQueueRxCurrentPacket()))
        {
			LED_YELLOW(1);
			
			a = (uint16) ((rxBuf[2]<<8) | rxBuf[3]);
			b = (uint16) ((rxBuf[4]<<8) | rxBuf[5]);
			c = (uint16) ((rxBuf[6]<<8) | rxBuf[7]);
			d = (uint16) ((rxBuf[8]<<8) | rxBuf[9]);
			e = (uint16) ((rxBuf[10]<<8) | rxBuf[11]);
			f = (uint16) ((rxBuf[12]<<8) | rxBuf[13]);
			g = (uint16) ((rxBuf[14]<<8) | rxBuf[15]);
			
			servoSetTarget(0, a);
			servoSetTarget(1, b);
			servoSetTarget(2, c);
			servoSetTarget(3, d);
			servoSetTarget(4, e);
			servoSetTarget(5, f);

        	sprintf(debug, "%x %x : %d : %d : %d : %d : %d : %d", rxBuf[0],rxBuf[1], a, b, c, d, e, f); 
            
            lastRx = getMs();
			radioQueueRxDoneWithPacket();
			txEnabled = 1;							// got a good packet, talk back
        }
		
        // if nothing for a while, turn off the leds
		if ( ((getMs() - lastTx) > ROUNDROBIN_TIME ) )
        {
			LED_RED(0);
        }
		
		if ( ((getMs() - lastRx) > ROUNDROBIN_TIME ) )
		{
			LED_YELLOW(0);
		}

		
        // transmit ADC values to master
        if (txEnabled && (uint8)(getMs() - lastTx) > MAX_TX_INTERVAL && (txBuf = radioQueueTxCurrentPacket()))
        {
		    txBuf[0] = PACKET_LENGTH;
			txBuf[1] = masterAddr;
			
			txBuf[2] = slaveAddr;		// tell master who sent the damn data!
			txBuf[3] = 0;
			
			txBuf[4] = adcValues[0] & 0x00ff;
			txBuf[5] = (adcValues[0] & 0xff00) >> 8;
			txBuf[6] = adcValues[1] & 0x00ff;
			txBuf[7] = (adcValues[1] & 0xff00) >> 8;
			txBuf[8] = adcValues[2] & 0x00ff;
			txBuf[9] = (adcValues[2] & 0xff00) >> 8;
			
			LED_RED(1);

			//sprintf(debug, "%x %x : %x %x : %x %x", txBuf[4],txBuf[5], txBuf[6],txBuf[7], txBuf[8],txBuf[9]);
			
            radioQueueTxSendPacket();
			
            lastTx = getMs();
			txEnabled = 0;			// wait for the master before we transmit again
        }
    }
}
