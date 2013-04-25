/*
 Copyright (c) 2013, University of Utah - Electrical and Computer Engineering - WiESEL
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the WiESEL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL WiESEL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Anh Luong
 * 06/25/2012
 */

extern void interpretCommand();

#include "BSL_Device_File.h"
#include "BSL430_PI.h"
#include "BSL430_Command_Interpreter.h"
#include "BSL430_Command_Definitions.h"

//errors
#define HEADER_INCORRECT (PI_COMMAND_UPPER + 0x01)
#define CHECKSUM_INCORRECT (PI_COMMAND_UPPER + 0x02)
#define PACKET_SIZE_ZERO (PI_COMMAND_UPPER + 0x03)
#define PACKET_SIZE_TOO_BIG (PI_COMMAND_UPPER + 0x04)
#define UNKNOWN_ERROR (PI_COMMAND_UPPER + 0x05)

#define MAX_BUFFER_SIZE 260

//#define SLAVE_ADDRESS 0x48
#define ADC

char verifyData(int checksum);

//I2C Interface specific functions
void receiveDataFromMaster(unsigned char);
void sendByte(char data);

char byStatusByte;
char RX_StatusFlags = RX_PACKET_ONGOING;
volatile int checksum = 0;

char *BSL430_ReceiveBuffer;
char *BSL430_SendBuffer;
unsigned int BSL430_BufferSize;
char RAM_TX_Buf[MAX_BUFFER_SIZE];
char RAM_RX_Buf[MAX_BUFFER_SIZE];

#define USCIB_I2C_VERSION 0x03
//const unsigned char BSL430_PI_Version = (BSL430_USCIB_I2C_PI + USCIB_I2C_VERSION);

unsigned char RxValue;
unsigned int rxdataPointer = 0;
unsigned int transmitBufferLength = 0;
unsigned int transmitPoint = 0;


/*******************************************************************************
 * *Function:    PI_init  - I2C
 * *Description: Initializes the I2C Slave
 *  Returns:      None
 *******************************************************************************/

void PI_init()
{

	BSL430_ReceiveBuffer = RAM_RX_Buf;
	BSL430_SendBuffer = &RAM_TX_Buf[4];
	rxdataPointer = 0;

    UCSCTL4 = SELA__REFOCLK + SELM__DCOCLK + SELS__DCOCLK;   // to do check SELA

    UCSCTL0 = 0x000;                                         // Set DCO to lowest Tap

    UCSCTL1 = DCORSEL_4;                                     // 8MHz nominal DCO
    UCSCTL5 = DIVM_0 + DIVS_0;
    UCSCTL2 = FLLD_2 | (((DCO_SPEED / ACLK_SPEED) / 4) - 1); // 8MHz

	// Set up address from ADC
#ifdef ADC
	P6SEL |= BIT3;
	REFCTL0 &= ~REFMSTR;
	ADC12CTL0 = ADC12ON+ADC12SHT0_12+ADC12MSC+ADC12REFON+ADC12REF2_5V;
	ADC12CTL1 = ADC12SHP+ADC12CONSEQ_2;         // Use sampling timer
	ADC12MCTL0 = ADC12INCH_3+ADC12SREF_1;
	volatile unsigned int i;
	for(i = 0; i < 0x30; i++);
	unsigned int sum = 0;
	for(i = 0; i < 16; i++)
	{
		ADC12CTL0 |= ADC12ENC;                  // Enable conversions
		ADC12CTL0 |= ADC12SC;                   // Start conversion
		while (!(ADC12IFG & BIT0));
		sum += ADC12MEM0;
	}
	sum >>= 9;
	if(sum == 0x00)
		sum++;
#endif

	TX_PORT_SEL |= RXD + TXD;                               // Assign I2C pins to USCI_B1
	UCZNCTL1 |= UCSWRST;                                    // Enable SW reset
	UCZNCTL0 = UCMODE_3 + UCSYNC;                           // I2C Slave, synchronous mode
#ifdef ADC
	UCZNI2COA = sum + UCGCEN;				    // Own Address is 048h
#else
	UCZNI2COA = 0x48 + UCGCEN;								// Own Address is 048h
#endif
	UCZNCTL1 &= ~UCSWRST;                                   // Clear SW reset, resume operation

	UCZNIE |= UCTXIE + UCRXIE + UCSTPIE + UCSTTIE;

} //init

/*******************************************************************************
 * *Function:    PI_receivePacket
 * *Description: Handles the sequence of START, DATA, RESTART and then SENDS DATA
 *             and then STOP
 *  Returns:      None
 *
 *******************************************************************************/
#define I2C_START_CONDITION_RECEIVED 0x06
#define I2C_STOP_CONDITION_RECEIVED  0x08
#define I2C_DATA_RECEIVED            0x0A
#define I2C_TRANSMIT_BUFFER_EMPTY    0x0C
char PI_receivePacket()
{
	RX_StatusFlags = RX_PACKET_ONGOING;

	while (RX_StatusFlags == RX_PACKET_ONGOING)
	{
		switch (UCZIV)
		{
		case I2C_START_CONDITION_RECEIVED:
			transmitPoint = 0;               //Start Condition Received
			rxdataPointer = 0;
			break;

		case I2C_STOP_CONDITION_RECEIVED:    //Stop Condition Received
			break;

		case I2C_DATA_RECEIVED:
			RxValue = UCZRXBUF;              //Data Received
			receiveDataFromMaster(RxValue);
			break;

		case I2C_TRANSMIT_BUFFER_EMPTY:
			if (transmitPoint < transmitBufferLength)
			{
				UCZTXBUF = RAM_TX_Buf[transmitPoint++];
			}
			break;

		default:          break;

		} // switch
	} // while
	return RX_StatusFlags;
}

/*******************************************************************************
 * *Function:    receiveDataFromMaster  - I2C routine
 * *Description: Retrieves the format info, data etc from the received data and
 *             sets error flags if format is wrong - wrong checksum, wrong header etc
 *             and stripes the data bytes from the format characters and headers
 *
 *             eg: 0x80 0x01 0x00 0x19 0xE8 0x62 gets converted to
 *                 Header - 0x80
 *                 Length - 0x01
 *                 Command - 0x19
 *                 Checksum - CKL-0xE8
 *                            CKH-0x62
 * *Returns: None
 *
 *******************************************************************************/

void receiveDataFromMaster(unsigned char dataByte)
{
	if (rxdataPointer == 0)                      // first byte is the size of the Core packet
	{
		if (dataByte != 0x80)                    // first byte in packet should be 0x80
		{
			sendByte(HEADER_INCORRECT);
			RX_StatusFlags = RX_ERROR_RECOVERABLE;
		}
		else
		{
			rxdataPointer++;
		}
	}
	else if (rxdataPointer == 1)                 // first byte is the size of the Core packet
	{
		BSL430_BufferSize = dataByte;
		rxdataPointer++;
	}
	else if (rxdataPointer == 2)
	{
		BSL430_BufferSize |= (int)dataByte << 8;
		if (BSL430_BufferSize == 0)
		{
			sendByte(PACKET_SIZE_ZERO);
			RX_StatusFlags = RX_ERROR_RECOVERABLE;
		}
		if (BSL430_BufferSize > MAX_BUFFER_SIZE) // For future devices that might need smaller
			// packets
		{
			sendByte(PACKET_SIZE_TOO_BIG);
			RX_StatusFlags = RX_ERROR_RECOVERABLE;
		}
		rxdataPointer++;
	}
	else if (rxdataPointer == (BSL430_BufferSize + 3))
	{
		// if the pointer is pointing to the Checksum low data byte which resides
		// after 0x80, rSize, Core Command.
		checksum = dataByte;
		rxdataPointer++;
	}
	else if (rxdataPointer == (BSL430_BufferSize + 4))
	{
		// if the pointer is pointing to the Checksum low data byte which resides
		// after 0x80, rSize, Core Command, CKL.
		checksum = checksum | dataByte << 8;
		if (verifyData(checksum))
		{
			sendByte(ACK);
			RX_StatusFlags = DATA_RECEIVED;
		}
		else
		{
			sendByte(CHECKSUM_INCORRECT);
			RX_StatusFlags = RX_ERROR_RECOVERABLE;
		}
	}
	else
	{
		RAM_RX_Buf[rxdataPointer - 3] = dataByte;
		rxdataPointer++;
	}

}

/*******************************************************************************
 * *Function:    verifyData
 * *Description: verifies the data in the data buffer against a checksum
 * *Parameters:
 *           int checksum    the checksum to check against
 * *Returns:
 *           1 checksum parameter is correct for data in the data buffer
 *           0 checksum parameter is not correct for the data in the buffer
 *******************************************************************************/

char verifyData(int checksum)
{
	int i;

	CRCINIRES = 0xFFFF;
	for (i = 0; i < BSL430_BufferSize; i++)
	{
		CRCDIRB_L = RAM_RX_Buf[i];
	}
	return (CRCINIRES == checksum);
}

/*******************************************************************************
 * *Function:    sendByte
 * *Description: Puts a single byte in the outgoing buffer, such as an error
 *
 * *Parameters:
 *           char data    the byte to send
 *******************************************************************************/

void sendByte(char data)
{

	byStatusByte = data;

	RAM_TX_Buf[0] = data; //Status Byte ACK/NACK
	transmitBufferLength = 1;

}

/*******************************************************************************
 * *Function:    getBufferSize
 * *Description: Returns the max Data Buffer Size for this PI
 * *Returns:     max buffer size
 *******************************************************************************/

int PI_getBufferSize()
{
	return MAX_BUFFER_SIZE;
}

/*******************************************************************************
 * *Function:    PI_sendData
 * *Description: Formats the Transmit Buffer to be used when the Transmit Flag gets set
 * *Returns:     None
 *******************************************************************************/

void PI_sendData(int size)
{
	int i;

	RAM_TX_Buf[0] = byStatusByte; //ACK

	RAM_TX_Buf[1] = 0x80;
	RAM_TX_Buf[2] = size & 0xFF;
	RAM_TX_Buf[3] = size >> 8 & 0xFF;
	CRCINIRES = 0xFFFF;
	for (i = 0; i < size; i++){
		CRCDIRB_L = BSL430_SendBuffer[i];
	}
	i = CRCINIRES;

	RAM_TX_Buf[4 + size] = i & 0xFF;
	RAM_TX_Buf[5 + size] = i >> 8 & 0xFF;
	transmitBufferLength = size + 6;
}

