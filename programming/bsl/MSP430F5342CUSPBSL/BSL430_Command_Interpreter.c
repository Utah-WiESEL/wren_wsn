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

#include "BSL430_Command_Interpreter.h"
#include "BSL430_Command_Definitions.h"
#include "BSL430_API.h"
#include "BSL430_PI.h"
#include "BSL_Device_File.h"

void interpretCommand();
void receiveDataBlock(unsigned long addr, char* data, char fastWrite);
void receivePassword();
void eraseLocation(char block_erase, unsigned long addr);
void CRC_Check(unsigned long addr, unsigned int length);
void sendDataBlock(unsigned long addr, unsigned int length);
void sendMessage(char message);

extern char* BSL430_ReceiveBuffer;
extern char* BSL430_SendBuffer;
extern unsigned int BSL430_BufferSize;

// DEBUG
extern void sendByte(char data);

//const unsigned char BSL430_Vendor_Version @ "BSL430_VERSION_VENDOR" = 0x00;
//const unsigned char BSL430_CI_Version @ "BSL430_VERSION_CI" = 0x07;

#pragma DATA_SECTION(BSL430_Version, "BSL430_Version_SEG")
const unsigned char BSL430_Version [4] = { 0x0C, 0x90, 0x10, 0x27 };

/*******************************************************************************
 * *Function:    main
 * *Description: Initializes the BSL Command Interpreter and begins listening for
 *             incoming packets
 *******************************************************************************/
//#pragma required=BSL430_Vendor_Version
//#pragma required=BSL430_CI_Version

void main()
{
    volatile unsigned char eventFlags = 0;

    //SYSBSLC &= ~SYSBSLPE;

    BSL430_API_init();

    PI_init();

    //_disable_interrupts();

    while(1)
    {
      eventFlags = PI_receivePacket();
      if( eventFlags &  DATA_RECEIVED )
      {
        interpretCommand();
      }
    }
}

/*******************************************************************************
 * *Function:    interpretCommand
 * *Description: Interprets the command contained in the data buffer and
 *             calls the appropriate BSL api function
 *******************************************************************************/

void interpretCommand()
{
	unsigned char command = BSL430_ReceiveBuffer[0];
	unsigned long addr = BSL430_ReceiveBuffer[1];

	addr |= ((unsigned long)BSL430_ReceiveBuffer[2]) << 8;
	addr |= ((unsigned long)BSL430_ReceiveBuffer[3]) << 16;
	switch (command)
	{
#ifdef SUPPORTS_RX_DATA_BLOCK_FAST
	case RX_DATA_BLOCK_FAST:
		receiveDataBlock(addr, &BSL430_ReceiveBuffer[4], 1);
		break;
#endif
/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_RX_PASSWORD
	case RX_PASSWORD:                 // Receive password
		receivePassword();
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_LOAD_PC
	case LOAD_PC:                     // Load PC
		sendMessage(BSL430_callAddress(addr));
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_RX_DATA_BLOCK
	case RX_DATA_BLOCK:               // Receive data block
		receiveDataBlock(addr, &BSL430_ReceiveBuffer[4], 0);
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_MASS_ERASE
	case MASS_ERASE:                  // Mass Erase
		sendMessage(BSL430_massErase());
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_ERASE_SEGMENT
	case ERASE_SEGMENT:               // Erase segment
		eraseLocation(0, addr);
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_TOGGLE_INFO
	case TOGGLE_INFO:                 // Toggle Info lock
		sendMessage(BSL430_toggleInfoLock());
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_ERASE_BLOCK
	case ERASE_BLOCK:                 // Erase Block
		eraseLocation(1, addr);
		break;
#endif
		/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_CRC_CHECK
	case CRC_CHECK:                   // CRC Check
	{
		unsigned int length;
		length = BSL430_ReceiveBuffer[4];
		length |= ((unsigned long)BSL430_ReceiveBuffer[5]) << 8;
		CRC_Check(addr, length);
	}
	break;
#endif
	/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_TX_DATA_BLOCK
	case TX_DATA_BLOCK:               // Transmit Data Block
	{
		unsigned int length;
		length = BSL430_ReceiveBuffer[4];
		length |= BSL430_ReceiveBuffer[5] << 8;
		sendDataBlock(addr, length);
	}
	break;
#endif
	/*----------------------------------------------------------------------------*/
#ifdef SUPPORTS_TX_BSL_VERSION
	case TX_BSL_VERSION:              // Transmit BSL Version array
		sendDataBlock((unsigned long)(BSL430_Version), 4);
		break;
#endif
		/*----------------------------------------------------------------------------*/
	default:                          // unknown command
		sendMessage(UNKNOWN_COMMAND);
	}
}

/*******************************************************************************
 * *Function:    receiveDataBlock
 * *Description: Calls the API to write a data block to a given memory location
 * *Parameters:
 *           unsigned long addr  Address to which the data should be written
 *           char* data          Pointer to an array of data to write
 *           char  FastWrite     If 1, do not send a reply message
 *******************************************************************************/

void receiveDataBlock(unsigned long addr, char* data, char fastWrite)
{
#ifndef RAM_WRITE_ONLY_BSL
	char returnValue;
	returnValue = BSL430_openMemory();
	if ((returnValue == SUCCESSFUL_OPERATION) & (BSL430_BufferSize > 4))
#endif
	{
#ifndef RAM_WRITE_ONLY_BSL
		returnValue = BSL430_writeMemory(addr, BSL430_BufferSize - 4, data);
#else
		BSL430_writeMemory(addr, BSL430_BufferSize - 4, data);
#endif
	}
#ifndef RAM_WRITE_ONLY_BSL
	if (!fastWrite)
	{
		sendMessage(returnValue);
	}
	BSL430_closeMemory();
#endif
}

/*******************************************************************************
 * *Function:    receivePassword
 * *Description: Calls the API to unlock the BSL
 *******************************************************************************/

void receivePassword()
{
	if (BSL430_unlock_BSL(&BSL430_ReceiveBuffer[1]) == SUCCESSFUL_OPERATION)
	{
		sendMessage(ACK);
	}
	else
	{
		sendMessage(BSL_PASSWORD_ERROR);
	}
}

/*******************************************************************************
 * *Function:    eraseSegment
 * *Description: Calls the API to erase a segment containing the given address
 * *Parameters:
 *           char block_erase      If 1, the entire block should be erased
 *           unsigned long addr    Address in the segment to be erased.
 *******************************************************************************/

void eraseLocation(char block_erase, unsigned long addr)
{
	sendMessage(BSL430_eraseLocation(block_erase, addr));
}

/*******************************************************************************
 * *Function:    CRC_Check
 * *Description: Calls the API to perform a CRC check over a given addr+length of data
 * *Parameters:
 *           unsigned long addr   Start Addr of CRC
 *           unsigned int length  number of bytes in the CRC
 *******************************************************************************/

void CRC_Check(unsigned long addr, unsigned int length)
{
	int crcResult;
	int crcStatus;

	crcStatus = BSL430_crcCheck(addr, length, &crcResult);
	if (crcStatus == BSL_LOCKED)
	{
		sendMessage(BSL_LOCKED);
	}
	else
	{
		BSL430_SendBuffer[0] = BSL_DATA_REPLY;
		BSL430_SendBuffer[1] = (char)(crcResult & 0xFF);
		BSL430_SendBuffer[2] = (char)((crcResult >> 8) & 0xFF);
		PI_sendData(3);
	}
}

/*******************************************************************************
 * *Function:    sendDataBlock
 * *Description: Fills the SendBuffer array with bytes from the given parameters
 *             Sends the data by calling the PI, or sends an error
 * *Parameters:
 *           unsigned long addr    The address from which to begin reading the block
 *           int length            The number of bytes to read
 *******************************************************************************/

void sendDataBlock(unsigned long addr, unsigned int length)
{
	unsigned long endAddr = addr + length;
	unsigned int bytes;
	char returnValue = SUCCESSFUL_OPERATION;

	while ((addr < endAddr) & (returnValue == SUCCESSFUL_OPERATION))
	{
		if ((endAddr - addr) > PI_getBufferSize() - 1)
		{
			bytes = PI_getBufferSize() - 1;
		}
		else
		{
			bytes = (endAddr - addr);
		}
		returnValue = BSL430_readMemory(addr, bytes, &BSL430_SendBuffer[1]);
		if (returnValue == SUCCESSFUL_OPERATION)
		{
			BSL430_SendBuffer[0] = BSL_DATA_REPLY;
			PI_sendData(bytes + 1);
		}
		else
		{
			sendMessage(returnValue);
		}
		addr += bytes;
	}
}

/*******************************************************************************
 * *Function:    sendMessage
 * *Description: Sends a Reply message with attached information
 * *Parameters:
 *           char message    the message to send
 *******************************************************************************/

void sendMessage(char message)
{
	BSL430_SendBuffer[0] = BSL_MESSAGE_REPLY;
	BSL430_SendBuffer[1] = message;
	PI_sendData(2);
}
