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

#include "BSL_Device_File.h"
#include "BSL430_API.h"
#include "BSL430_Command_Definitions.h"

#ifdef RAM_BASED_BSL
#    ifdef RAM_WRITE_ONLY_BSL
#        error Can NOT have RAM write and RAM based BSL
#    endif
#    define DEFAULT_STATE (UNLOCKED)
#else
#    define DEFAULT_STATE (LOCKED)
#endif

volatile unsigned int LockedStatus;

#define API_VERSION (0x04)
#ifdef RAM_WRITE_ONLY_BSL
#    define API_V (API_VERSION + RAM_WRITE_ONLY_BSL)
#else
#    define API_V API_VERSION
#endif

/*******************************************************************************
 * *Function:    __low_level_init
 * *Description: function used to erase the RAM in device and stop WDT
 *******************************************************************************/
#if __VER__ <= 420
void
#else
int
#endif
_system_pre_init(void)
{
#ifndef RAM_BASED_BSL
	register unsigned int* addr;
#endif
	WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
	_disable_interrupts();                // disable interrupts
#ifndef RAM_BASED_BSL
	for (addr = (unsigned int*)SECURE_RAM_START; addr < (unsigned int*)_get_SP_register();
			addr += 1)
	{
		*addr = 0x574C;                   // known pattern for verification of RAM overwrite
	}
#endif
}

/*******************************************************************************
 * *Function:    BSL430_API_init
 * *Description: sets the key for writing to flash,  sets device state
 *******************************************************************************/

void BSL430_API_init()
{
	_system_pre_init();
	LockedStatus = DEFAULT_STATE;
}

/*******************************************************************************
 * *Function:    BSL430_lock_BSL
 * *Description: Locks the BSL
 * *Returns:
 *             SUCCESSFUL_OPERATION  BSL Locked
 *******************************************************************************/

char BSL430_lock_BSL()
{
	LockedStatus = LOCKED;
	return SUCCESSFUL_OPERATION;
}

/*******************************************************************************
 * *Function:    BSL430_unlock_BSL
 * *Description: Causes the BSL to compare the data buffer against the BSL password
 *             BSL state will be UNLOCKED if successful
 ****************Parameters:
 *             char* data            A pointer to an array containing the password
 ****************Returns:
 *             SUCCESSFUL_OPERATION  All data placed into data array successfully
 *             BSL_PASSWORD_ERROR    Correct Password was not given
 *******************************************************************************/

char BSL430_unlock_BSL(char* data)
{
	int i;
	int retValue = 0;
	char *interrupts = (char*)INTERRUPT_VECTOR_START;

	for (i = 0; i <= (INTERRUPT_VECTOR_END - INTERRUPT_VECTOR_START); i++, interrupts++)
	{
		retValue |=  *interrupts ^ data[i];
	}
	if (retValue == 0)
	{
#ifndef RAM_WRITE_ONLY_BSL
		volatile int i;
		for (i = MASS_ERASE_DELAY - 1; i > 0; i--) ;
#endif
		LockedStatus = UNLOCKED;
		return SUCCESSFUL_OPERATION;
	}
	else
	{
		BSL430_massErase();
		return BSL_PASSWORD_ERROR;
	}
}

/*******************************************************************************
 * *Function:    BSL430_toggleInfoLock
 * *Description: Toggles the LOCKA bit for writing/erasing info A segment
 * *Returns:
 *             SUCCESSFUL_OPERATION  Info A is now open for writing or erasing.
 *             BSL_LOCKED            Correct Password has not yet been given
 *******************************************************************************/

char BSL430_toggleInfoLock()
{
	char exceptions;

	if (LockedStatus == UNLOCKED)
	{
		exceptions = SUCCESSFUL_OPERATION;
		FCTL3 = FWKEY + LOCKA + (FCTL3 & LOCK); // toggle LOCKA bit
	}
	else
	{
		exceptions = BSL_LOCKED;
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_openMemory
 * *Description: Unlocks the Flash for writing
 * *Returns:
 *             SUCCESSFUL_OPERATION  Flash is now open for writing.
 *             BSL_LOCKED            Correct Password has not yet been given
 *******************************************************************************/

char BSL430_openMemory()
{
	char exceptions = BSL_LOCKED;

	if (LockedStatus == UNLOCKED)
	{
		exceptions = SUCCESSFUL_OPERATION;
		FCTL3 = FWKEY;                   // Clear Lock bit
		FCTL1 = FWKEY + WRT;             // Set write bit
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_closeMemory
 * *Description: Locks the Flash against writing
 * *Returns:
 *             SUCCESSFUL_OPERATION  Flash is now locked.
 *******************************************************************************/

char BSL430_closeMemory(void)
{
	FCTL1 = FWKEY;                       // Clear WRT bit
	FCTL3 = FWKEY + LOCK;                // Set LOCK bit

	return SUCCESSFUL_OPERATION;
}

/*******************************************************************************
 * *Function:    BSL430_readMemory
 * *Description: Reads an array of bytes from memory into a supplied array
 * *Parameters:
 *             unsigned long addr    The address from which the read should begin
 *             char length           The amount of bytes to read
 *             char* data            The array into which the data will be saved
 ****************Returns:
 *             SUCCESSFUL_OPERATION  All Data placed into data array successfully
 *             BSL_LOCKED            Correct Password has not yet been given
 *******************************************************************************/

char BSL430_readMemory(unsigned long addr, unsigned int length, char* data)
{
	unsigned long i;
	char exceptions = SUCCESSFUL_OPERATION;

	for (i = addr + length - 1; i >= addr; i--)
	{
		if (LockedStatus == UNLOCKED)
		{
			data[i - addr] = _data20_read_char(i);
		}
		else
		{
			return BSL_LOCKED;
		}
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_crcCheck
 * *Description: return a CRC check on the memory specified
 * *Parameters:
 *           unsigned long addr    The address from which to start the check
 *           int length            The length of the data area to check
 *           int* return           variable in which to put the return value
 ****************Returns:
 *           SUCCESSFUL_OPERATION  CRC check done correctly
 *           BSL_LOCKED            Correct Password has not yet been given
 *******************************************************************************/

int BSL430_crcCheck(unsigned long addr, unsigned int length, int* result)
{
	unsigned long i;

	CRCINIRES = 0xFFFF;
	for (i = addr; i < addr + length; i++)
	{
		if (LockedStatus == UNLOCKED)
		{
			CRCDIRB_L = _data20_read_char(i);
		}
		else
		{
			return BSL_LOCKED;
		}
	}
	*result = CRCINIRES;
	return SUCCESSFUL_OPERATION;

}

/*******************************************************************************
 * *Function:    BSL430_callAddress
 * *Description: Loads the Program Counter with the supplied address
 * *Parameters:
 *           unsigned long addr    The address to which the function call should go
 ****************Returns:
 *           SUCCESSFUL_OPERATION  Called location has returned
 *           BSL_LOCKED            Correct Password has not yet been given
 ****************Note:
 *           If successful, this function does not return.
 *******************************************************************************/

char BSL430_callAddress(unsigned long addr)
{
	if (LockedStatus == UNLOCKED)
	{
		((void (*)())addr)();               // type cast addr to function ptr, call
		return SUCCESSFUL_OPERATION;
	}
	return BSL_LOCKED;                      // can only be reached if BSL is locked
}

/*******************************************************************************
 * *Function:    BSL430_writeMemory
 * *Description: Writes a byte array starting at a given address.
 *             Note: the function will write in word mode if possible
 *             (when start address is even)
 ****************Parameters:
 *           unsigned long startAddr        The address to which the write should begin
 *           int size                       The number of bytes to write
 *           char* data                     The array of bytes to write (must be even aligned)
 ****************Returns:
 *           SUCCESSFUL_OPERATION           Bytes written successfully
 *           MEMORY_WRITE_CHECK_FAILED      A byte in data location post-write does not match data
 ***************parameter
 *                                          Note: write stops immediatly after a byte check fails
 *           BSL_LOCKED                     Correct Password has not yet been given
 *           VOLTAGE_CHANGE_DURING_PROGRAM  Voltage changed during write (of a single byte/word)
 *******************************************************************************/

char BSL430_writeMemory(unsigned long startAddr, unsigned int size,  char* data)
{
	unsigned long i;
	char exceptions = SUCCESSFUL_OPERATION;

	for (i = startAddr; i < startAddr + size; i++)
	{
#ifndef RAM_WRITE_ONLY_BSL
		if ((startAddr & 0x01) || i == startAddr + size - 1)
#endif
		{
			exceptions = BSL430_writeByte(i, *data);
			data += 1;
		}
#ifndef RAM_WRITE_ONLY_BSL
		else
		{
			exceptions = BSL430_writeWord(i, *(int *)data);
			data += 2;
			i++;
		}
		if (exceptions != SUCCESSFUL_OPERATION)
		{
			return exceptions;
		} // if
#endif
	}     // for
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_writeByte
 * *Description: Writes a byte at a given address
 * *Parameters:
 *           unsigned long addr             The address to which the byte should be written
 *           char data                      The byte to write
 ****************Returns:
 *           SUCCESSFUL_OPERATION           Byte written successfully
 *           MEMORY_WRITE_CHECK_FAILED      Byte in data location post-write does not match data
 ***************parameter
 *           VOLTAGE_CHANGE_DURING_PROGRAM  Voltage changed during write
 *           BSL_LOCKED                     Correct Password has not yet been given
 *******************************************************************************/

char BSL430_writeByte(unsigned long addr, char data)
{
	char exceptions;

	if (LockedStatus == UNLOCKED)
	{
		exceptions = SUCCESSFUL_OPERATION;
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		_data20_write_char(addr, data);
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		if (data != _data20_read_char(addr))
		{
			exceptions = MEMORY_WRITE_CHECK_FAILED;
		}
#ifndef RAM_WRITE_ONLY_BSL
#    ifndef DO_NOT_CHECK_VPE
		if (FCTL4 & VPE)
		{
			exceptions = VOLTAGE_CHANGE_DURING_PROGRAM;
		}
#    endif
#endif
	}
	else
	{
		exceptions = BSL_LOCKED;
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_writeWord
 * *Description: Writes a word at a given address
 * *Parameters:
 *           unsigned long addr             The address to which the word should be written
 *           int data                       The byte to write
 ****************Returns:
 *           SUCCESSFUL_OPERATION           Word written successfully
 *           MEMORY_WRITE_CHECK_FAILED      Word in data location post-write does not match data
 ***************parameter
 *           VOLTAGE_CHANGE_DURING_PROGRAM  Voltage changed during write
 *           BSL_LOCKED                     Correct Password has not yet been given
 *******************************************************************************/

char BSL430_writeWord(unsigned long addr, int data)
{
	char exceptions;

	if (LockedStatus == UNLOCKED)
	{
		exceptions = SUCCESSFUL_OPERATION;
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		_data20_write_short(addr, data);
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		if (data != _data20_read_short(addr))
		{
			exceptions = MEMORY_WRITE_CHECK_FAILED;
		}
#ifndef DO_NOT_CHECK_VPE
		if (FCTL4 & VPE)
		{
			exceptions = VOLTAGE_CHANGE_DURING_PROGRAM;
		}
#endif
	}
	else
	{
		exceptions = BSL_LOCKED;
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_eraseLocation
 * *Description: Erases a memory segment which contains a given address
 * *Parameters:
 *           char block_erase               currently unused 1: erase block 0: erase segment
 *           unsigned long addr             An address which is within the segment to be erased
 ****************Returns:
 *           SUCCESSFUL_OPERATION           Segment erased
 *           BSL_LOCKED                     Correct Password has not yet been given
 *******************************************************************************/

char BSL430_eraseLocation(char block_erase, unsigned long addr)
{
	char exceptions;

	if (LockedStatus == UNLOCKED)
	{
		exceptions = SUCCESSFUL_OPERATION;
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		FCTL3 = FWKEY;                       // Clear Lock bit
		FCTL1 = FWKEY + ERASE;               // Set Erase bit
		_data20_write_char(addr, 0);           // Dummy write to erase Flash seg
#ifdef RAM_BASED_BSL
		while (FCTL3 & BUSY) ;
#endif
		FCTL3 = FWKEY + LOCK;                // Set LOCK bit
	}
	else
	{
		exceptions = BSL_LOCKED;
	}
	return exceptions;
}

/*******************************************************************************
 * *Function:    BSL430_massErase
 * *Description: Mass erases the entire MSP430 device
 * *Returns:
 *           SUCCESSFUL_OPERATION           Flash erased
 *           BSL_LOCKED                     Correct Password has not yet been given
 *******************************************************************************/

char BSL430_massErase()
{
	char exceptions = SUCCESSFUL_OPERATION;
	volatile char *Flash_ptr;                   // Flash pointer

#ifdef RAM_BASED_BSL
	while (FCTL3 & BUSY) ;
#endif
	FCTL3 = FWKEY;
#ifdef RAM_BASED_BSL
	while (FCTL3 & BUSY) ;
#endif
	Flash_ptr = (char *)INTERRUPT_VECTOR_START; // Initialize Flash pointer
	FCTL1 = FWKEY + MERAS + ERASE;           // Set Mass Erase bit
	*Flash_ptr = 0;                             // Dummy write to erase main flash
#ifdef RAM_BASED_BSL
	while (FCTL3 & BUSY) ;
#endif
	FCTL3 = FWKEY + LOCK;                    // Set LOCK bit

	return exceptions;
}

