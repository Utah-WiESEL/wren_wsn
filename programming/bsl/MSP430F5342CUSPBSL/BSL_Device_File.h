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

#include "msp430f5342.h"
#include <intrinsics.h>

//Device Specific Definitions
#define MASS_ERASE_DELAY 0x8000
#define INTERRUPT_VECTOR_START 0xFFE0
#define INTERRUPT_VECTOR_END   0xFFFF
#define SECURE_RAM_START 0x1C00

#define TX_PORT_SEL P3SEL
#define TX_PORT_DIR P3DIR
#define RX_PORT_SEL P3SEL
#define RX_PORT_DIR P3DIR
#define RXD       BIT1                      // RXD on P3.1 - I2C CLK
#define TXD       BIT0                      // TXD on P3.0 - I2C Data

#define DCO_SPEED 8000000
#define ACLK_SPEED 32768

#define UCZNCTL1	UCB0CTL1
#define UCZNCTL0	UCB0CTL0
#define UCZNI2COA	UCB0I2COA
#define UCZNIE		UCB0IE
#define UCZIV		UCB0IV
#define UCZRXBUF	UCB0RXBUF
#define UCZTXBUF	UCB0TXBUF

//Device Specific Definitions for commands and bugs
#define FULL_FLASH_BSL
#define DO_NOT_CHECK_VPE

// standard command includes
#ifdef RAM_WRITE_ONLY_BSL
#define SUPPORTS_RX_DATA_BLOCK_FAST
#define SUPPORTS_RX_PASSWORD
#define SUPPORTS_LOAD_PC
#endif

#ifdef FULL_FLASH_BSL
#define FULL_GENERIC_COMMANDS
#define FLASH_COMMANDS
#endif

#ifdef FULL_FRAM_BSL
#define FULL_GENERIC_COMMANDS
#endif

#ifdef FULL_GENERIC_COMMANDS
#define SUPPORTS_RX_DATA_BLOCK_FAST
#define SUPPORTS_RX_PASSWORD
#define SUPPORTS_LOAD_PC
#define SUPPORTS_RX_DATA_BLOCK
#define SUPPORTS_MASS_ERASE
#define SUPPORTS_CRC_CHECK
#define SUPPORTS_TX_DATA_BLOCK
#define SUPPORTS_TX_BSL_VERSION
#endif

#ifdef FLASH_COMMANDS
#define SUPPORTS_ERASE_SEGMENT
#define SUPPORTS_TOGGLE_INFO
#endif
