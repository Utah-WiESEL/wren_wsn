;**************************************************************
;Copyright (c) 2013, University of Utah - Electrical and Computer Engineering - WiESEL
;All rights reserved.
;
;Redistribution and use in source and binary forms, with or without
;modification, are permitted provided that the following conditions are met:
;   * Redistributions of source code must retain the above copyright
;notice, this list of conditions and the following disclaimer.
;   * Redistributions in binary form must reproduce the above copyright
;notice, this list of conditions and the following disclaimer in the
;documentation and/or other materials provided with the distribution.
;   * Neither the name of the WiESEL nor the
;names of its contributors may be used to endorse or promote products
;derived from this software without specific prior written permission.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
;ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
;WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;DISCLAIMED. IN NO EVENT SHALL WiESEL BE LIABLE FOR ANY
;DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
;(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
;ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
;SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

;**************************************************************
; Anh Luong
; 06/25/2012
;**************************************************************

;**************************************************************
; BSL SW low level functions
;**************************************************************

    .cdecls C,LIST,"msp430f5342.h"
    .cdecls C,LIST,"BSL430_Command_Interpreter.h"

ARG1 .equ R12
ARG2 .equ R13
ARG3 .equ R14
ARG4 .equ R15

RET_low .equ R12
RET_high .equ r13

 .ref _c_int00
 ;.ref bsl

;--------------------------------------------------------------
 .sect "ZAREA"
;--------------------------------------------------------------
BSL_Entry_JMP
			  BR	#_c_int00
			 ;BR	#bsl

 .sect "ZAREA_CODE"
 ;**************************************************************
; Name       :RETURN_TO_BSL
; Function   :Returns to a BSL function after that function has made
;            :an external function call
; Arguments  none
; Returns    :none
;**************************************************************
RETURN_TO_BSL
              POP.W    RET_low                 ; remove first word from return addr
              POP.W    RET_high                ; remove second word from return addr

              RETA                             ; should now return to the BSL location

;**************************************************************
; Name       :BSL_Protect
; Function   :Protects the BSL memory and protects the SYS module
; Arguments  :none
; Returns    :0 in R12.0 for lock (keep JTAGLOCK_KEY state)
;            :1 in R12.0 for unlock (overwrite JTAGLOCK_KEY) : BSL_REQ_JTAG_OPEN
;            :0 in R12.1 for no appended call
;            :1 in R12.1 for appended call via BSLENTRY : BSL_REQ_APP_CALL
;**************************************************************
BSL_REQ_JTAG_OPEN  .equ  0x0001                  ;Return Value for BSLUNLOCK Function to open JTAG
BSL_REQ_APP_CALL   .equ  0x0002                  ;Return Value for BSLUNLOCK Function to Call BSL again
BSL_Protect     
              CLR      RET_low                  ;lock (keep JTAGLOCK_KEY state)
             
              BIC     #SYSBSLPE+SYSBSLSIZE0+SYSBSLSIZE1, &SYSBSLC ; protects BSL
              ;BIC     #BSL_REQ_JTAG_OPEN, RET_low   ;lock (keep JTAGLOCK_KEY state)
              BIS     #BSL_REQ_JTAG_OPEN, RET_low   ;make sure it remains open for debugging

              AND.B   #0x00fc,&P3DIR

			  ; check I2C lines are high
			  BIT.B   #1,&P3IN
			  JEQ     BCC2BSL
			  BIT.B   #2,&P3IN
              JEQ     BCC2BSL

			  ;BR	  #bsl
              BIS.W   #BSL_REQ_APP_CALL, RET_low
BCC2BSL       RETA
              
 .sect "BSLSIG"
                 .word       0xFFFF
BslProtectVecLoc .word       BSL_Protect             ; adress of function
PBSLSigLoc       .word       03CA5h                  ; 1st BSL signature
SBSLSigLoc       .word       0C35Ah                  ; 2nd BSL signature
                 .word       0xFFFF					 ; Reserved
BslEntryLoc      .word       BSL_Entry_JMP           ; BSL_Entry_JMP

PJTAGLOCK_KEY    .word       0xFFFF                  ; Primary Key Location
SJTAGLOCK_KEY    .word       0xFFFF                  ; Secondary Key Location
                                                     ; set default unlock JTAG with option to lock with writting
                                                     ; a value <> 0x0000 or 0xFFFF


