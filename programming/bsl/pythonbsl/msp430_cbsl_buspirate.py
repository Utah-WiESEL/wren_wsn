#!/usr/bin/env python
# encoding: utf-8
"""
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
"""

########################################################################
### I2C Bootstrap Loader software for the msp430 embedded proccessor
###
### Anh Luong
### Based on Chris Liechti cc430 2010-04-11
### fixes from Colin Domoney
### cc430 port by Mark Hays 2010-04-11
### Based on tos-bsl version 1.39-telos-8
###

#'''exec' python $0 ${1+"$@"}
#'''
#'

import os, sys, time, string
#sys.path.append("/usr/lib/tinyos")
#//sys.path.append(os.environ.get("OSIAN_ROOT", "/opt/osian") + "/lib/python")
import serial
from I2C import *

VERSION = "CUSP BSL Programmer v0.0.1 2012-06-25"

# Maximum number of times to retry programming until we fail permanently
MAXIMUM_ATTEMPTS = 1

DEBUG = 3

ADDRESS = 0x48

def debug(level, fmt, *rest):
    "print debugging message"
    if DEBUG < level:
        return
    if rest:
        fmt = fmt % rest
    print >>sys.stderr, fmt.rstrip()
    sys.stderr.flush()

def hexl(items):
    "convert binary data to hex bytes for display"
    if not items:
        return "<None>"
    c = items[0]
    if isinstance(c, str) and len(c) == 1:
        ### convert chars to hex
        items = ["%02x" % ord(x) for x in items]
    elif isinstance(c, int):
        ### convert ints to hex
        items = ["%02x" % x for x in items]
    else:
        assert isinstance(c, str) and len(c) > 1
    return " ".join(items)

def i2caddr(addr, write=True):
    "convert standard address to 7 bit i2c address"
    if write:
        return ((addr << 1))
    else:
        return ((addr << 1) + 1)

### base exc
class BSLException(Exception):     pass

### serial port timeout
class BSLTimeout(BSLException):    pass

### BSL responses for txFrame
class BadHeader(BSLException):     pass
class BadChecksum(BSLException):   pass
class EmptyPacket(BSLException):   pass
class PacketTooLong(BSLException): pass
class UnknownError(BSLException):  pass
class BadBaudRate(BSLException):   pass
class BadResponse(BSLException):   pass # default for garbage resp

### serial proto errors for rxFrame
class BadRespHdr(BSLException):    pass
class BadRespLen(BSLException):    pass
class BadRespSum(BSLException):    pass
class BadRespCode(BSLException):   pass
class GarbledResp(BSLException):   pass

### BSL response messages for rxFrame
class BadFlashWrite(BSLException): pass
class FlashFail(BSLException):     pass
class VoltageChange(BSLException): pass
class Locked(BSLException):        pass
class BadPassword(BSLException):   pass
class BadByteWrite(BSLException):  pass
class UnknownCmd(BSLException):    pass
class Overrun(BSLException):       pass
class BadMessage(BSLException):    pass # default for garbage resp

### BSL response code in txFrame
XCODE = {
    0x51: BadHeader,
    0x52: BadChecksum,
    0x53: EmptyPacket,
    0x54: PacketTooLong,
    0x55: UnknownError,
    0x56: BadBaudRate,
}

### BSL response error in rxFrame
MCODE = {
    0x01: BadFlashWrite,
    0x02: FlashFail,
    0x03: VoltageChange,
    0x04: Locked,
    0x05: BadPassword,
    0x06: BadByteWrite,
    0x07: UnknownCmd,
    0x08: Overrun,
}

class LowLevel:
    "low level communication"

    DEFAULT_TIMEOUT = 0.5

    MAX_DATA_BYTES  = 240

    MAX_PI_SIZE = 256

    def __init__(self, aTimeout = None):
        """init bsl object, don't connect yet"""
        if aTimeout is None:
            self.timeout = self.DEFAULT_TIMEOUT
        else:
            self.timeout = aTimeout
        self.i2c  = None

        # max data size in txFrame
        self.maxData = self.MAX_DATA_BYTES

        # are we running BSL right now?
        self.inBSL = False

        self.address = ADDRESS

    def calcChecksum(self, data):
        """Calculates a CCITT CRC-16 checksum of data."""
        if not isinstance(data, str):
            data = "".join([chr(x) for x in data])
        crc = 0xffff
        for c in data:
            byte = ord(c)
            ### OVTA implementation
            ### See:
            ###   Efficient High Hamming Distance CRCs for Embedded Networks
            ###   Justin Ray and Philip Koopman
            ###   www.ece.cmu.edu/~koopman/pubs/ray06_crcalgorithms.pdf
            crc  = ((crc >> 8) | (crc << 8)) & 0xffff
            crc ^= byte
            crc ^= (crc & 0xf0) >> 4
            crc ^= (crc & 0x0f) << 12
            crc ^= (crc & 0xff) << 5
        return crc

    ### XXX everything from here up to txFrame was lifted from tos-bsl...
    ### XXX more or less

    def comInit(self, port):
        """
        Tries to open the serial port given and
        initialises the port and variables.
        """
        # Startup-Baudrate: 115200, 8, N, 1, timeout
        debug(2, "comInit port %s, 115200 8N1, timeout %s", port, self.timeout)

        # Init I2C object
        i2c = I2C(port, 115200, self.timeout)

        # Start Bit Banging Mode
        print "Entering binmode: ",
        if i2c.BBmode():
            print "OK."
        else:
            print "failed."
            sys.exit()

        # Start I2C
        print "Entering raw I2C mode: ",
        if i2c.enter_I2C():
            print "OK."
        else:
            print "failed."
            sys.exit()

        print "Configuring I2C."
        if not i2c.cfg_pins(I2CPins.POWER | I2CPins.PULLUPS):
            print "Failed to set I2C peripherals."
            sys.exit()
        if not i2c.set_speed(I2CSpeed._400KHZ):
            print "Failed to set I2C Speed."
            sys.exit()
        i2c.timeout(0.2)

        debug(2, "/comInit")

        self.i2c = i2c

    def close(self):
        print "Reset Bus Pirate to user terminal: "
        if self.i2c.resetBP():
            print "OK."
        else:
            print "failed."
            sys.exit()

    def __del__(self):
        #self.close()
        return

    def i2c_write_data(self, data, doResp=True):
        error = 0
        self.i2c.send_start_bit()
        for i in data:
            status = self.i2c.bulk_trans(1, [ord(i)])
            if status.find(chr(0x01)) != -1:
                print i + " NACK"
                error = 1
        self.i2c.send_stop_bit()
        if not doResp:
            debug(3, "/txFram !doResp")
            return
        if error:
            print "I2C command not acknowledged!"
            print [hex(ord(x)) for x in data]
            print "Reset Bus Pirate to user terminal: "
            if self.i2c.resetBP():
                print "OK."
            else:
                print "failed."
            raise BSLTimeout
        else:
            status = ord(status)
            if status:
                klass = XCODE.get(status, BadResponse)
                debug(3, "/txFrame resp %02x %s", status, klass.__name__)
                raise klass, status
            else:
                debug(3, "/txFrame ACK")

    def i2c_read_bytes(self, numbytes):
        data_out=[]
        while numbytes > 0:
            data_out.append(ord(self.i2c.read_byte()))
            self.i2c.send_ack()
            numbytes-=1
        return data_out

    def txFrame(self, data, doResp=True):
        "send data to BSL"
        if not isinstance(data, str):
            data = "".join([chr(x) for x in data])
        n = len(data)
        assert n <= self.MAX_PI_SIZE
        s = self.calcChecksum(data)
        f = chr(i2caddr(self.address)) + chr(0x80) + chr(n & 0xff) + chr(n >> 8) + data + chr(s & 0xff) + chr(s >> 8)
        debug(3, "txFrame %s", hexl(f))
        self.i2c_write_data(f)
        if not doResp:
            debug(3, "/txFrame !doResp")
            return

    def rxFrame(self):
        "get response from BSL"
        "send start bit"
        self.i2c.send_start_bit()
        self.i2c.bulk_trans(1, [i2caddr(self.address, False)])
        "receive first 4 bytes"
        h = self.i2c_read_bytes(4)
        if len(h) != 4:
            raise BSLTimeout
        debug(3, "rxFrame Head %s", hexl(h))
        if h[0] != 0x00:
            raise BadRespHdr, h[0]
        if h[1] != 0x80:
            raise BadRespHdr, h[1]
        n = (h[3] << 8) | h[2]
        if not n:
            raise BadRespLen
        "receive data"
        p = self.i2c_read_bytes(n)
        debug(3, "rxFrame Data %d: %s", n, hexl(p))
        if len(p) != n:
            raise BadRespLen
        "checksum"
        s = self.i2c_read_bytes(2)
        if len(s) != 2:
            raise BadRespSum, s
        s = (s[1] << 8) | s[0]
        debug(3, "rxFrame Sum %04x", s)
        "send nack for stopping"
        self.i2c.send_nack()
        "send stop bit"
        self.i2c.send_stop_bit()
        if s != self.calcChecksum(p):
            raise BadRespSum
        if not isinstance(p[0], int):
            c = ord(p[0])
        else:
            c = p[0]
        if c == 0x3a:
             p = p[1:]
             debug(3, "/rxFrame resp %s", hexl(p))
        elif c == 0x3b:
            if len(p) != 2:
                raise GarbledResp, p
            if not isinstance(p[1], int):
                c = ord(p[1])
            else:
                c = p[1]
            if c:
                klass = MCODE.get(c, BadMessage)
                debug(3, "/rxFrame Msg %02x %s", c, klass.__name__)
                raise klass
            else:
                debug(3, "/rxFrame Msg SUCCESS")
                return
        else:
            raise BadRespCode, c
        return p

class BSL(LowLevel):
    "Implements (most) BSL commands using txFrame and rxFrame"

    CMD_BAUD_RATE     = 0x52 # x Won't implement
    CMD_RX_DATA       = 0x10 # - Implemented
    CMD_RX_DATA_FAST  = 0x1b # - Implemented
    CMD_RX_PASSWD     = 0x11 # - Implemented
    CMD_ERASE_SEGMENT = 0x12 # - Implemented
    CMD_UNLOCK_INFO   = 0x13 # - Implemented
    CMD_MASS_ERASE    = 0x15 # - Implemented
    CMD_CRC_CHECK     = 0x16 # x Won't implement
    CMD_LOAD_PC       = 0x17 # - Implemented, RETA doesn't seem to return...
    CMD_TX_DATA       = 0x18 # - Implemented
    CMD_BSL_VERSION   = 0x19 # - Implemented
    CMD_TX_BUF_SIZE   = 0x1a # ? Doesn't work


    def __init__(self, *rest, **kw):
        LowLevel.__init__(self, *rest, **kw)
        self.fastrx = False
        self.passwd = None

    def massErase(self):
        "erase program flash"
        assert self.inBSL
        debug(2, "massErase")
        self.txFrame([self.CMD_MASS_ERASE])
        assert self.rxFrame() is None
        debug(3, "/massErase")

    def txPassword(self, passwd):
        "send BSL password"
        # XXX needs test, works after massErase
        assert self.inBSL
        passwd = passwd or self.passwd or ([0xff] * 32)
        debug(2, "txPassword %s", hexl(passwd))
        if not isinstance(passwd, str):
            passwd = "".join([chr(x) for x in passwd])
        self.txFrame(chr(self.CMD_RX_PASSWD) + passwd)
        assert self.rxFrame() is None
        debug(3, "/txPassword")

    def txBslVersion(self):
        "print out BSL version info"
        assert self.inBSL
        debug(2, "txBslVersion")
        try:
            self.txFrame(chr(self.CMD_BSL_VERSION))
            resp = self.rxFrame()
        except BSLTimeout:
            raise
        except BSLException:
            resp = ""
        if len(resp) != 4:
            debug(0, "BSL Version garbled or unimplemented, continuing anyway")
        else:
            #vid, ver, api, pi = [ord(x) for x in resp]
            vid, ver, api, pi = resp
            if vid == 0x00:
                vid = "Vendor %s(%02x)" % ("TI", vid)
            elif vid == 0x0C:
                vid = "Vendor %s(%02x)" % ("WiESEL", vid)
            else:
                vid = "Vendor %s(%02x)" % ("Unknown", vid)
            ver = "Interp %02x" % ver
            api = "API %02x%s" % (api, api & 0x80 and " Limited" or "")
            if pi == 0x27:
                pi = "PI I2C(%02x)" % pi
            elif pi < 0x20:
                pi = "PI TA_UART(%02x)" % pi
            elif pi < 0x50:
                pi = "PI USB(%02x)" % pi
            elif pi < 0x70:
                pi = "PI USCI_UART(%02x)" % pi
            else:
                pi = "PI Unknown(%02x)" % pi
            debug(0, "BSL Version: %s, %s, %s, %s", vid, ver, api, pi)
        debug(3, "/txBslVersion")

    def txData(self, addr, length):
        "get data from MCU memory"
        assert self.inBSL
        assert 0x000000 <= addr <= 0xffffff
        assert 0x0000 < length <= 0xffff
        ### CMD_TX_BUF_SIZE doesn't seem to work
        ### in practice, 260 seems to be the limit
        ### stick to 0x100 and lower
        assert 0 < length <= 0x100
        debug(2, "txData %04x @ %06x", length, addr)
        self.txFrame([self.CMD_TX_DATA,
                      (addr >>  0) & 0xff,
                      (addr >>  8) & 0xff,
                      (addr >> 16) & 0xff,
                      (length >> 0) & 0xff,
                      (length >> 8) & 0xff])
        ret = self.rxFrame()
        if len(ret) != length:
            raise GarbledResp, (len(ret), ret)
        debug(3, "txData resp %s", hexl(ret))
        debug(3, "/txData")
        return ret

    def _rxData(self, addr, data, slow):
        "write data into MCU memory"
        assert self.inBSL
        assert 0x000000 <= addr <= 0xffffff
        assert isinstance(data, str) and data
        ### CMD_TX_BUF_SIZE doesn't seem to work
        ### in practice, 260 seems to be the limit
        ### stick to 0x100 and lower
        assert 0 < len(data) <= 0x100
        debug(2, "rxData%s %d @ %06x: %s",
              (not slow) and " Fast" or "", len(data), addr, hexl(data))
        head = "".join([chr(slow and self.CMD_RX_DATA or self.CMD_RX_DATA_FAST),
                        chr((addr >>  0) & 0xff),
                        chr((addr >>  8) & 0xff),
                        chr((addr >> 16) & 0xff)])
        self.txFrame(head + data)
        if slow:
            assert self.rxFrame() is None
        debug(3, "/rxData")

    def rxData(self, addr, data):
        "program with verify"
        self._rxData(addr, data, not self.fastrx)

    def eraseSegment(self, addr):
        "erase a segment by address"
        assert self.inBSL
        assert 0x000000 <= addr <= 0xffffff
        debug(2, "eraseSegment %06x", addr)
        self.txFrame([self.CMD_ERASE_SEGMENT,
                      (addr >>  0) & 0xff,
                      (addr >>  8) & 0xff,
                      (addr >> 16) & 0xff])
        assert self.rxFrame() is None
        debug(3, "/eraseSegment")

    def _toggleLOCKA(self):
        "toggle the LOCKA bit"
        debug(3, "toggleLOCKA")
        self.txFrame(chr(self.CMD_UNLOCK_INFO))
        assert self.rxFrame() is None
        debug(3, "/toggleLOCKA")

    def _eraseInfoA(self):
        "erase info segment A"
        assert self.inBSL
        debug(3, "eraseInfoA")
        ### XXX toggling LOCKA doesn't seem to matter??
        self._toggleLOCKA()
        self.eraseSegment(0x001980)
        self._toggleLOCKA()
        debug(3, "/eraseInfoA")

    ### XXX this probably only works for:
    ### XXX   F5133, F5135, F6125, F6135, F6126, F5137, F6127, and F6137
    INFO_ADDRS = {
        "a": None,     # use eraseInfoA() for this one
        "b": 0x001900,
        "c": 0x001880,
        "d": 0x001800,
    }

    def eraseInfo(self, which="abcd"):
        "erase some/all of info segments A-D"
        assert self.inBSL
        assert isinstance(which, str) and which and len(which) <= 4
        d = { }
        for seg in which.lower():
            assert seg in self.INFO_ADDRS
            d.setdefault(seg)
        d = d.keys()
        d.sort()
        debug(2, "eraseInfo segs=%s", "".join(d))
        for seg in d:
            addr = self.INFO_ADDRS[seg]
            if addr is None:
                self._eraseInfoA()
            else:
                debug(2, "Erasing INFO %s", seg)
                self.eraseSegment(addr)
        debug(3, "/eraseInfo")

    def loadPC(self, addr, expectReturn=False):
        "jump to location"
        assert self.inBSL
        assert 0x000000 <= addr <= 0xffffff
        assert not (addr & 1)
        debug(2, "loadPC %06x", addr)
        self.txFrame([self.CMD_LOAD_PC,
                      (addr >>  0) & 0xff,
                      (addr >>  8) & 0xff,
                      (addr >> 16) & 0xff])
        ### XXX a RETA doesn't seem to return to BSL?
        ### XXX OK, just break ourself then
        self.inBSL = False
        debug(3, "/loadPC")

### this class lifted in its entirety from tos-bsl
class Segment:
    """store a string with memory contents along with its startaddress"""
    def __init__(self, startaddress = 0, data=None):
        if data is None:
            self.data = ''
        else:
            self.data = data
        self.startaddress = startaddress

    def __getitem__(self, index):
        return self.data[index]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return "Segment(startaddress = 0x%04x, data=%r)" % (self.startaddress, self.data)

### this class lifted in its entirety from tos-bsl
class Memory:
    """represent memory contents. with functions to load files"""
    def __init__(self, filename=None):
        self.segments = []
        if filename:
            self.filename = filename
            self.loadFile(filename)

    def append(self, seg):
        self.segments.append(seg)

    def __getitem__(self, index):
        return self.segments[index]

    def __len__(self):
        return len(self.segments)

    def loadIHex(self, file):
        """load data from a (opened) file in Intel-HEX format"""
        segmentdata = []
        currentAddr = 0
        startAddr   = 0
        lines = file.readlines()
        for l in lines:
            if l[0] != ':': raise BSLException("File Format Error\n")
            l = l.strip()       #fix CR-LF issues...
            length  = int(l[1:3],16)
            address = int(l[3:7],16)
            type    = int(l[7:9],16)
            check   = int(l[-2:],16)
            if type == 0x00:
                if currentAddr != address:
                    if segmentdata:
                        self.segments.append( Segment(startAddr, "".join(segmentdata)) )
                    startAddr = currentAddr = address
                    segmentdata = []
                for i in range(length):
                    segmentdata.append( chr(int(l[9+2*i:11+2*i],16)) )
                currentAddr = length + currentAddr
            elif type in (0x01, 0x02, 0x03, 0x04, 0x05):
                pass
            else:
                sys.stderr.write("Ignored unknown field (type 0x%02x) in ihex file.\n" % type)
        if segmentdata:
            self.segments.append( Segment(startAddr, "".join(segmentdata)) )

    def loadTIText(self, file):
        """load data from a (opened) file in TI-Text format"""
        next        = 1
        startAddr   = 0
        segmentdata = []
        #Convert data for MSP430, TXT-File is parsed line by line
        while next >= 1:
            #Read one line
            l = file.readline()
            if not l: break #EOF
            l = l.strip()
            if l[0] == 'q': break
            elif l[0] == '@':        #if @ => new address => send frame and set new addr.
                #create a new segment
                if segmentdata:
                    self.segments.append( Segment(startAddr, "".join(segmentdata)) )
                startAddr = int(l[1:],16)
                segmentdata = []
            else:
                for i in l.split():
                    segmentdata.append(chr(int(i,16)))
        if segmentdata:
            self.segments.append( Segment(startAddr, "".join(segmentdata)) )

    def loadELF(self, file):
        """load data from a (opened) file in ELF object format.
        File must be seekable"""
        import elf
        obj = elf.ELFObject()
        obj.fromFile(file)
        if obj.e_type != elf.ELFObject.ET_EXEC:
            raise Exception("No executable")
        for section in obj.getSections():
            debug(2, "ELF section %s at 0x%04x %d bytes",
                  section.name, section.lma, len(section.data))
            if len(section.data):
                self.segments.append( Segment(section.lma, section.data) )

    def loadFile(self, filename):
        """fill memory with the contents of a file. file type is determined from extension"""
        x = os.path.splitext(filename)[1].lower()
        if x == '.txt':
            self.loadTIText(open(filename, "rb"))
        elif x in ('.a43', '.hex', '.ihex'):
            self.loadIHex(open(filename, "rb"))
        else:
            self.loadELF(open(filename, "rb"))

    def getMemrange(self, fromadr, toadr):
        """get a range of bytes from the memory. unavailable values are filled with 0xff."""
        res = ''
        toadr = toadr + 1   #python indxes are excluding end, so include it
        while fromadr < toadr:
            #print "fromto: %04x %04x" % (fromadr, toadr)
            for seg in self.segments:
                #print seg
                segend = seg.startaddress + len(seg.data)
                if seg.startaddress <= fromadr and fromadr < segend:
                    #print "startok 0x%04x %d" % (seg.startaddress, len(seg.data))
                    #print ("0x%04x "*3) % (segend, fromadr, toadr)
                    if toadr > segend:   #not all data in segment
                        #print "out of segment"
                        catchlength = segend-fromadr
                    else:
                        catchlength = toadr-fromadr
                    #print toadr-fromadr
                    #print catchlength
                    res = res + seg.data[fromadr-seg.startaddress : fromadr-seg.startaddress+catchlength]
                    fromadr = fromadr + catchlength    #adjust start
                    if len(res) >= toadr-fromadr:
                        break#return res
            else:
                    res = res + chr(255)
                    fromadr = fromadr + 1 #adjust start
                    #print "fill FF"
        #print "res: %r" % res
        return res

class BootStrapLoader(BSL):
    """higher level Bootstrap Loader functions."""

    def __init__(self, *rest, **kw):
        BSL.__init__(self, *rest, **kw)
        self.byteCtr = 0
        self.data    = None
        self.speed   = 115200

    def programBlk(self, addr, blkout):
        debug(1, "Program starting at 0x%04x, %i bytes ..." % \
              (addr, len(blkout)))
        self.rxData(addr, blkout)

    # segments:
    # list of tuples or lists:
    # segments = [ (addr1, [d0,d1,d2,...]), (addr2, [e0,e1,e2,...])]
    def programData(self, segments):
        """program data"""
        bgn = time.time()
        for seg in segments:
            currentAddr = seg.startaddress
            pstart = 0
            while pstart < len(seg.data):
                length = self.maxData
                if pstart+length > len(seg.data):
                    length = len(seg.data) - pstart
                self.programBlk(currentAddr, seg.data[pstart:pstart+length])
                pstart = pstart + length
                currentAddr = currentAddr + length
                self.byteCtr = self.byteCtr + length # total sum
        self.progTime = time.time() - bgn

    def uploadData(self, startaddress, size, wait=0):
        """upload a datablock"""
        debug(2, "uploadData")
        data = ''
        pstart = 0
        while pstart<size:
            length = self.maxData
            if pstart+length > size:
                length = size - pstart
            data  += self.txData(pstart + startaddress, length)
            pstart = pstart + length
        return data

    #-----------------------------------------------------------------

    def actionMassErase(self, info=""):
        "erase program flash"
        for i in xrange(5):
            if i:
                debug(1, "Retrying...")
            try:
                self.massErase()
                if len(info) > 0:
                    self.txPassword(None)
                    self.eraseInfo(info)
            except BSLTimeout:
                time.sleep(0.1)
                #self.actionStartBSL()
                #time.sleep(0.1)
            else:
                return
        raise BSLTimeout

    def actionStartBSL(self):
        "fire up BSL"
        #self.i2c_write_data(chr(i2caddr(0x00)) + chr(0x52) + chr(0x55) + chr(0x4e) + chr(0x20) + chr(0x42) + chr(0x53) + chr(0x4c))
        self.inBSL = True

    def actionChangeBaudrate(self, baudrate=9600):
        #self.setBaud(baudrate)
        return

    def actionTxPassword(self):
        "send BSL password"
        self.txPassword(None)

    def actionProgram(self):
        """program data into flash memory."""
        if self.data is not None:
            debug(0, "Program ...")
            self.programData(self.data)
            debug(0, "%i bytes programmed in %.1fsec, %dbps" % \
                  (self.byteCtr,
                   self.progTime,
                   self.byteCtr * 8.0 / (self.progTime or 1e-6)))
        else:
            raise BSLException, "programming without data not possible"

    def actionReset(self):
        "reset MCU"
        debug(0, "Reset device")
        self.bslReset(False)

    def actionRun(self, address):
        "jump to address"
        self.loadPC(address)

    def actionReadBSLVersion(self):
        self.txBslVersion()

def usage():
    """print some help message"""
    sys.stderr.write("""
USAGE: %s [options] [file]
Version: %s

If "-" is specified as file the data is read from the stdinput.
A file ending with ".txt" is considered to be in TIText format,
'.a43' and '.hex' as IntelHex and all other filenames are
considered as ELF files.

General options:
  -h, --help            Show this help screen.
  -c, --comport=port    Specify the communication port to be used.
                        (Default is 0)
                                0->COM1 / ttyS0
                                1->COM2 / ttyS1
                                etc.
  -P, --password=file   Specify a file with the interrupt vectors that
                        are used as password. This can be any file that
                        has previously been used to program the device.
                        (e.g. -P INT_VECT.TXT).
  -f, --framesize=num   Max. number of data bytes within one transmitted
                        frame (16 to 240 in steps of 16) (e.g. -f 240).
  -D, --debug           Increase level of debug messages. This won't be
                        very useful for the average user...
  -I, --intelhex        Force fileformat to IntelHex
  -T, --titext          Force fileformat to be TIText
  -N, --notimeout       Don't use timeout on serial port (use with care)
  -S, --speed=baud      Reconfigure speed, only possible with newer
                        MSP403-BSL versions (>1.5, read slaa089a.pdf for
                        details). If the --bsl option is not used, an
                        internal BSL replacement will be loaded.
                        Needs a target with at least 2kB RAM!
                        Possible values are 9600, 19200, 38400, 57600, 115200
                        (default 9600)
  --surf                Assume SuRF hardware, currently same as --max-baud
  --fastrx              Program without verify (use with caution)
  --max-baud            Use the fastest supported baud rate (115200)
  --wipe-info=(abcd)    Wipe the specified info sections, abcd

Program Flow Specifiers:
  -e, --masserase       Mass Erase (clear all flash memory)
  -p, --program         Program file

Data retreiving:
  -u, --upload=addr     Upload a datablock (see also: -s).
  -s, --size=num        Size of the data block do upload. (Default is 2)
  -x, --hex             Show a hexadecimal display of the uploaded data.
                        (Default)
  -b, --bin             Get binary uploaded data. This can be used
                        to redirect the output into a file.

Do before exit:
  -g, --go=address      Start programm execution at specified address.
                        This implies option --wait.
  -r, --reset           Reset connected MSP430. Starts application.
                        This is a normal device reset and will start
                        the programm that is specified in the reset
                        vector. (see also -g)
  -w, --wait            Wait for <ENTER> before closing serial port.

Protocol:
  -a, --address         Programming Address for I2C communication
""" % (sys.argv[0], VERSION))

def main():
    global DEBUG
    import getopt
    comPort     = 0     # Default setting.
    wait        = False
    reset       = False
    goaddr      = None
    startaddr   = None
    size        = 2
    hexoutput   = True
    filetype    = None
    notimeout   = False
    massErase   = False
    infoErase   = ""
    todo        = []
    bsl         = BootStrapLoader()
    # bsl.address = ADDRESS
    # bsl.passwd  = ([0xFF] * 14) + [0x00, 0x44] + ([0xFF] * 14) + [0x54, 0x44, 0x8D, 0x4F]
    # bsl.maxData
    # bsl.fastrx
    # bsl.speed
    # bsl.inBSL = True

    filename    = None

    sys.stderr.write("MSP430 Bootstrap Loader Version: %s\n" % VERSION)

    try:
        opts, args = getopt.getopt(sys.argv[1:],
                "hc:P:f:DITNS:epu:s:xbg:rwa:",
            ["help", "comport=", "password=", "framesize=", "debug",
             "intelhex", "titext", "notimeout", "speed=",
             "surf", "fastrx", "max-baud", "masserase", "program",
             "bslversion", "wipe-info=", "upload=", "size=", "hex", "bin",
             "go=","reset", "wait", "address="]
        )
    except getopt.GetoptError:
        # print help information and exit:
        usage()
        sys.exit(2)

    for o, a in opts:
        if o in ("-h", "--help"):
            usage()
            sys.exit()
        elif o in ("-c", "--comport"):
            try:
                comPort = int(a)                    #try to convert decimal
            except ValueError:
                comPort = a                         #take the string and let serial driver decide
        elif o in ("-P", "--password"):
            #extract password from file
            bsl.passwd = Memory(a).getMemrange(0xffe0, 0xffff)
        elif o in ("-f", "--framesize"):
            try:
                maxData = int(a)                    #try to convert decimal
            except ValueError:
                sys.stderr.write("framesize must be a valid number\n")
                sys.exit(2)
            #Make sure that conditions for maxData are met:
            #( >= 16 and == n*16 and <= MAX_DATA_BYTES!)
            if maxData > BootStrapLoader.MAX_DATA_BYTES:
                maxData = BootStrapLoader.MAX_DATA_BYTES
            elif maxData < 16:
                maxData = 16
            bsl.maxData = maxData & 0xfffff0
            sys.stderr.write( "Max. number of data bytes within one frame set to %i.\n" % maxData)
        elif o in ("-D", "--debug"):
            DEBUG = DEBUG + 1
        elif o in ("-I", "--intelhex"):
            filetype = 0
        elif o in ("-T", "--titext"):
            filetype = 1
        elif o in ("-N", "--notimeout"):
            notimeout = True
        elif o in ("-S", "--speed"):
            try:
                bsl.speed = int(a, 0)
            except ValueError:
                sys.stderr.write("speed must be decimal number\n")
                sys.exit(2)
        elif o in ("--surf", ):
            bsl.speed = 0
        elif o in ("--fastrx", ):
            bsl.fastrx = True
        elif o in ("--max-baud", ):
            pass
        elif o in ("-e", "--masserase"):
            massErase = True
        elif o in ("--wipe-info", ):
            sys.stderr.write("Erasing info sections (%s)\n" % a);
            infoErase = a
        elif o in ("-p", "--program"):
            todo.append(bsl.actionProgram)
        elif o in ("-u", "--upload"):
            try:
                startaddr = int(a, 0)
            except ValueError:
                sys.stderr.write("upload address must be a valid number\n")
                sys.exit(2)
        elif o in ("-s", "--size"):
            try:
                size = int(a, 0)
            except ValueError:
                sys.stderr.write("size must be a valid number\n")
                sys.exit(2)
        elif o in ("-x", "--hex"):
            hexoutput = True
        elif o in ("-b", "--bin"):
            hexoutput = False
        elif o in ("-g", "--go"):
            try:
                goaddr = int(a, 0)
            except ValueError:
                sys.stderr.write("go address must be a valid number\n")
                sys.exit(2)
            wait = True
        elif o in ("-r", "--reset"):
            reset = True
        elif o in ("-w", "--wait"):
            wait = True
        elif o in ("-a", "--address"):
            try:
                bsl.address = int(a, 0)
            except ValueError:
                sys.stderr.write(a + " is not a valid i2c address\n")
                sys.exit(2)
        else:
            raise RuntimeError, "bugs in the taters"

    if len(args) == 0:
        sys.stderr.write("Use -h for help\n")
    elif len(args) == 1:
        if not todo:                                # if there are no actions
            todo.extend([                           # add some useful actions...
                bsl.actionProgram,
            ])
        filename = args[0]
    else:                                           #number of args is wrong
        usage()
        sys.exit(2)

    debug(1, "Debug level set to %d", DEBUG)
    debug(1, "Python version: %s", sys.version)

    #sanity check of options
    if notimeout and goaddr is not None and startaddr is not None:
        raise SystemExit, "Option --notimeout can not be used together with both --upload and --go"

    if notimeout:
        print >>sys.stderr, "Warning: option --notimeout can cause improper function in some cases!"
        bsl.timeout = 0

    if goaddr and reset:
        print >>sys.stderr, "Warning: --reset ignored as --go is specified"
        reset = False

    if startaddr and goaddr:
        print >>sys.stderr, "Warning: --go ignored as --upload is specified"
        goaddr = None

    if startaddr and reset:
        print >>sys.stderr, "Warning: --reset ignored as --upload is specified"
        reset = False

    if startaddr and wait:
        print >>sys.stderr, "Warning: --wait ignored as --upload specified"
        wait = False

    # prepare data to download
    bsl.data = Memory()                             # prepare downloaded data
    if filetype is not None:                        # if filetype is given...
        if filename is None:
            raise ValueError, "no filename but filetype specified"
        if filename == '-':                         # get data from stdin
            file = sys.stdin
        else:
            file = open(filename, "rb")             # or from a file
        if filetype == 0:                           # select load function
            bsl.data.loadIHex(file)                 # intel hex
        elif filetype == 1:
            bsl.data.loadTIText(file)               # TI's format
        else:
            raise ValueError, "illegal filetype specified"
    else:                                           # no filetype given...
        if filename == '-':                         # for stdin:
            bsl.data.loadIHex(sys.stdin)            # assume intel hex
        elif filename:
            bsl.data.loadFile(filename)             # autodetect otherwise

    debug(3, "File: %s", filename)

    bsl.comInit(comPort)                            # init port

    # get BSL running
    if todo or massErase or goaddr or startaddr:
        bsl.actionStartBSL()

    # initialization list
    if massErase:
        debug(1, "Preparing device ...")
        bsl.actionMassErase(infoErase)

    # send password
    if todo or goaddr or startaddr or massErase:
        bsl.actionTxPassword()
        bsl.actionReadBSLVersion()

    # work list
    if todo:
        # show a nice list of sheduled actions
        debug(2, "TODO list:")
        for f in todo:
            try:
                debug(2, "   %s", f.func_name)
            except AttributeError:
                debug(2, "   %s", f)
        for f in todo: f()                          # work through todo list

    if reset:
        bsl.actionReset()

    if goaddr is not None:
        bsl.actionRun(goaddr)                       # load PC and execute

    # upload datablock and output
    if startaddr is not None:
        data = bsl.uploadData(startaddr, size)      # upload data
        if hexoutput:                               # depending on output format
            while data:
                print "%06x %s" % (startaddr, hexl(data[:16]))
                startaddr += 16
                data = data[16:]
        else:
            sys.stdout.write(data)                  # binary output w/o newline!

    if wait:                                        # wait at the end if desired
        sys.stderr.write("Press <ENTER> ...\n")     # display a prompt
        sys.stderr.flush()
        raw_input()                                 # wait for newline

    bsl.close()           #Release serial communication port

if __name__ == '__main__':
    i = 0
    while True:
        try:
            main()
            exit(0)
        except SystemExit:
            raise               #let pass exit() calls
        except KeyboardInterrupt:
            if DEBUG:
                raise     #show full trace in debug mode
            raise SystemExit, "user abort."
        except Exception, msg:  #every Exception is caught and displayed
            if DEBUG:
                raise
            i += 1
            if i > MAXIMUM_ATTEMPTS:
                raise SystemExit, ("\nAn error occoured:\n%s\n" % msg)

### EOF msp430-cbsl

