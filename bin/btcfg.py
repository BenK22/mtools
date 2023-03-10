#!/usr/bin/python -u
__version__ = '0.2.3'
'''Configuration tool for Brultech power monitoring devices.

Copyright 2012 Matthew Wall, all rights reserved

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See http://www.gnu.org/licenses/


Read configuration values from ECM-1220, ECM-1240, or Green-Eye Monitor.

Based on the following documents:
 'GEM Commands V1.pdf' as of 1 October 2012
 'ECM1240 Packet format ver9.pdf' as of 8 December 2011
 'ECM1240 Commands ver3.pdf' as of 15 April 2009

Changelog:

- 0.2.3 07feb15 mwall
* fix gain state for RCV data (thanks to eric sandeen)
* added rb command to reboot device (thanks to eric sandeen)
* fix RCV command (thanks to eric sandeen)
* added x1 and x2 commands to set gain on AUX channels (thanks to eric sandeen)
* better way to find ecm1240 devices

- 0.2.2 30jul13 mwall
* copyright notices
* fix rqsrks command on ecm-1240 (thanks to Greg Schaefer)
* added quit (thanks to eric sandeen)
* send numeric values as unsigned char (thanks to eric sandeen)

- 0.2.0 21oct12 mwall
* support ecm 1220
* support ecm 1240
* added packet objects with basic packet printing

- 0.1.1 09oct12 mwall
* include support for sys commands

- 0.1.0 01oct12 mwall
* initial implementation - ip only but with scaffolding for serial also
* includes full set of rqs and api commands
* includes partial support for ch and che commands
'''
__author__ = 'mwall'

# TODO: testing with serial port

# devices
DEV_ECM1220 = 'ecm1220'
DEV_ECM1240 = 'ecm1240'
DEV_GEM = 'gem'
DEVICE_TYPES = [DEV_ECM1220, DEV_ECM1240, DEV_GEM]
DEFAULT_DEVICE_TYPE = DEV_GEM

# serial settings
# the com/serial port to which device is connected (COM4, /dev/ttyS01, etc)
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 19200
SERIAL_TIMEOUT = 1
SERIAL_BUFFER_SIZE = 2048

# ethernet settings
IP_PORT = 5000
IP_TIMEOUT = 1
IP_BUFFER_SIZE = 2048

import optparse
import socket
import sys
import time
import traceback
import struct

try:
    import serial
except Exception, e:
    serial = None

try:
    import ConfigParser
except Exception, e:
    ConfigParser = None

# logging and error reporting
#
# note that setting the log level to debug will affect the application
# behavior, especially when sampling the serial line, as it changes the
# timing of read operations.
LOG_ERROR = 0
LOG_WARN  = 1
LOG_INFO  = 2
LOG_DEBUG = 3
LOGLEVEL  = 2

def dbgmsg(msg):
    if LOGLEVEL >= LOG_DEBUG:
        logmsg(msg)

def infmsg(msg):
    if LOGLEVEL >= LOG_INFO:
        logmsg(msg)

def wrnmsg(msg):
    if LOGLEVEL >= LOG_WARN:
        logmsg(msg)

def errmsg(msg):
    if LOGLEVEL >= LOG_ERROR:
        logmsg(msg)

def logmsg(msg):
    ts = fmttime(time.localtime())
    print "%s %s" % (ts, msg)

def fmttime(seconds):
    return time.strftime("%Y/%m/%d %H:%M:%S", seconds)

# Helper Functions

def getgmtime():
    return int(time.time())

def getbit(n, bytes):
    idx = int(n / 8)
    mask = n % 8
    if bytes[idx] & (1 << mask):
        return 1
    return 0


class BasePacket(object):
    def __init__(self):
        self.START_HEADER0     = 254
        self.START_HEADER1     = 255
        self.END_HEADER0       = 255
        self.END_HEADER1       = 254
        self.PACKET_LENGTH     = 0 # must be defined by derived class
        self.PACKET_ID         = 0 # must be defined by derived class

    def _convert(self, bytes):
        return reduce(lambda x,y:x+y[0] * (256**y[1]), zip(bytes,xrange(len(bytes))),0)

    def _calculate_checksum(self, packet, id):
        '''calculate the packet checksum'''
        checksum = self.START_HEADER0
        checksum += self.START_HEADER1
        checksum += id
        checksum += sum(packet)
        checksum += self.END_HEADER0
        checksum += self.END_HEADER1
        return checksum & 0xff

    def validatebytes(self, bytes):
        if not len(bytes) == self.PACKET_LENGTH:
            raise Exception('expected packet length %d, got %d' %
                            (self.PACKET_LENGTH, len(bytes)))
        byte = bytes[0]
        if not byte == self.START_HEADER0:
            raise Exception("expected START_HEADER0 %s, got %s" %
                            (hex(self.START_HEADER0), hex(byte)))
        byte = bytes[1]
        if not byte == self.START_HEADER1:
            raise Exception("expected START_HEADER1 %s, got %s" %
                            (hex(self.START_HEADER1), hex(byte)))
        byte = bytes[2]
        if not byte == self.PACKET_ID:
            raise Exception("expected PACKET_ID %s, got %s" %
                            (hex(self.PACKET_ID), hex(byte)))
        byte = bytes[self.PACKET_LENGTH-3]
        if not byte == self.END_HEADER0:
            raise Exception("expected END_HEADER0 %s, got %s" %
                            (hex(self.END_HEADER0), hex(byte)))
        byte = bytes[self.PACKET_LENGTH-2]
        if not byte == self.END_HEADER1:
            raise Exception("expected END_HEADER1 %s, got %s" %
                            (hex(self.END_HEADER1), hex(byte)))
        byte = bytes[self.PACKET_LENGTH-1]
        checksum = self._calculate_checksum(bytes[3:self.PACKET_LENGTH-2], self.PACKET_ID)
        # FIXME: figure out source of this off-by-one error
        if byte != checksum + 1:
            raise Exception("bad checksum: expected %s, got %s" %
                            (hex(checksum), hex(byte)))


class ECM1220BinaryPacket(BasePacket):
    def __init__(self):
        super(ECM1220BinaryPacket, self).__init__()
        self.PACKET_ID = 1
        self.PACKET_LENGTH = 43
        
    def printpacket(self, bytes):
        print 'packet type = %d' % self._convert(bytes[2:3])
        x = 0.1 * self._convert(bytes[4:2:-1])
        print 'volts = %.2f' % x
        print 'ch1 aws = %d' % self._convert(bytes[5:10])
        print 'ch2 aws = %d' % self._convert(bytes[10:15])
        print 'ch1 pws = %d' % self._convert(bytes[15:20])
        print 'ch2 pws = %d' % self._convert(bytes[20:25])
        print 'serial = %d' % self._convert(bytes[29:31])
        print 'flag = %d' % self._convert(bytes[31:32])
        print 'unit id = %d' % self._convert(bytes[32:33])
        x = 0.01 * self._convert(bytes[33:35])
        print 'ch1 amps = %.2f' % x
        x = 0.01 * self._convert(bytes[35:37])
        print 'ch2 amps = %.2f' % x
        print 'seconds = %d' % self._convert(bytes[37:40])


class ECM1240BinaryPacket(ECM1220BinaryPacket):
    def __init__(self):
        super(ECM1240BinaryPacket, self).__init__()
        self.PACKET_ID = 3
        self.PACKET_LENGTH = 65

    def printpacket(self, bytes):
        super(ECM1240BinaryPacket, self).printpacket(bytes)
        print 'aux1 ws = %d' % self._convert(bytes[40:44])
        print 'aux2 ws = %d' % self._convert(bytes[44:48])
        print 'aux3 ws = %d' % self._convert(bytes[48:52])
        print 'aux4 ws = %d' % self._convert(bytes[52:56])
        print 'aux5 ws = %d' % self._convert(bytes[56:60])
        print 'aux5 volts = %d' % self._convert(bytes[60:62])


class GEM48PBinaryPacket(BasePacket):
    def __init__(self):
        super(GEM48PBinaryPacket, self).__init__()
        self.PACKET_LENGTH = 50


def packetfactory(bytes):
    if bytes[2] == 1:
        return ECM1220Packet()
    elif bytes[2] == 3:
        return ECM1240Packet()
    elif bytes[2] == 5:
        return GEM48PBinaryPacket()
    return None


class BaseDevice(object):
    def __init__(self):
        self._id = None

    def setup(self):
        pass

    def cleanup(self):
        pass

    def getprompt(self):
        return ''


class ECMDevice(BaseDevice):
    def __init__(self):
        super(ECMDevice, self).__init__()
        self.CFM_BYTE = chr(0xfc)
        self._id = chr(0xfc)

    def getprompt(self):
        return hex(ord(self._id))

    def printsettings(self, resp):
        bytes = [ord(c) for c in resp]
        chksum = 0
        for idx,b in enumerate(bytes):
            dbgmsg('%d: %d' % (idx, b))
            if (idx < 32):
                chksum += b
        chksum = chksum & 0xff
        print '# settings for %s at %s' % \
            (hex(ord(self._id)), fmttime(time.localtime()))
        print 'ct1 type = %d' % bytes[0]
        print 'ct1 range = %d' % bytes[1]
        print 'ct2 type = %d' % bytes[2]
        print 'ct2 range = %d' % bytes[3]
        print 'pt type = %d' % bytes[4]
        print 'pt range = %d' % bytes[5]
        print 'send frequency = %d' % bytes[6]
        print 'storage interval = %d' % bytes[7]
        v = 0.001 * (256* bytes[8] + bytes[9])
        print 'firmware version = %1.3f' % v
        print 'device id = %d' % bytes[10]
        v = 256 * bytes[11] + bytes[12]
        print 'serial number = %d' % v
        print 'gain X2: ',
        for ct in xrange(1,6):
            b = getbit(ct-1, bytes[14:15])
            print 'aux%d[%s] ' % (ct, 'X' if b else ' '),
        print ''
        v = bytes[29] + 256 * bytes[30]
        print 'trigger = %d' % v
        print 'checksum = %d (%s)' % (bytes[32],
                                      'ok' if chksum == bytes[32] else 'fail')

    def _finddevice(self, cmd):
        devices = [0xfc, 0xfd, 0xfe, 0xff]
        for dev in devices:
            dbgmsg('Trying device %s' % hex(dev))
            com.send(chr(dev))
            resp = com.recv(1)
            dbgmsg('got response %s' % hex(ord(resp)))
            if resp == self.CFM_BYTE:
                print 'Found device at %s' % hex(dev)
                break
            time.sleep(2)
        if resp != self.CFM_BYTE:
            print "Could not find device"

    def _setdevice(self, cmd):
        a = cmd.split(' ')
        if a[1] == 'fc' or a[1] == 'fd' or a[1] == 'fe' or a[1] == 'ff':
            self._id = chr(int(a[1],16))
            print 'id set to %s' % hex(ord(self._id))
        else:
            print "bogus identifier '%s', must be one of fc, fd, fe, or ff" % a[1]

    def _confirm(self, com):
        resp = com.recv(1)
        if not resp == self.CFM_BYTE:
            raise Exception('wrong response %s, expected %s' %
                            (hex(ord(resp)), hex(ord(self.CFM_BYTE))))

    def _sendrcv(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('RCV')
        self._confirm(com)
        resp = com.recv(33)
        com.send(id) # doc says to do this
        self._confirm(com)
        return resp

    def _sendspk(self, com, id, pktsz):
        com.send(id)
        self._confirm(com)
        com.send('SPK')
        self._confirm(com)
        packet = com.recv(pktsz)
        return packet


class ECM1220Device(ECMDevice):
    def __init__(self):
        super(ECM1220Device, self).__init__()
        self.PACKET_SIZE = 43

    def isvalidcmd(self, cmd):
        cmd = cmd.lower()
        return cmd == 'rcv' or \
            cmd == 'spk' or \
            cmd.startswith('d ')

    def sendcmd(self, cmd, com):
        try:
            com.open()
            if cmd == 'rcv':
                settings = self._sendrcv(com, self._id)
                self.printsettings(settings)
            elif cmd == 'spk':
                packet = self._sendspk(com, self._id, self.PACKET_SIZE)
                self.printpacket(packet)
            elif cmd.startswith('d '):
                self._setdevice(cmd)
        finally:
            com.close()

    def printpacket(self, resp):
        bytes = [ord(c) for c in resp]
        packet = ECM1220BinaryPacket()
        packet.validatebytes(bytes)
        packet.printpacket(bytes)

    def printhelp(self):
        print 'rcv - display settings'
        print 'spk - send a single packet'
        print 'd XX - specify device identifier one of [fc,fd,fe,ff]'
        print 'q - exit btcfg'


class ECM1240Device(ECMDevice):
    def __init__(self):
        super(ECM1240Device, self).__init__()
        self.PACKET_SIZE = 65

    def isvalidcmd(self, cmd):
        cmd = cmd.lower()
        return (cmd == 'rcv' or
                cmd.startswith('ct ') or
                cmd.startswith('pt ') or
                cmd.startswith('sf ') or
                cmd.startswith('tv ') or
                cmd.startswith('rqs') or
                cmd.startswith('x1') or
                cmd.startswith('x2') or
                cmd == 'rkw' or
                cmd == 'rks' or
                cmd.startswith('ng ') or
                cmd == 'xtd' or
                cmd == 'off' or
                cmd == 'spk' or
                cmd == 'rb' or
                cmd == 'id' or
                cmd.startswith('d '))

    def sendcmd(self, cmd, com):
        try:
            com.open()
            if cmd == 'rcv':
                settings = self._sendrcv(com, self._id)
                self.printsettings(settings)
            elif cmd.startswith('ct'):
                args = cmd.split(' ')
                self._sendct(com, self._id, int(args[1]), int(args[2]), int(args[3]))
            elif cmd.startswith('pt'):
                args = cmd.split(' ')
                self._sendpt(com, self._id, int(args[1]), int(args[2]))
            elif cmd.startswith('sf'):
                args = cmd.split(' ')
                self._sendsf(com, self._id, int(args[1]))
            elif cmd.startswith('tv'):
                args = cmd.split(' ')
                self._sendtv(com, self._id, int(args[1]))
            elif cmd == 'rks':
                self._sendrks(com, self._id)
            elif cmd == 'rkw':
                self._sendrkw(com, self._id)
            elif cmd == 'rb':
                self.sendrlp(com, self._id)
            elif cmd.startswith('ng'):
                args = cmd.split(' ')
                self._sendng(com, self._id, int(args[1]))
            elif cmd == 'xtd':
                self._sendxtd(com, self._id)
            elif cmd == 'off':
                self._sendoff(com, self._id)
            elif cmd == 'spk':
                packet = self._sendspk(com, self._id, self.PACKET_SIZE)
                self.printpacket(packet)
            elif cmd.startswith('x1') or cmd.startswith('x2'):
                args = cmd.split(' ')
                self._sendopt(com, self._id, args)
            elif cmd == 'id':
                self._finddevice(cmd)
            elif cmd.startswith('d '):
                self._setdevice(cmd)
        finally:
            com.close()

    def _sendct(self, com, id, ch, t, r):
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('CT%d' % ch)
        self._confirm(com)
        com.send('TYP')
        self._confirm(com)
        com.sendbyte(t)
        self._confirm(com)
        com.send('RNG')
        self._confirm(com)
        com.sendbyte(r)
        self._confirm(com)

    def _sendpt(self, com, id, t, r):
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('PTT')
        self._confirm(com)
        com.sendbyte(t)
        self._confirm(com)
        com.send('PTR')
        self._confirm(com)
        com.sendbyte(r)
        self._confirm(com)

    def _sendsf(self, com, id, v):
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('IV2')
        self._confirm(com)
        com.sendbyte(v)
        self._confirm(com)

    def _sendtv(self, com, id, v):
        hi = v / 256
        lo = v % 256
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('IV3')
        self._confirm(com)
        com.sendbyte(lo)
        self._confirm(com)
        com.sendbyte(hi)
        self._confirm(com)

    def _sendrks(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('RQS')
        self._confirm(com)
        com.send('RKS')
        self._confirm(com)

    def _sendrkw(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('RQS')
        self._confirm(com)
        com.send('RKW')
        self._confirm(com)

    def _sendrlp(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('RQS')
        self._confirm(com)
        com.send('RLP')
        self._confirm(com)

    def _sendng(self, com, id, ch):
        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('NG%d' % ch)
        self._confirm(com)

    def _sendxtd(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('TOG')
        self._confirm(com)
        com.send('XTD')

    def _sendoff(self, com, id):
        com.send(id)
        self._confirm(com)
        com.send('TOG')
        self._confirm(com)
        com.send('OFF')

    def _sendopt(self, com, id, args):
        # save and remove the first argument (x1/x2)
        gain = args[0]
        del args[0]
        # Get current gain bitmask in the 14th byte of the settings
        resp = self._sendrcv(com, id)
        bytes = [ord(c) for c in resp]
        # args is now a list of CTs to act on; settings[14] is the bitmap
        for bit in args:
            mask = 1 << (int(bit[0])-1)
            if (gain == 'x2'):
                bytes[14] |= mask
            else: # (gain == 'x1')
                bytes[14] &= ~mask

        print 'Setting Gain X2: ',
        for ct in xrange(1,6):
          b = getbit(ct-1, bytes[14:15])
          print 'aux%d[%s] ' % (ct, 'X' if b else ' '),
        print ''

        com.send(id)
        self._confirm(com)
        com.send('SET')
        self._confirm(com)
        com.send('OPT')
        self._confirm(com)
        com.sendbyte(bytes[14])
        self._confirm(com)

    def printpacket(self, resp):
        bytes = [ord(c) for c in resp]
        packet = ECM1240BinaryPacket()
        packet.validatebytes(bytes)
        packet.printpacket(bytes)

    def printhelp(self):
        print 'rcv - display settings'
        print 'ct N TTT RRR - set type and range of CT N to TTT and RRR'
        print 'pt TTT RRR - set the PT type and range to TTT and RRR'
        print 'sf X - set the packet send frequency to X seconds [1,255]'
        print 'tv X - set the packet send trigger value to X watts [1,32000]'
        print 'rkw - reset the watt-second counters'
        print 'rks - reset the seconds counter'
        print 'ng N - toggle polarity on channel N'
        print 'xtd - set real-time mode to on'
        print 'off - set real-time mode to off'
        print 'spk - send a single packet'
        print 'x1 a b c ... - set X1 gain on AUX channels a b c ...'
        print 'x2 a b c ... - set X2 gain on AUX channels a b c ...'
        print 'id XX - discover device identifier'
        print 'd XX - specify device identifier one of [fc,fd,fe,ff]'
        print 'rb - reboot device'
        print 'q - exit btcfg'


class GEMDevice(BaseDevice):
    def __init__(self):
        super(GEMDevice, self).__init__()

    def getprompt(self):
        return ''

    def isvalidcmd(self, cmd):
        cmd = cmd.lower()
        return cmd.startswith('rqs') or \
            cmd.startswith('api') or \
            cmd.startswith('ch') or \
            cmd.startswith('che') or \
            cmd.startswith('sys') or \
            cmd.startswith('tmp') or \
            cmd.startswith('pls')

    def sendcmd(self, cmd, com):
        if cmd.lower() == 'sysbr0' or cmd.lower() == 'sysbr1':
            return 'escape from bridge mode not implemented'

        resp = ''
        try:
            com.open()
            com.send('^^^' + cmd.upper())
            resp = com.recv()
        finally:
            com.close()

        cmd = cmd.lower()
        dbgmsg(resp)
        if cmd == 'rqsall':
            idx = resp.find('ALL')
            if idx < 0:
                print 'unrecognized response: no ALL tag'
                return
            self.printsettings(resp)
        elif cmd == 'rqsatp':
            idx = resp.find('ATP')
            if idx < 0:
                print 'unrecognized response: no ATP tag'
                return
            bytes = self._str2bytes(resp[idx+5:])
            print bytes
        elif cmd == 'rqsarg':
            idx = resp.find('ARG')
            if idx < 0:
                print 'unrecognized response: no ARG tag'
                return
            bytes = self._str2bytes(resp[idx+5:])
            print bytes
        elif cmd == 'rqsaop':
            idx = resp.find('AOP')
            if idx < 0:
                print 'unrecognized response: no AOP tag'
                return
            # bit 0,1 - phase
            #  00 phase A
            #  01 phase B
            #  10 phase C
            #  11 invalid - no phase
            # bits 2,3 - phase compensation
            # bit 4 - double load for 240V balanced loads
            # bit 7 - polarity toggle
            bytes = self._str2bytes(resp[idx+5:])
            print bytes
        elif cmd == 'rqsrpt':
            bytes = self._str2bytes(resp)
            print 'PT: type:%d range:%d' % (bytes[0], bytes[1])
        elif cmd == 'rqstst':
            if not len(resp) == 10:
                print 'unrecognized response length %d' % len(resp)
                return
            for x in range(1,9):
                print 't%d: %s' % (x, resp[x-1])
        elif cmd == 'rqspst':
            for x in range(1,5):
                print 'p%d: %s' % (x, resp[4-x])
        elif cmd == 'apival':
            sidx = resp.find('VAL')
            if sidx < 0:
                print 'unrecognized response: no VAL tag'
                return
            eidx = resp.find('END')
            if eidx < 0:
                print 'unrecognized response: no END tag'
                return
            vals = self._str2floats(resp[sidx+3:eidx])
            for x in range(1,49):
                print 'ch%2d: % 5.3f' % (x, vals[x-1])
            for x in range(1,9):
                print '  t%d: % 5.3f' % (x, vals[47+x])
            for x in range(1,5):
                print '  p%d: % 5.3f' % (x, vals[55+x])
        elif cmd.startswith('sysdtm'):
            sidx = resp.find('DTM')
            if sidx < 0:
                print 'date unchanged'
            else:
                print resp
        else:
            print resp

    def _str2bytes(self, str):
        bytes = str.split(',')
        return [int(c,16) for c in bytes]

    def _str2floats(self, str):
        bytes = str.split(',')
        return [float(c) for c in bytes]

    def printsettings(self, resp):
        print '# settings at %s' % fmttime(time.localtime())
        idx = resp.find('ALL')
        bytes = self._str2bytes(resp[idx+5:])
        for idx,b in enumerate(bytes):
            dbgmsg('%d: %d' % (idx, b))
        # 0: spare
        print '#      type chopt pktopt send'
        for x in range(1,49):
            s = getbit(x-1, bytes[306:312])
            print 'ch%02d = %3d  %3d   %3d    %d' % (x, bytes[48+x], bytes[x], bytes[130+x], s)
        print '#    on inc send code'
        for x in range(1,9):
            e = getbit(x-1, bytes[246:247])
            i = getbit(x-1, bytes[181:182])
            s = getbit(x-1, bytes[313:314])
            c = bytes[182+8*(x-1):182+8*x]
            print 't%d = %d  %d   %d    %s' % (x, e, i, s, c)
        print '#    on send'
        for x in range(1,5):
            s = getbit(x-1, bytes[312:313])
            o = getbit(x-1, bytes[319:320])
            print 'p%d = %d  %d' % (x, o, s)
        print 'PT type = %d' % bytes[121]
        print 'PT range = %d' % bytes[122]
        print 'packet format = %d' % bytes[123]
        print 'packet send interval = %d' % bytes[124]
        print 'wifi auto reset timer = %d' % bytes[125]
        print 'wifi missing response reset value = %d' % bytes[126]
        print 'wifi options = %d' % bytes[126]
        print 'xbee auto reset timer = %d' % bytes[127]
        print 'xbee missing response reset value = %d' % bytes[128]
        print 'xbee options = %d' % bytes[129]
        # 130: primary com port
        # 131-177: packet options for each channel
        print 'counter options = %d %d %d %d' % (getbit(0,bytes[178:179]), getbit(1,bytes[178:179]), getbit(2,bytes[178:179]), getbit(3,bytes[178:179]))
        # 179: one-wire enable
        # 180-243: one-wire ROM code
        # 244: one-wire enabled
        print 'one-wire device options = %d' % bytes[245]
        print 'hardware modules installed = %d' % bytes[246]
        print 'maximum channels processed = %d' % bytes[247]
        # 248-295: reserved
        print 'system settings = %s' % bytes[298:306]
        # 306-311: selects channels for http put
        # 312: selects pulse for http put
        # 313: selects one-wire for http put
        print 'general packet options = %s' % bytes[314:316]
        print 'net metering options = %d' % bytes[316]
        n = bytes[317]*256+bytes[318]
        print 'chunk size = %d' % n
        # 319: which pulse counters are enabled

    def printhelp(self):
        print 'rqs'
        print '  all - all settings'
        print '  atp - all CT types'
        print '  arg - all CT ranges'
        print '  aop - all channel options'
        print '  phXX - channel phase settings'
        print '  rtl - realtime status'
        print '  itv - realtime send interval'
        print '  pst - pulse on/off status'
        print '  pks - packet chunk size'
        print '  pki - packet chunk interval'
        print '  bff - max buffer size'
        print '  cXX - channel CT type and range'
        print '  rpt - PT type and range'
        print '  hz - frequency setting'
        print '  rtrX - ROM byte setting for one-wire channel X'
        print '  cmx - max channel setting'
        print '  enmX - master enable polarized watt-second counters'
        print '  tsb - thingspeak number of blocks'
        print '  url - URL'
        print '  ure - URL extension'
        print '  teX - output from one-wire channel X'
        print '  war - wifi auto reset value'
        print '  wrr - wifi missing response counter value'
        print '  vr1 - communication firmware version'
        print '  vr2 - processor firmware version ch01-ch32'
        print '  vr3 - processor firmware version ch33-ch48'
        print '  dtm - realtime clock'
        print '  srn - serial number'
        print '  epc - enabled channels'
        print '  epp - enabled pulse counters'
        print '  ept - enabled one-wire channels'
        print 'api'
        print '  watXX - power on channel XX'
        print '  srgSSRR - power for RR channels starting at SS'
        print '  enrXX - watt-seconds on channel XX'
        print '  egrSSRR - watt-seconds for RR channels starting at SS'
        print '  epc - channels that are included for post'
        print '  epp - pulse channels that are included for post'
        print '  ept - one-wire channels that are included for post'
        print '  vlt - voltage'
        print '  sec - seconds counter'
        print '  val - recent values from every counter'
        print '  tpX - value from one-wire channel X'
        print '  spk - send a single packet'
        print 'ch'
        print '  XXtypeYYY - set type on channel XX in [01-48] to YYY in [001-255]'
        print '  XXrngY - set range on channel XX in [01-48] to Y in [0-9]'
        print '  XXphY - set phase on channel XX in [01-48] to Y in (A,B,C)'
        print '  XXpcY - set phase compensation on channel XX in [01-48] to Y in [0-2]'
        print '  PXX - set polarity on channel XX in [01-48] to P in [0-1]'
        print 'che'
        print '  typYYY - set type on all channels to YYY in [001-255]'
        print '  rngY - set range on all channels to Y in [0-9]'
        print '  phY - set phase on all channels to Y in (A,B,C)'
        print '  pol0 - set polarity on all channels to 0 (default)'
        print '  pol1 - set polarity on all channels to 1'
        print 'sys'
        print '  _on - start sending real-time packets'
        print '  off - stop sending real-time packets'
        print '  pktXX - set packet format to XX in [00-99]'
        print '  ivlXXX - set packet send interval to XXX in [001-256]'
        print '  pksN - set packet chunk size to N in [80-65000]'
        print '  pkiN - set packet chunk interval time to N in 1-65000]'
        print '  bffN - set maximum buffer size to N in [10-1700]'
        print '  fl1X - enable(1) or disable(0) wifi hardware flow control'
        print '  fl2X - enable(1) or disable(0) zigbee flow control'
        print '  tptXXX - set potential transformer type to XXX in [001-255]'
        print '  vrgXX - set potential transformer range to XX in [00-10]'
        print '  rspX - reset pulse counter X in [1-4]'
        print '  rstp - set all four pulse counters to zero'
        print '  rsta - zero all counters (channel, pulse, seconds)'
        print '  rsts - set seconds counter to zero'
        print '  rstcXX - set watt-second counter for channel XX to zero'
        print '  hzX - set line frequency to 50Hz (1) or 60Hz (0)'
        print '  dtmYY,MM,DD,hh,mm,ss - set date and time'
        print '  urlURL - set http post url'
        print '  ureURLEXT - set extension to post url'
        print '  urtTOKEN - set token for http post'
        print '  urkXKEY - set additional keys for post'
        print '  urnNODE - set data host device node'
        print '  zbr - reset xbee module'
        print '  rsw - reset wifi module'
        print '  warX - set wifi auto reset interval to X in [30-255]'
        print '  wrrX - set wifi no-http-response reset interval X in [2-255]'
        print '  zarX - set zigbee auto reset interval to X in [30-255]'
        print '  zrrX - set zigbee no-http-response reset interval X in [2-255]'
        print '  hdwX - set hardware configuration X in [0-4]'
        print '  wbr - reset baud to 19200'
        print '  rbc - restart the com processor'
        print '  rbe - restart the engine processor'
        print '  phX - master polyphase setting X in [0-3]'
        print '  epcXXX,XXX,XXX,XXX,XXX,XXX - set channels to post'
        print '  eppXXX - set pulse counters to post XXX in [000-015]'
        print '  eptXXX - set one-wire channels to post XXX in [000-255]'
        print '  cmxXX - set max channel to be used XX in [01-48]'
        print '  tsbX - set number of thingspeak blocks X in [1-8]'
        print '  br1 - enter bridge mode'
        print '  br0 - exit bridge mode'
        print '  fpX - set to factory preset X in [0-4]'
        print 'tmp'
        print '  romXVVVVVVVV - save sensor rom code'
        print '  enX - enable channel X'
        print '  dsX - disable channel X'
        print '  rowX - read then save'
        print '  dgc - set units to Celsius'
        print '  dgf - set units to Fahrenheit'
        print 'pls'
        print '  enX - enable counter X'
        print '  dsX - disable counter X'
        print 'd X - specify serial number of device for multiplexed comms'
        print 'q - exit btcfg'


class BaseComm(object):
    def __init__(self):
        pass

    def setup(self):
        pass

    def cleanup(self):
        pass

    def send(self, str):
        pass

    def recv(self, sz):
        return ''

class SerialComm(BaseComm):
    def __init__(self, port, rate, timeout):
        super(SerialComm, self).__init__()
        if not serial:
            print 'Error: serial module could not be imported.'
            sys.exit(1)
        self._port = port
        self._baudrate = int(rate)
        self._timeout = timeout
        self._conn = None
        dbgmsg('serial port: %s' % self._port)
        dbgmsg('baud rate: %d' % self._baudrate)

    def cleanup(self):
        self.close()

    def open(self):
        dbgmsg('open serial port %s' % self._port)
        self._conn = serial.Serial(self._port, self._baudrate, timeout=self._timeout)

    def close(self):
        if self._conn:
            dbgmsg('close serial port')
            self._conn.close()
            self._conn = None

    # Send string 'str' to serial port
    def send(self, str):
        dbgmsg('sending %s' % str)
        self._conn.write(str)

    # Send value 'v' as unsigned char to serial port
    def sendbyte(self, v):
        dbgmsg('sending binary char %d' % v)
        self._conn.write(struct.pack('<B', v))

    def recv(self, sz=SERIAL_BUFFER_SIZE):
        resp = ''
        try:
            dbgmsg('waiting for %d bytes' % sz)
            resp = self._conn.read(sz)
            while len(resp) < sz:
                dbgmsg('waiting for %d bytes' % (sz-len(resp)))
                resp += self._conn.read(sz - len(resp))
        except Exception, e:
            dbgmsg('exception while receiving')
            raise e
        return resp

class SocketComm(BaseComm):
    def __init__(self, host, port, timeout):
        if not host:
            print 'Socket Error: no host specified'
            sys.exit(1)
        super(SocketComm, self).__init__()
        self._host = host
        self._port = int(port)
        self._timeout = int(timeout)
        self._sock = None
        socket.setdefaulttimeout(self._timeout)
        dbgmsg('host: %s' % self._host)
        dbgmsg('port: %d' % self._port)
        dbgmsg('timeout: %d' % self._timeout)

    def cleanup(self):
        self.close()

    def open(self):
        dbgmsg('open socket to %s:%d' % (self._host, self._port))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._host, self._port))

    def close(self):
        if self._sock:
            dbgmsg('close socket')
            self._sock.close()
            self._sock = None

    def send(self, str):
        dbgmsg('sending %s' % str)
        self._sock.sendall(str)

    def recv(self, sz=IP_BUFFER_SIZE):
        resp = ''
        try:
            dbgmsg('waiting for %d bytes' % sz)
            resp = self._sock.recv(sz)
            while len(resp) < sz:
                dbgmsg('waiting for %d bytes' % (sz-len(resp)))
                resp += self._sock.recv(sz - len(resp))
        except socket.timeout, e:
            dbgmsg('timeout while receiving')
            pass
        except Exception, e:
            dbgmsg('exception while receiving')
            raise e
        return resp


class Configurator(object):
    def __init__(self, dev, com):
        self.dev = dev
        self.com = com
        self.running = True

    def setup(self):
        self.dev.setup()
        self.com.setup()

    def cleanup(self):
        self.com.cleanup()
        self.dev.cleanup()

    def getcmd(self, prompt=''):
        p = 'btcfg: '
        if len(prompt) > 0:
            p += '%s: ' % prompt
        return raw_input(p)

    def processcmd(self, cmd):
        if cmd == 'q' or cmd == 'quit':
            self.running = False
        elif cmd == 'h' or cmd == 'help' or cmd == '?':
            self.dev.printhelp()
        elif self.dev.isvalidcmd(cmd):
            self.dev.sendcmd(cmd, self.com)
        elif len(cmd) > 0:
            print "unknown command '%s', type 'help' for list of options" % cmd

    def run(self):
        try:
            self.setup()
            while self.running:
                try:
                    cmd = self.getcmd(dev.getprompt())
                    self.processcmd(cmd)
                except KeyboardInterrupt, e:
                    raise e
                except Exception, e:
                    if LOGLEVEL >= LOG_DEBUG:
                        traceback.print_exc()
                    else:
                        errmsg(e)
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception, e:
            if LOGLEVEL >= LOG_DEBUG:
                traceback.print_exc()
            else:
                errmsg(e)
            sys.exit(1)
        finally:
            try:
                self.cleanup()
            except:
                if LOGLEVEL >= LOG_DEBUG:
                    traceback.print_exc()
                else:
                    errmsg(e)


if __name__ == '__main__':
    parser = optparse.OptionParser(version=__version__)

    parser.add_option('-q', '--quiet', action='store_true', dest='quiet', default=False, help='quiet output')
    parser.add_option('-v', '--verbose', action='store_false', dest='quiet', default=False, help='verbose output')
    parser.add_option('--debug', action='store_true', default=False, help='debug output')

    group = optparse.OptionGroup(parser, 'device options')
    group.add_option('--device-type', dest='device', default=DEFAULT_DEVICE_TYPE, help='one of gem, ecm1240, or ecm1220')
    parser.add_option_group(group)

    group = optparse.OptionGroup(parser, 'serial options')
    group.add_option('--serial', action='store_true', dest='serial', default=False, help='communicate via serial port')
    group.add_option('--serial-port', dest='serial_port', help='serial port')
    group.add_option('--serial-baud', dest='serial_baud', help='serial baud rate')
    group.add_option('--serial-timeout', help='serial timeout')
    parser.add_option_group(group)

    group = optparse.OptionGroup(parser, 'tcp/ip options')
    group.add_option('--ip', action='store_true', dest='ip', default=False, help='communicate via tcp/ip socket')
    group.add_option('--ip-host', help='ip host')
    group.add_option('--ip-port', help='ip port')
    group.add_option('--ip-timeout', help='tcp/ip timeout')
    parser.add_option_group(group)

    (options, args) = parser.parse_args()

    if options.quiet:
        LOGLEVEL = LOG_ERROR
    if options.debug:
        LOGLEVEL = LOG_DEBUG

    if options.serial:
        com = SerialComm(options.serial_port or SERIAL_PORT,
                         options.serial_baud or SERIAL_BAUD,
                         options.serial_timeout or SERIAL_TIMEOUT)
    elif options.ip:
        com = SocketComm(options.ip_host,
                         options.ip_port or IP_PORT,
                         options.ip_timeout or IP_TIMEOUT)
    else:
        print 'Please specify a communication mechanism:'
        print '  --serial     communicate via serial'
        print '  --ip         communicate via tcp/ip socket'
        sys.exit(1)

    if options.device == DEV_GEM:
        dev = GEMDevice()
    elif options.device == DEV_ECM1240:
        dev = ECM1240Device()
    elif options.device == DEV_ECM1220:
        dev = ECM1220Device()
    else:
        print "Unsupported device type '%s'" % options.device
        print 'supported devices include:'
        for dev in DEVICE_TYPES:
            print '  %s' % dev
        sys.exit(1)

    cfg = Configurator(dev, com)
    cfg.run()

    sys.exit(0)
