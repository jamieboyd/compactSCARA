#! /usr/bin/python
#-*-coding: utf-8 -*
from abc import ABCMeta, abstractmethod
import os
import inspect
import serial
import struct
import serial.tools.list_ports
from array import array

portList=serial.tools.list_ports.comports()

class SCARA:
    # data about robot
    #defaultPORT = '/dev/cu.usbmodem141203'
    defaultPORT = 'COM9'
    defaultBAUD = 115200
    length_L1 = 150
    length_L2 = 150
    pulses_per_degree = 9.48867
    zaxis_steps_per_mm = 40

    # command codes corresponding to positions in msp430 command array
    zeroEncodersCode =0
    getPosCode = 1
    setMtrCode = 2
    emStopCode = 3
    emResetCode = 4
    zGetPosCode = 5
    zZeroCode = 6
    zSetUpperCode = 7
    zSetUpperHereCode = 8
    zSetLowerCode = 9
    zSetLowerHereCode = 10
    zSetSpeedCode = 11
    zGotoPosCode = 12
    zJogStartCode = 13
    zJogStopCode = 14
    movjCode = 15
    movjCoordCode = 16
    movlCode =17
    movecCode = 18
    # SCARA state 
    # format = error, motor1 PWM, motor2 PWM, toolData, encoder 1 counts, encoder 2 counts, z axis counts,  
    # size = 4 + 3*2 = 10
    # data formats for packing and unpacking streams of bytes. Remember that when sending bytes to the
    # microcontroller, multi-byte values must start on an even byte, note use of pad bytes (x) to ensure this
    # the number of bytes the microcontroller is expecting to read must include the pad bytes
    # Note that the msp430 has little endian byte order, coded as <
    posReceiveFormat = struct.Struct ('<Bhh')    # unsigned byte error code and signed word arm positions
    posSendFormat = struct.Struct('<Bxhh')
    zAxisConfirmSendFormat = struct.Struct ('<BBh')     # unsigned byte function code, confirm request and signed word position
    zAxisSendFormat = struct.Struct ('<Bxh')
    zAxisSpeedSendFormat = struct.Struct ('<BxH')
    zAxisReceiveFormat = struct.Struct ('<Bh')
    mtrsFormat = struct.Struct ('<Bxx')
    posFloatFormat = struct.Struct('<BBff')
    movCformat = struct.Struct ('<BBhhf')
    noErrCode =  0          # first byte of results msp430 sends must be one of these error codes
    EmStoppedCode = 1       #
    ZaxisOverCode = 2
    ZaxisUnderCode = 3
    armSolLeftCode =0
    armSolRightCode =1
    def __init__(self, port, baud):
        self.serPort = port
        self.serBaud = baud
        self.ser = serial.Serial(port, baud, timeout = 10)
        self.state = dict(L1pos = 0, L2pos=0, Zpos=0, lastErr =0)

    def zeroEncoders (self):
        self.ser.write ((SCARA.zeroEncodersCode).to_bytes(1, byteorder='little', signed=False))

    def getPos (self):
        self.ser.write ((SCARA.getPosCode).to_bytes(1, byteorder='little', signed=False))
        buffer=self.ser.read (5)
        posData = SCARA.posReceiveFormat.unpack (buffer)
        self.state['lastErr'] = posData [0]
        if (posData [0] == SCARA.noErrCode):
            self.state['L1pos'] = posData[1]
            self.state['L2pos'] = posData[2]
        else:
           print("Error: ", self.state['lastErr'])

    def setMtrs (self, M1val, M2val, doConfirm=0):
        buffer = (SCARA.setMtrCode).to_bytes(1, byteorder='little', signed=False)\
                 +(doConfirm).to_bytes(1, byteorder='little', signed=False)\
                 + (M1val).to_bytes(2, byteorder='little', signed=True)\
                 + (M2val).to_bytes(2, byteorder='little', signed=True)
        print (buffer)       

        self.ser.write (buffer)
        if doConfirm:
            buffer = self.ser.read (3)
            print (buffer)
            self.state['lastErr'] = SCARA.mtrsSendFormat.unpack (buffer)[0]
            if (self.state['lastErr'] != SCARA.noErrCode):
                print ("Error: ", self.state['lastErr'])

    #does an emergency stop     
    def EmStop (self):
        self.ser.write ((SCARA.emStopCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state['lastErr'] = SCARA.EmStoppedCode

    #resets emergency stop, nothing is returned     
    def EmStopReset (self):
        self.ser.write ((SCARA.emResetCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state['lastErr'] = SCARA.noErrCode
        
    def zeroZaxis(self):
        self.ser.write ((SCARA.zZeroCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state ['Zpos'] = 0

    # gets Z position
    def getZpos (self):
        self.ser.write((SCARA.zGetPosCode).to_bytes (1, byteorder = 'little', signed = False))
        buffer=self.ser.read (3)
        data = SCARA.zAxisReceiveFormat.unpack (buffer)
        self.state['lastErr'] = data[0]
        if self.state['lastErr'] != SCARA.noErrCode:
            print ("Error: ", self.state['lastErr'])
        else:
            self.state['Zpos'] = data[1]/SCARA.zaxis_steps_per_mm
        
    def setZSpeed (self, speed):
        buffer = SCARA.zAxisSpeedSendFormat.pack(SCARA.zSetSpeedCode, speed)
        self.ser.write (buffer)
    
    def gotoZpos (self, position, doConfirm = 0):
        buffer = SCARA.zAxisConfirmSendFormat.pack (SCARA.zGotoPosCode, doConfirm, position * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)
        if doConfirm:
            buffer = self.ser.read (3)
            self.state['lastErr'] = struct.unpack('<Bxx',buffer)

    def jogZstart (self, speedDir):
        buffer = SCARA.zAxisSendFormat.pack(SCARA.zJogStartCode, speedDir/SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)


    def jogZstop (self):
        self.ser.write ((SCARA.zJogStopCode).to_bytes (1, byteorder = 'little', signed = False))

    def setZupperLimit (self, limit):
        buffer = SCARA.zAxisSendFormat.pack (SCARA.zSetUpperCode, limit * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)

    def setZlowerLimit (self, limit):
        buffer = SCARA.zAxisSendFormat.pack (SCARA.zSetLowerCode, limit * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)
    
    def setZupperHere (self):
        self.ser.write ((SCARA.zSetUpperHereCode).to_bytes (1, byteorder = 'little', signed = False))
        
    def setZlowerHere (self):
        self.ser.write ((SCARA.zSetLowerHereCode).to_bytes (1, byteorder = 'little', signed = False))

    def moveJ (self, endAng1, endAng2):
        buffer = SCARA.posSendFormat.pack (SCARA.movjCode, endAng1, endAng2)
        self.ser.write (buffer)

    def moveJcoord (self, xPos, yPos, armSol):
          buffer = SCARA.posFloatFormat.pack (SCARA.movjCoordCode, armSol, xPos, yPos)
          self.ser.write (buffer)

    def moveL(self, xPos, yPos, armSol):
          buffer = SCARA.posFloatFormat.pack (SCARA.movlCode, armSol, xPos, yPos)
          self.ser.write (buffer)
            


    def moveC (self, startAngle, endAngle, radius, armSol):
         buffer = SCARA.movCformat.pack(SCARA.movecCode, armSol, startAngle, endAngle, radius)
         self.ser.write (buffer)

    #def moveCustom (self, 
        

if __name__ == '__main__':
    clem = SCARA (SCARA.defaultPORT, SCARA.defaultBAUD) 
    
    
"""
for i in range (0,160):
    serial_data = ser.read(2)#;print (serial_data)
    decVal = int.from_bytes(serial_data, byteorder='little', signed=True)
    print (decVal)









int.to_bytes(length, byteorder, *, signed=False)
#Return an array of bytes representing an integer.

-1024).to_bytes(10, byteorder='little', signed=True)
b'\x00\xfc\xff\xff\xff\xff\xff\xff\xff\xff'


import struct
...

try:
    ieee754_data = my_serial.read(4)
    my_float = struct.unpack('f', ieee754_data)
catch:
    # I/O Error, or junk data
    my_float = 0.0
And packing:

 ieee754_data = struct.pack('f', my_float)
 try:
     my_serial.write(ieee754_data)
 catch:
     pass #TODO - I/O Error
"""
