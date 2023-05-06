#! /usr/bin/python
#-*-coding: utf-8 -*
"""
class SCARA - a Python class to control the SCARA Robot by sending commands and receiving data using serial communication.
All commands are sent to the microcontroller in binary format as packed byte streams. Each command has a matching
microcontroller function that unpacks the data and carries out the command.

The command's id number (as an unsigned byte)is the first (and sometimes the only) byte that is sent as part of a command.
Function arguments are packed into subsequent bytes, whose number and type vary according the function. The command parser
running on the microcontroller will read the first byte to get this id number, which tells it how many more bytes to read
into a buffer, and which function to call, with the buffer as an argument. The function knows how the data need to be unpacked.
Similarly, data returned from the robot will also be packed as bytes, which will be unpacked and used to update the state
of the SCARA object.

Author: Jamie Boyd/Matthew Wonneburg
Created on: April 1, 2022
"""
from abc import ABCMeta, abstractmethod
import os
import inspect
import serial
import struct
import serial.tools.list_ports
from array import array

portList=serial.tools.list_ports.comports()

class SCARA:
    # class data about robot
    # defaultPORT = '/dev/cu.usbmodem141203' #for mac
    defaultPORT = 'COM9'
    defaultBAUD = 115200
    length_L1 = 150
    length_L2 = 150
    pulses_per_degree = 9.48867
    zaxis_steps_per_mm = 40

    # class data - command codes
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
    # data formats for packing and unpacking streams of bytes. Remember that when packing bytes
    # multi-byte values must start on an even byte, note use of pad bytes (x) to ensure this
    # the number of bytes the microcontroller is expecting to read must include the pad bytes
    # Note that the msp430 has little endian byte order, coded as <
    posReceiveFormat = struct.Struct ('<Bhh')    # unsigned byte error code and signed word arm positions
    posSendFormat = struct.Struct('<Bxhh')
    zAxisConfirmSendFormat = struct.Struct ('<BBh')     # unsigned byte function code, confirmation request and signed word position
    zAxisSendFormat = struct.Struct ('<Bxh')
    zAxisReceiveFormat = struct.Struct ('<Bh')      # unsigned byte error code, signed byte steps from 0
    zAxisSpeedSendFormat = struct.Struct ('<BxH')   # unsigned byte function code, pad byte, unsigned word step frequency
    zAxisSpeedReceiveFormat = struct.Struct ('<BH') # unsigned byte error code unsigned word step frequency
    mtrsFormat = struct.Struct ('<Bxx')
    posFloatFormat = struct.Struct('<BBff')
    movCformat = struct.Struct ('<BBhhf')
    noErrCode =  0          # first byte of results msp430 sends must be one of these error codes
    EmStoppedCode = 1
    ZaxisOverCode = 2
    ZaxisUnderCode = 3
    
    def __init__(self, port, baud):
        self.serPort = port
        self.serBaud = baud
        self.ser = serial.Serial(port, baud, timeout = 10)
        self.state = dict(L1pos = 0, L2pos=0, Zpos=0, lastErr =0)
        # SCARA state is a dictionary containing information about current state of the robot, things like
        # error code, motor1 PWM, motor2 PWM, toolData, encoder 1 counts, encoder 2 counts, z axis counts 

    """
    Sends command to zero the encoder positions for both motors
    no arguments
    """
    def zeroEncoders (self):
        self.ser.write ((SCARA.zeroEncodersCode).to_bytes(1, byteorder='little', signed=False))

    """
    Sends command to get the current encoder counts for both encoders
    receives encoder counts, updates state
    no arguments
    """
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

    """
    Sends command to manually set the PWM output value for the motors
    THIS WILL START THE MOTORS MOVING UNTIL THEY ARE SET TO 0
    if doConfirm, receives a confirmation message from the robot
    arguments:
    M1val - pwm value for L1 motor
    M2val - pwm value for L2 motor
    doConfirm - TRUE to wait for confirmation from the robot
    """
    def setMtrs (self, M1val, M2val, doConfirm=0):
        buffer = (SCARA.setMtrCode).to_bytes(1, byteorder='little', signed=False)\
                 +(doConfirm).to_bytes(1, byteorder='little', signed=False)\
                 + (M1val).to_bytes(2, byteorder='little', signed=True)\
                 + (M2val).to_bytes(2, byteorder='little', signed=True)
        self.ser.write (buffer)
        if doConfirm:
            buffer = self.ser.read (3)
            print (buffer)
            self.state['lastErr'] = SCARA.mtrsSendFormat.unpack (buffer)[0]
            if (self.state['lastErr'] != SCARA.noErrCode):
                print ("Error: ", self.state['lastErr'])

    """
    sends command to trigger an emergency stop
    no arguments
    """
    def EmStop (self):
        self.ser.write ((SCARA.emStopCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state['lastErr'] = SCARA.EmStoppedCode

    """
    sends command to reset emergency stop
    no arguments
    """
    def EmStopReset (self):
        self.ser.write ((SCARA.emResetCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state['lastErr'] = SCARA.noErrCode

    """
    Sends command to do a joint-interpolated movement from current position to given agular coordinates
    arguments:
    endAng1: angular position for L1, shoulder joint
    endAng1: angular position for L2, elbow joint
    """
    def moveJ (self, endAng1, endAng2):
        buffer = SCARA.posSendFormat.pack (SCARA.movjCode, endAng1, endAng2)
        self.ser.write (buffer)

    """
    Sends command to do a joint-interpolated movement from current position to given X,Y location
    arguments:
    xPos: X position of tool tip, in mm
    yPos: Y position of tool tip, in mm
    armSol: 1 for right arm solution (elbow bends counterclockwise) , 0 for left arm sol (elbow bends clockwise)
    """
    def moveJcoord (self, xPos, yPos, armSol):
          buffer = SCARA.posFloatFormat.pack (SCARA.movjCoordCode, armSol, xPos, yPos)
          self.ser.write (buffer)

    """
    Sends command to move in a linear fashion from current position to given X,Y location
    arguments:
    xPos: X position of tool tip, in mm
    yPos: Y position of tool tip, in mm
    armSol: 1 for right arm solution (elbow bends counterclockwise) , 0 for left arm sol (elbow bends clockwise)
    """
    def moveL(self, xPos, yPos, armSol):
          buffer = SCARA.posFloatFormat.pack (SCARA.movlCode, armSol, xPos, yPos)
          self.ser.write (buffer)

    """
    Sends command to move in an arc from current position to a position defined by an
    arc from starting angle to given angle along a circle of given radius
    arguments:
    startAngle: starting angle position along arc
    endAngle: ending angle position along arc
    radius: radius of arc
    armSol: 1 for right arm solution (elbow bends counterclockwise) , 0 for left arm sol (elbow bends clockwise)
    """        
    def moveC (self, startAngle, endAngle, radius, armSol):
         buffer = SCARA.movCformat.pack(SCARA.movecCode, armSol, startAngle, endAngle, radius)
         self.ser.write (buffer)

    """
    sends command to zero the Z axis
    no arguments
    """   
    def zeroZaxis(self):
        self.ser.write ((SCARA.zZeroCode).to_bytes (1, byteorder = 'little', signed = False))
        self.state ['Zpos'] = 0

    """
    sends command to get Z position from robot
    receives Z-position data from robot and updates state
    no arguments
    """
    def getZpos (self):
        self.ser.write((SCARA.zGetPosCode).to_bytes (1, byteorder = 'little', signed = False))
        buffer=self.ser.read (3)
        data = SCARA.zAxisReceiveFormat.unpack (buffer)
        self.state['lastErr'] = data[0]
        if self.state['lastErr'] != SCARA.noErrCode:
            print ("Error: ", self.state['lastErr'])
        else:
            self.state['Zpos'] = data[1]/SCARA.zaxis_steps_per_mm

    """
    sends command to set speed for z-axis movement in mm/sec
    arguments:
    speed: desired movement speed for Z-axis in mm/sec.
    """  
    def setZSpeed (self, speed):
        #  mm/sec * steps/mm = steps/sec = Hz for updating, which is what microcontroller wants
        buffer = SCARA.zAxisSpeedSendFormat.pack(SCARA.zSetSpeedCode, speed * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)
        buffer = self.ser.read (3)  # 1byte for error code, 2 bytes for unsigned int hz
        data = SCARA.zAxisReceiveFormat.unpack (buffer)
        print ("Actual Speed: ",data[1]/SCARA.zaxis_steps_per_mm, " mm/sec") # steps/sec / steps/mm = mm/sec
        

    """
    sends command to send Z-axis to a given location defined from setting the zero axis
    arguments:
    position: Z-axis position in mm
    doConfirm: non-zero to request confirmation from robot
    """  
    def gotoZpos (self, position, doConfirm = 0):
        buffer = SCARA.zAxisConfirmSendFormat.pack (SCARA.zGotoPosCode, doConfirm, position * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)
        if doConfirm:
            buffer = self.ser.read (3)
            self.state['lastErr'] = struct.unpack('<Bxx',buffer)

    """
    sends command to start Z-axis moving with a particular speed and direction
    arguments:
    speedDir: speed in mm/sec, + values going down, -values coming up
    """ 
    def jogZstart (self, speedDir):
        buffer = SCARA.zAxisSendFormat.pack(SCARA.zJogStartCode, speedDir/SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)

    """
    sends command to stop Z-axis movement
    no arguments
    """ 
    def jogZstop (self):
        self.ser.write ((SCARA.zJogStopCode).to_bytes (1, byteorder = 'little', signed = False))

    """
    sends command to set a minimum allowed upper limit for Z-axis travel in mm from 0 position
    first set a zero before using this command
    arguments:
    limit: the limit, in mm
    """ 
    def setZupperLimit (self, limit):
        buffer = SCARA.zAxisSendFormat.pack (SCARA.zSetUpperCode, limit * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)

    """
    sends command to set a maximim allowed lower limit for Z-axis travel in mm from 0 position
    first set a zero before using this command
    arguments:
    limit: the limit, in mm
    """
    def setZlowerLimit (self, limit):
        buffer = SCARA.zAxisSendFormat.pack (SCARA.zSetLowerCode, limit * SCARA.zaxis_steps_per_mm)
        self.ser.write (buffer)

    """
    sends command to set upper limit to current position of Z-axis
    first set a zero before using this command
    no arguments
    """  
    def setZupperHere (self):
        self.ser.write ((SCARA.zSetUpperHereCode).to_bytes (1, byteorder = 'little', signed = False))

    """
    sends command to set lower limit to current position of Z-axis
    first set a zero before using this command
    no arguments
    """     
    def setZlowerHere (self):
        self.ser.write ((SCARA.zSetLowerHereCode).to_bytes (1, byteorder = 'little', signed = False))


         
"""
If run as main funtion, make a SCARA object named clem using default port and baud that can be used from
the command line. Run fuunctions as, for instance, clem.zeroEncoders()
"""
if __name__ == '__main__':
    clem = SCARA (SCARA.defaultPORT, SCARA.defaultBAUD) 
