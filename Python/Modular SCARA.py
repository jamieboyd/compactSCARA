from cgitb import enable
from os import popen
from tkinter import *
from array import array
import time
import math
import tkinter.ttk as ttk
from SCARA import SCARA
from zlib import DEFLATED
import serial.tools.list_ports
from array import array
portList=serial.tools.list_ports.comports()
portLength = len(portList)
baud = 115200

robot = None
L1array = array('h', (i for i in range(401)))
L2array = array('h', (i for i in range(401)))


def selectPort(portOption):
    global robot 
    pop.grab_release()
    pop.destroy()
    robot = SCARA(portOption.name, SCARA.defaultBAUD)
    # robot = SCARA( '/dev/' +  portOption.name, SCARA.defaultBaud) #for mac

#----------------------- GUI commands -------------------------------

def moveJ():
    screen.grab_release()
    robot.moveJ(int(eJ1.get()), int(eJ2.get()))
    screen.grab_set()
    
def moveJcoord():
    screen.grab_release()
    robot.moveJcoord(float(eX.get()), float(eY.get()), armSolution.get())
    screen.grab_set()

def moveL():
    screen.grab_release()
    robot.moveL(float(eX.get()), float(eY.get()), armSolution.get())
    screen.grab_set()

def moveC():
    screen.grab_release()
    startAnglePulses = eStartAng.get()*SCARA.pulses_per_degree
    endAnglePulses = eEndAng.get()*SCARA.pulses_per_degree
    robot.moveC(startAnglePulses, endAnglePulses, eArc.get(), armSolution.get())
    screen.grab_set()

def moveZ():
    screen.grab_release()
    toolMovement = eTool.get()
    robot.moveC(toolMovement)
    screen.grab_set()

def resetPosition():
    screen.grab_release()
    robot.zeroEncoders()
    screen.grab_set()

def homeZaxis():
    screen.grab_release()
    robot.zeroZaxis()
    screen.grab_set()

def stopProgram():
    pass
    robot.emStop()

def runProgram():
    pass
    robot.emStopReset()

def quitProgram():
    robot.doStop=1
    pass
    quit()
#-------------------------------------------------------------------------------


#----------------------User created Move J -------------------------------------
def moveJProgram():
    screen.grab_release() # disables the GUI command screen
    global indexCMD
    indexCMD = 8
    # code to send both joint angles to the msp430
    screen.grab_set() # enables the GUI command screen


#--------------------- User created Move L ---------------------------------------
def moveLProgram():
    screen.grab_release()
    global A_MAX_LINEAR
    #Xa, user provided
    Xa = 0
    #Xb, user provided
    #Ya, user provided
    Ya = 0
    #Yb, user provided
    deltaX =0
    deltaY =0
    deltaD = 0
    tInc = 0

    deltaD = math.sqrt(pow(deltaX, 2) + pow(deltaY, 2))
    timeForMove = math.sqrt((abs(deltaD)*2*PI)/A_MAX_LINEAR)
    w = (2*(PI))/timeForMove
    arrayLength = (timeForMove/T_UPDATE)+1

    for tInc in range(arrayLength):
        # now you must determine all of the x,y coordinates for every
        # index of the arrayLength throughout the line based on the time of the move.

        #The displacement along the line from 0 to D with respect to time is d(t).
        d = ((A_MAX_LINEAR * (tInc*T_UPDATE))/w -  (A_MAX_LINEAR)*(math.sin(w*(tInc*T_UPDATE)))/pow(w,2))

        #from the displacement along the line, and the known start and end coordinates of the line,
        # use this formula to compute the corrisponding (x, y) coordinate of the move
        x = Xa + (d*deltaX)/deltaD 
        y = Ya + (d*deltaY)/deltaD
        
        # to do:
        # check that the x, y coordinates are ouside of the robot's base boundary.
        # compute the joint angles using inverse kinematics.
        # convert the joint angles from degrees to pulses of the motor using the constant 9.48866 Pulses of the motor per degree of rotation


    for tInc in range(arrayLength):
        # in send over two array values in x,y coordinates if the array was computed successfully
        indexCMD = 9 
    
    screen.grab_set()



screen = Tk()
screen.title("Modular SCARA")
screen.geometry('1100x800')
#screen.config(bg='#DCDCDC')

blank = Label(screen, text="  ")
blank.grid(row=0, column=0, pady=5) 

armSolution = IntVar()
toolPosition = IntVar()
indexCMD = IntVar()
displayMainPage=0

portOption = StringVar(screen)
portOption.set("  Select  ") # default value


#----------------------- GUI commands -------------------------------
    

global pop
pop = Toplevel(screen)
pop.title("COMM SELECT")
pop.geometry("250x150")
pop.grab_set()

pop_Label = Label(pop, text="Select a COMM port")
pop_Label.pack()

my_frame = Frame(pop)
my_frame.pack()

portMenu = OptionMenu(pop, portOption, *portList, command=selectPort)
portMenu.pack()

#------------------- Label section---------------------
theta1Label = Label(screen, text="Joint 1 angle (deg):", font=('Calibri 14'))
theta1Label.grid(row=2, column=0)
theta2Label = Label(screen, text="Joint 2 angle (deg):",font=('Calibri 14'))
theta2Label.grid(row=2, column=1)

xLabel = Label(screen, width =20, text="\n\nX position (cm):",font=('Calibri 14'))
xLabel.grid(row=4, column=0)
yLabel = Label(screen, text="\n\nY position (cm):",font=('Calibri 14'))
yLabel.grid(row=4, column=1)

startAngLabel = Label(screen, width =20, text="\n\nStarting arc angle:",font=('Calibri 14'))
startAngLabel.grid(row=6, column=0)
endAngLabel = Label(screen, width=20, text="\n\nEnding arc angle:",font=('Calibri 14'))
endAngLabel.grid(row=6, column=1)
radiusLabel = Label(screen, width=20, text="\n\nArc Radius:",font=('Calibri 14'))
radiusLabel.grid(row=6, column=2)

startAngLabel = Label(screen, width =20, text="\n\nArm Solution:",font=('Calibri 14'))
startAngLabel.grid(row=8, column=0)

startAngLabel = Label(screen, width =20, text="\n\nTool Position:",font=('Calibri 14'))
startAngLabel.grid(row=10, column=0)

startAngLabel = Label(screen, width =20, text="\n\nMove Tool by (cm):",font=('Calibri 14'))
startAngLabel.grid(row=10, column=2)

startAngLabel = Label(screen, width =20, text="\n\nCommand Movement:",font=('Calibri 14'))
startAngLabel.grid(row=12, column=0)

startAngLabel = Label(screen, width =20, text="\n\nRobot Settings:",font=('Calibri 14'))
startAngLabel.grid(row=14, column=0)


#----------------------Input field section -----------------------
# joint angle input fields
eJ1 = Entry(screen, width=15, borderwidth=5)
eJ1.grid(row=3, column=0)
eJ1.insert(0, "0")
eJ1.focus

eJ2 = Entry(screen, width=15, borderwidth=5)
eJ2.grid(row=3, column=1)
eJ2.insert(0, "0")



# arm coordiante input fields
eX = Entry(screen, width=15, borderwidth=5)
eX.grid(row=5, column=0)
eX.insert(0, "30")
eY = Entry(screen, width=15, borderwidth=5)
eY.grid(row=5, column=1)
eY.insert(0, "0")

# cicular motion input fields
eStartAng = Entry(screen, width=15, borderwidth=5)
eStartAng.grid(row=7, column=0)
eStartAng.insert(0, "0")
eEndAng = Entry(screen, width=15, borderwidth=5)
eEndAng.grid(row=7, column=1)
eEndAng.insert(0, "360")
eArc = Entry(screen, width=15, borderwidth=5)
eArc.grid(row=7, column=2)
eArc.insert(0, "5")

# tool position entry box in centimeters
eTool = Entry(screen, width=15, borderwidth=5)
eTool.grid(row=11, column=2)
eTool.insert(0, "0")

#------------------------ Button section --------------------------------------------
leftB = Radiobutton(screen, width= 15, text="left", variable=armSolution, value =1,font=('Calibri 12'))
leftB.grid(row=9, column=0)
rightB = Radiobutton(screen, width= 15, text="right", variable=armSolution, value =0,font=('Calibri 12'))
rightB.grid(row=9, column=1)
upB = Radiobutton(screen, width= 15, text="tool up", variable=toolPosition, value=0,font=('Calibri 12'))
upB.grid(row=11, column=0)
dnB = Radiobutton(screen, width= 15, text="tool down", variable=toolPosition, value=1,font=('Calibri 12'))
dnB.grid(row=11, column=1)


buttonMJ = Button(screen, width = 15, text= "Move J", borderwidth=5, padx=15, pady=5, command=moveJ)
buttonMJ.grid(row=13, column=0)
buttonMJC = Button(screen, width = 15, text= "Move J coordinate",borderwidth=5, padx=15, pady=5, command=moveJcoord)
buttonMJC.grid(row=13, column=1)
buttonML = Button(screen, width = 15, text= "Move Linear", borderwidth=5,padx=15, pady=5, command=moveL)
buttonML.grid(row=13, column=2)
buttonMC = Button(screen, width = 15, text= "Move Circular", borderwidth=5,padx=15, pady=5, command=moveC)
buttonMC.grid(row=13, column=3)
buttonMT = Button(screen, width = 15, text= "Move Z by distance", borderwidth=5,padx=15, pady=5, command=moveZ)
buttonMT.grid(row=13, column=4)

buttonDisp = Button(screen, width = 15, text= "Display Position", borderwidth=5,padx=15, pady=5) #command display position
buttonDisp.grid(row=15, column=0)
buttonReset = Button(screen, width = 15, text= "Reset Position", borderwidth=5,padx=15, pady=5, command=resetPosition) #command reset position
buttonReset.grid(row=15, column=1)
buttonReset = Button(screen, width = 15, text= "Home Z-Axis", borderwidth=5,padx=15, pady=5, command=homeZaxis) #command reset position
buttonReset.grid(row=15, column=2)
button = Button(screen, width = 15, text= "STOP", borderwidth=5, bg="red", fg="white", padx=15, pady=5, command=stopProgram)
button.grid(row=15, column=3)
button = Button(screen, width = 15, text= "continue", borderwidth=5, padx=15, pady=5, command=runProgram)
button.grid(row=15, column=4)
button = Button(screen, width = 15, text= "End Program", borderwidth=5,padx=15, pady=5,command = quitProgram)
button.grid(row=15, column=5)

A_MAX_LINEAR = 10
T_UPDATE = 0.01
PI = 3.141593



screen.mainloop()
