import serial
from time import sleep

from tkinter import *

from tkinter.ttk import *

window = Tk()

window.title("SCARA Controls")

selected = IntVar()

rad1 = Radiobutton(window,text='First', value=1, variable=selected)

rad2 = Radiobutton(window,text='Second', value=2, variable=selected)

rad3 = Radiobutton(window,text='Third', value=3, variable=selected)

def clicked():

   print(selected.get())

btn = Button(window, text="Click Me", command=clicked)

rad1.grid(column=0, row=0)

rad2.grid(column=1, row=0)

rad3.grid(column=2, row=0)

btn.grid(column=3, row=0)

window.mainloop()



ser = serial.Serial('/dev/cu.usbmodem141203', 19200, timeout = 0)
serial_data = ser.read(60);print (serial_data)
ser.write (b'\r');sleep (.05);ser.write (b'portSetup 1 1 1\r')
serial_data = ser.read(60);print (serial_data)
ser.write (b'\r');sleep (.05);ser.write (b'portSetup 1 0 2\r')
serial_data = ser.read(60);print (serial_data)
for i in range (0,10):
    ser.write (b'\r');sleep (.05);ser.write (b'writeBits 1 2 1\r')
    ser.write (b'\r');sleep (.05);ser.write (b'readBits 1 2\r')
    serial_data = ser.read(60)
    print (serial_data)
    sleep (.5)


