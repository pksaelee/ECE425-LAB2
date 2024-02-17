import serial
import time
import tkinter as tk

window = tk.Tk()
arduino = serial.Serial(port='COM6',baudrate=9600,timeout=0.1)

def write_read(x):
    arduino.write(x.encode())
    time.sleep(0.5)
    data = arduino.readline()
    return data

## MAPPING
## list to trace value for mapping
entries = []

def enter(event):
    to_arduino = ""
    for entry in entries:
        x = entry.get()
        to_arduino += x
        to_arduino += " "
    write_read(to_arduino)
##    print(to_arduino)
    
frame1 = tk.LabelFrame(window)
frame1.grid(row=0,column=0)
for i in range(4):
    for j in range(4):
        entry = tk.Entry(frame1, width=10, justify='center')
        entry.grid(row=i,column=j,padx=5,pady=5)
        entries.append(entry)
        
window.bind('<Return>',enter)

## POSITIONING
## list to track value for position
entries_pos = []
entries_start = []
entries_end = []

def start():
    to_arduino = ""
    for pos in entries_pos:
        x = pos.get()
        to_arduino += x
        to_arduino += " "
##        print(pos.get())
    for start in entries_start:
        x = start.get()
        to_arduino += x
        to_arduino += " "
##        print(start.get())
    for end in entries_end:
        x = end.get()
        to_arduino += x
        to_arduino += " "
##        print(end.get())
    write_read(to_arduino)

frame2 = tk.LabelFrame(window,text="Positioning:")
frame2.grid(row=0,column=1)
labelx = tk.Label(frame2,text="X:").grid(row=0,column=0)
labely = tk.Label(frame2,text="Y:").grid(row=0,column=1)
labelt = tk.Label(frame2,text="Theta:").grid(row=0,column=2)
for i in range(3):
    pos = tk.Entry(frame2, justify="right")
    pos.grid(row=1,column=i,padx=5,pady=5)
    entries_pos.append(pos)

lb_start = tk.Label(frame2,text="Path Planning").grid(row=2,column=1)
lb_end = tk.Label(frame2,text="to").grid(row=4,column=1)
for i in range(2):
    begin = tk.Entry(frame2, justify="right")
    begin.grid(row=3,column=i,padx=5,pady=5)
    entries_start.append(begin)
    
    end = tk.Entry(frame2, justify="right")
    end.grid(row=5,column=i,padx=5,pady=5)
    entries_end.append(end)

start = tk.Button(frame2,text="Start!",command=start)
start.grid(row=6,column=1)

## CONTROLLER
def pivotL():
    write_read("PL")
##        print("Pivot Left")
def forward():
    write_read("F")
##        print("Forward")
def pivotR():
    write_read("PR")
##        print("Pivot Right")
def spinL():
    write_read("SL")
##        print("Spin Left")
def spinR():
    write_read("SR")
##        print("Spin Right")
def backwards():
    write_read("B")
##        print("Backwards")

frame3 = tk.LabelFrame(window, text="Controller")
frame3.grid(row=1,column=0)
pivotl = tk.Button(frame3,text="Pivot Left",command=pivotL,height=3,width=10).grid(row=0,column=0,padx=5,pady=5)
pivotr = tk.Button(frame3,text="Pivot Right",command=pivotR,height=3,width=10).grid(row=0,column=1,padx=5,pady=5)
forw = tk.Button(frame3,text="Forward",command=forward,height=3,width=10).grid(row=0,column=2,padx=5,pady=5)
back = tk.Button(frame3,text="Backwards",command=backwards,height=3,width=10).grid(row=1,column=0,padx=5,pady=5)
spinl = tk.Button(frame3,text="Spin Left",command=spinL,height=3,width=10).grid(row=1,column=2,padx=5,pady=5)
spinr = tk.Button(frame3,text="Spin Right",command=spinR,height=3,width=10).grid(row=2,column=1,padx=5,pady=5)

## SENSORS
frame4 = tk.LabelFrame(window,text="Sensors")
frame4.grid(row=1,column=1)

def sense():
    to_arduino = ""
    for sensor in entries_sens:
        x = sensor.get()
        to_arduino += x
        to_arduino += " "
##        print(sensor.get())
    write_read(to_arduino)

## list to tract value for sensors
entries_sens = []

for i in range(3):
    for j in range(3):
        if((i == 1 and j == 1) or (i == 2 and j == 0) or (i == 2 and j == 2)):
            continue
        sensor = tk.Entry(frame4, width=10)
        sensor.grid(row=i,column=j,padx=5,pady=5)
        entries_sens.append(sensor)

run = tk.Button(frame4,text="Start!",command=sense)
run.grid(row=3,column=1)

## COMMAND
def run():
    print(comm.get())

frame5 = tk.LabelFrame(window,text="Command")
frame5.grid(row=2,column=0)
comm = tk.Entry(frame5)
comm.pack()
button = tk.Button(frame5,text="Run",command=run)
button.pack()

window.mainloop()
 
##
##while True:
##    num = input("Enter number: ")
##    value = write_read(num)
##    print(value)


