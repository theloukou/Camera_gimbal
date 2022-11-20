import serial, time, json, sys
from tkinter import *
from tkinter import messagebox as msgbx
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib import style
style.use('ggplot')

DEBUG=True
graphActive=True

fig = Figure(figsize = (8, 5), dpi = 100)
pGraph = fig.add_subplot(211)
rGraph = fig.add_subplot(212)
tList=[]
piList=[]
poList=[]
riList=[]
roList=[]
line=""

gimbal=serial.Serial()
gimbal.baudrate=115200
gimbal.timeout=1

def on_closing():
    gimbal.close()
    form.destroy()

def isJson(myjson):
  try:
    jsonObject = json.loads(myjson)
  except ValueError as e:
    return False
  return True

def getPortList():
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    portList=[]
    for x in range(len(ports)):
        portList.append(ports[x][0])
    print("Got ports:")
    print(portList)
    if len(portList)==0:
        portList.append("No ports!")
    return portList
    gimbal.close()

def openSerial():
    global gimbal
    gimbal.close()
    port=portValue.get()
    if str(port)=="Select port":
        msgbx.showwarning(title="Oops!", message="Please select a port!")
    elif str(port)=="No ports!":
        portList = getPortList()
        portValue.set("Select port")
    else:
        gimbal.port=str(port)
        print("Opening serial port %s..." % gimbal.port)
        gimbal.open()

def animate(i):
    pGraph.clear()
    pGraph.plot(tList, piList, 'b', label="pIn", antialiased=False)
    pGraph.plot(tList, poList, 'm', label="pOut", antialiased=False)
    pGraph.axis(ymin=-25,ymax=25)
    pGraph.legend()
    rGraph.clear()
    rGraph.plot(tList, riList, 'c', label="rIn", antialiased=False)
    rGraph.plot(tList, roList, 'y', label="rOut", antialiased=False)
    rGraph.axis(ymin=-25,ymax=25)
    rGraph.legend()

def toggleGraph():
    global graphActive
    if graphActive: graphActive=False
    else: graphActive=True

def sendParams():
    if gimbal.isOpen():
        serBuf=str.encode("S%.2f %.2f %.2f %.2f %.2f %.2f 80E" % (float(pKp.get()), float(pKi.get()), float(pKd.get()),
                                                              float(rKp.get()), float(rKi.get()), float(rKd.get())))
        if DEBUG: print(serBuf)
        gimbal.write(serBuf)
    else:
        msgbx.showwarning(title="Not connected!", message="Please connect to gimbal first.")
    
def sendSetpoints():
    if gimbal.isOpen():
        if len(rSetpoint.get())==0:
            msgbx.showwarning(title="Entry box empty!", message="Please give setpoint value!")
        else:
            serBuf=str.encode("SET%+04d %+04d" % (float(pSetpoint.get()), float(rSetpoint.get())))
            if DEBUG: print(serBuf)
            gimbal.write(serBuf)
    else:
        msgbx.showwarning(title="Not connected!", message="")

form = Tk()
form.title("Gimbal Utility")
form.protocol("WM_DELETE_WINDOW", on_closing)

f_controls = Frame(form)
f_controls.grid(row=1, column=1, columnspan=3)
portList = getPortList()
portValue = StringVar(form)
portValue.set("Select port")
portMenu = OptionMenu(f_controls, portValue, *portList)
portMenu.configure(width=10)
portMenu.grid(row=1, column=1, columnspan=1)
b_openSer = Button(f_controls, text='Open port', command=openSerial)
b_openSer.grid(row=1, column=2, padx=(0,20))
b_pid = Button(f_controls, text='Update PID', command=sendParams)
b_pid.grid(row=1, column=3)
b_setpoints = Button(f_controls, text='Update setpoints', command=sendSetpoints)
b_setpoints.grid(row=1, column=4)
b_graph = Button(f_controls, text='Toggle graph', command=toggleGraph)
b_graph.grid(row=1, column=5, padx=(20,0))

f_pitch = Frame(form, bd=1, relief=SUNKEN)
f_pitch.grid(row=2, column=1)
l_pitch = Label(f_pitch, text="Pitch:")
l_pitch.grid(row=1, column=1, rowspan=2)
l_pKp = Label(f_pitch, text="Kp")
l_pKp.grid(row=1, column=2)
pKp = StringVar(form)
t_pKp = Entry(f_pitch, textvariable=pKp, width=6, justify='center')
t_pKp.grid(row=2, column=2)
l_pKi = Label(f_pitch, text="Ki")
l_pKi.grid(row=1, column=3)
pKi = StringVar(form)
t_pKi = Entry(f_pitch, textvariable=pKi, width=6, justify='center')
t_pKi.grid(row=2, column=3)
l_pKd = Label(f_pitch, text="Kd")
l_pKd.grid(row=1, column=4)
pKd = StringVar(form)
t_pKd = Entry(f_pitch, textvariable=pKd, width=6, justify='center')
t_pKd.grid(row=2, column=4)
l_pSetpoint = Label(f_pitch, text="Setpoint")
l_pSetpoint.grid(row=1, column=5)
pSetpoint = StringVar(form)
pSetpoint.set(0)
t_pSetpoint = Entry(f_pitch, textvariable=pSetpoint, width=6, justify='center')
t_pSetpoint.grid(row=2, column=5)

f_roll = Frame(form, bd=1, relief=SUNKEN)
f_roll.grid(row=2, column=2)
l_roll = Label(f_roll, text="Roll:")
l_roll.grid(row=1, column=1, rowspan=2)
l_rKp = Label(f_roll, text="Kp")
l_rKp.grid(row=1, column=2)
rKp = StringVar(form)
t_rKp = Entry(f_roll, textvariable=rKp, width=6, justify='center')
t_rKp.grid(row=2, column=2)
l_rKi = Label(f_roll, text="Ki")
l_rKi.grid(row=1, column=3)
rKi = StringVar(form)
t_rKi = Entry(f_roll, textvariable=rKi, width=6, justify='center')
t_rKi.grid(row=2, column=3)
l_rKd = Label(f_roll, text="Kd")
l_rKd.grid(row=1, column=4)
rKd = StringVar(form)
t_rKd = Entry(f_roll, textvariable=rKd, width=6, justify='center')
t_rKd.grid(row=2, column=4)
l_rSetpoint = Label(f_roll, text="Setpoint")
l_rSetpoint.grid(row=1, column=5)
rSetpoint = StringVar(form)
rSetpoint.set(0)
t_rSetpoint = Entry(f_roll, textvariable=rSetpoint, width=6, justify='center')
t_rSetpoint.grid(row=2, column=5)

f_graph = Frame(form, bd=1, relief=SUNKEN)
f_graph.grid(row=3, column=1, columnspan=2)
canvas = FigureCanvasTkAgg(fig, master = f_graph)  
canvas.draw()
canvas.get_tk_widget().pack()
ani = animation.FuncAnimation(fig, animate, interval=100)

statusbar = Label(form, text="Connect gimbal and select port", bd=1, relief=SUNKEN)
statusbar.grid(column=1, row=99, columnspan=2, sticky="nsew")

while line!="Starting\r\n":
    try:
        line=gimbal.readline().decode('utf-8')
        print(line)
    except:
        gimbal.close()
    finally:
        form.update_idletasks()
        form.update()
            
jsonRaw=gimbal.readline().decode('utf-8')
if DEBUG: print(jsonRaw)
jsonData=json.loads(jsonRaw)
vBat=jsonData["vbat"]/1000
vBatPrc=int(((vBat-9)*100)/3.6)
statusbar.configure(text="Battery voltage: "+str(vBat)+" Volts ("+str(vBatPrc)+"%)")

jsonRaw=gimbal.readline().decode('utf-8')
if DEBUG: print(jsonRaw)
if isJson(jsonRaw):
    jsonData=json.loads(jsonRaw)
    pKp.set(jsonData["pKp"])
    pKi.set(jsonData["pKi"])
    pKd.set(jsonData["pKd"])
    rKp.set(jsonData["rKp"])
    rKi.set(jsonData["rKi"])
    rKd.set(jsonData["rKd"])

tVal=0
while True:
    try:
        jsonRaw=gimbal.readline().decode('utf-8')
        if DEBUG: print(jsonRaw)
        if isJson(jsonRaw):
            jsonData=json.loads(jsonRaw)
            if "vbat" in jsonData:
                vBat=jsonData["vbat"]/1000
                vBatPrc=int(((vBat-9)*100)/3.6)
                statusbar.configure(text="Battery voltage: "+str(vBat)+" Volts ("+str(vBatPrc)+"%)")
            elif graphActive:
                tVal+=0.01; tList.append(tVal)
                pi=jsonData["pi"]; piList.append(pi)
                po=jsonData["po"]; poList.append(po)
                ri=jsonData["ri"]; riList.append(ri)
                ro=jsonData["ro"]; roList.append(ro)
                if len(tList)>200:
                    del tList[0]; del piList[0]; del poList[0]; del riList[0]; del roList[0]
    except:
        gimbal.close()
    finally:
        form.update_idletasks()
        form.update()