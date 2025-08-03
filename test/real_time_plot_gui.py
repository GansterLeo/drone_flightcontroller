from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
import numpy as np
import serial as sr

comPort = 'COM3'

#------global variables
data = np.array([])
cond = False

#-----plot data-----
def plot_data():
    global cond, data
    
    if (cond == True):
        
        a = str(s.readline(), 'utf-8')
        print(a, end='')

        axisNames = ['pitch', 'roll', 'yaw']
        attitude = {'pitch': 5.0, 'roll': 0.0, 'yaw': 0.0}
        isAttitude = True

        for i in range(3):
            idx = a.find(axisNames[i])
            if(idx != -1):
                valueStartIdx   = idx + np.char.str_len(axisNames[i]) + 2
                valueEndIdx     = a.find('°', idx) - 1
                attitude[axisNames[i]] = float(a[valueStartIdx:valueEndIdx])
            else:
                isAttitude = False
        if isAttitude:
            if(len(data) < 100):
                data = np.append(data,attitude['pitch'])
            else:
                data[0:99] = data[1:100]
                data[99] = attitude['pitch']

            lines.set_xdata(np.arange(0,len(data)))
            lines.set_ydata(data)

            canvas.draw()
    
    root.after(1,plot_data)

def plot_start():
    global cond
    cond = True
    s.reset_input_buffer()

def plot_stop():
    global cond
    cond = False
    

#-----Main GUI code-----
root = tk.Tk()
root.title('Real Time Plot')
root.configure(background = 'light blue')
root.geometry("1400x788") # set the window size

#------create Plot object on GUI----------
# add figure canvas
fig = Figure()
ax = fig.add_subplot(111)

#ax = plt.axes(xlim=(0,100),ylim=(0, 120)); #displaying only 100 samples
ax.set_title('Attitude')
ax.set_xlabel('pitch')
ax.set_ylabel('Degrees [°]')
ax.set_xlim(0,100)
ax.set_ylim(-50,50)
lines = ax.plot([],[])[0]

canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.get_tk_widget().place(x = 10,y=10, width = 500,height = 400)
canvas.draw()

#----------create button---------
root.update()
start = tk.Button(root, text = "Start", font = ('calbiri',12),command = lambda: plot_start())
start.place(x = 100, y = 450 )

root.update()
stop = tk.Button(root, text = "Stop", font = ('calbiri',12), command = lambda:plot_stop())
stop.place(x = start.winfo_x()+start.winfo_reqwidth() + 20, y = 450)

#----start serial port----
s = sr.Serial(comPort,250000)
s.reset_input_buffer()



root.after(1,plot_data)
root.mainloop()