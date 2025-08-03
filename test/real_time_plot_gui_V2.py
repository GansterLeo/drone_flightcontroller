from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
import numpy as np
import serial as sr

comPort = 'COM3'
MAX_BUFFER_SIZE = 4000

# Global variables
data = np.zeros(MAX_BUFFER_SIZE)  # Pre-allocated circular buffer
cond = False
background = None  # For blitting

# Create main window
root = tk.Tk()
root.title('Real Time Plot')
root.configure(background='light blue')
root.geometry("1400x788")

# Create figure with larger plot area
fig = Figure(figsize=(12, 6), dpi=100)  # Larger figure size
ax = fig.add_subplot(111)
ax.set_title('Attitude')
ax.set_xlabel('Samples')
ax.set_ylabel('Degrees [°]')
ax.set_xlim(0, MAX_BUFFER_SIZE)
ax.set_ylim(-40, 40)
line, = ax.plot([], [], lw=1)

# Add after initial setup
fig.tight_layout()
fig.subplots_adjust(left=0.05, right=0.98, bottom=0.08, top=0.95)

# Place larger canvas (1200x600)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.place(x=10, y=10, width=1200, height=600)
canvas.draw()

# Store background for blitting
background = canvas.copy_from_bbox(ax.bbox)

# Serial setup
s = sr.Serial(comPort, 250000)
s.reset_input_buffer()

def plot_data():
    global cond, data, background
    
    if cond:
        # Process all available lines (non-blocking)
        while s.in_waiting:
            
            try:
                a = str(s.readline(), 'utf-8').strip()
                # print(a, end='')
                # Direct float conversion (if format is consistent)
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
                    print(attitude)
                    data = np.roll(data, -1)
                    data[-1] = attitude['pitch']
                   
            except (UnicodeDecodeError, ValueError, IndexError):
                continue
        
        # Update plot with blitting
        canvas.restore_region(background)
        line.set_data(np.arange(len(data)), data)
        ax.draw_artist(line)
        canvas.blit(ax.bbox)
    
    root.after(1, plot_data)

def plot_start():
    global cond
    cond = True
    s.reset_input_buffer()

def plot_stop():
    global cond
    cond = False

# Control buttons
start = tk.Button(root, text="Start", font=('calibri', 12), command=plot_start)
start.place(x=100, y=620)

stop = tk.Button(root, text="Stop", font=('calibri', 12), command=plot_stop)
stop.place(x=start.winfo_x() + start.winfo_reqwidth() + 20, y=620)

root.after(1, plot_data)
root.mainloop()