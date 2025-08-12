from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
import numpy as np
import serial as sr

comPort = 'COM3'
baudrate = 1500000
refreshrate = 50
MAX_BUFFER_SIZE = 12000

# Global variables - separate buffers for each axis
pitch_data = np.zeros(MAX_BUFFER_SIZE)
roll_data = np.zeros(MAX_BUFFER_SIZE)
yaw_data = np.zeros(MAX_BUFFER_SIZE)
cond = False
background = None

# Create main window
root = tk.Tk()
root.title('Attitude Plotter')
root.configure(background='light blue')
root.geometry("1400x788")

# Create figure with three subplots
fig = Figure(figsize=(12, 8), dpi=100)  # Increased height for three plots
ax1 = fig.add_subplot(311)  # Pitch
ax2 = fig.add_subplot(312)  # Roll
ax3 = fig.add_subplot(313)  # Yaw

# Configure pitch plot
ax1.set_title('Pitch')
ax1.set_ylabel('Degrees [°]')
ax1.set_xlim(0, MAX_BUFFER_SIZE)
ax1.set_ylim(-40, 40)
pitch_line, = ax1.plot([], [], 'b-', lw=1, label='Pitch')

# Configure roll plot
ax2.set_title('Roll')
ax2.set_ylabel('Degrees [°]')
ax2.set_xlim(0, MAX_BUFFER_SIZE)
ax2.set_ylim(-40, 40)
roll_line, = ax2.plot([], [], 'r-', lw=1, label='Roll')

# Configure yaw plot
ax3.set_title('Yaw')
ax3.set_xlabel('Samples')
ax3.set_ylabel('Degrees [°]')
ax3.set_xlim(0, MAX_BUFFER_SIZE)
ax3.set_ylim(-180, 180)  # Wider range for yaw
yaw_line, = ax3.plot([], [], 'g-', lw=1, label='Yaw')

# Add legends
ax1.legend(loc='upper right')
ax2.legend(loc='upper right')
ax3.legend(loc='upper right')

# Layout adjustments
fig.tight_layout()
fig.subplots_adjust(hspace=0.4)  # Space between subplots

# Place canvas
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.place(x=10, y=10, width=1200, height=700)  # Increased height
canvas.draw()

# Store backgrounds for blitting
backgrounds = [
    canvas.copy_from_bbox(ax1.bbox),
    canvas.copy_from_bbox(ax2.bbox),
    canvas.copy_from_bbox(ax3.bbox)
]

# Serial setup
s = sr.Serial(comPort, baudrate)
s.reset_input_buffer()

def plot_data():
    global cond, pitch_data, roll_data, yaw_data, backgrounds
    
    if cond:
        new_data_count = 0
        
        # Process all available lines
        while s.in_waiting:
            try:
                raw_line = s.readline()
                a = raw_line.decode('utf-8').strip()
                
                # Parse attitude data
                axis_names = ['pitch', 'roll', 'yaw']
                attitude = {}
                valid_data = True
                
                for axis in axis_names:
                    idx = a.find(axis)
                    if idx != -1:
                        # Extract value between colon and degree symbol
                        start_idx = a.find(':', idx) + 1
                        end_idx = a.find('°', start_idx)
                        if start_idx != -1 and end_idx != -1:
                            try:
                                value = float(a[start_idx:end_idx].strip())
                                attitude[axis] = value
                            except ValueError:
                                valid_data = False
                                break
                        else:
                            valid_data = False
                            break
                    else:
                        valid_data = False
                        break
                
                if valid_data and len(attitude) == 3:
                    # Update data buffers
                    pitch_data = np.roll(pitch_data, -1)
                    roll_data = np.roll(roll_data, -1)
                    yaw_data = np.roll(yaw_data, -1)
                    
                    pitch_data[-1] = attitude['pitch']
                    roll_data[-1] = attitude['roll']
                    yaw_data[-1] = attitude['yaw']
                    
                    new_data_count += 1
                    
            except (UnicodeDecodeError, ValueError, IndexError):
                continue
        
        # Only update plots if we have new data
        if new_data_count > 0:
            # Update pitch plot
            canvas.restore_region(backgrounds[0])
            pitch_line.set_data(np.arange(MAX_BUFFER_SIZE), pitch_data)
            ax1.draw_artist(pitch_line)
            canvas.blit(ax1.bbox)
            
            # Update roll plot
            canvas.restore_region(backgrounds[1])
            roll_line.set_data(np.arange(MAX_BUFFER_SIZE), roll_data)
            ax2.draw_artist(roll_line)
            canvas.blit(ax2.bbox)
            
            # Update yaw plot
            canvas.restore_region(backgrounds[2])
            yaw_line.set_data(np.arange(MAX_BUFFER_SIZE), yaw_data)
            ax3.draw_artist(yaw_line)
            canvas.blit(ax3.bbox)
    
    root.after(20, plot_data)

def plot_start():
    global cond
    cond = True
    s.reset_input_buffer()

def plot_stop():
    global cond
    cond = False

# Control buttons
btn_frame = tk.Frame(root, bg='light blue')
btn_frame.place(x=100, y=720)

start = tk.Button(btn_frame, text="Start", font=('calibri', 12), 
                  width=8, command=plot_start)
start.pack(side=tk.LEFT, padx=5)

stop = tk.Button(btn_frame, text="Stop", font=('calibri', 12), 
                 width=8, command=plot_stop)
stop.pack(side=tk.LEFT, padx=5)

root.after(1, plot_data)
root.mainloop()