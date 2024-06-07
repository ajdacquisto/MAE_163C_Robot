import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Replace 'COM3' with your actual serial port
ser = serial.Serial('COM3', 9600)

fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'r-')

def init():
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 1024)
    return ln,

def update(frame):
    line = ser.readline().decode('utf-8').strip()
    y = int(line)
    xdata.append(frame)
    ydata.append(y)
    if len(xdata) > 100:  # Keep only the latest 100 data points
        xdata.pop(0)
        ydata.pop(0)
    ln.set_data(xdata, ydata)
    ax.set_xlim(frame-100, frame)
    return ln,

ani = animation.FuncAnimation(fig, update, frames=range(100), init_func=init, blit=True, repeat=False)
plt.show()
