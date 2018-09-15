'''
Created on 15 Sep. 2018

@author: Denver
'''

'''
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,2,1)
ax2 = fig.add_subplot(1,2,2)

axes = [ax1,ax2]

def animate(i):
    graph_data = open('example.txt','r').read()
    lines = graph_data.split('\n')
    for axis in axes:
        xs = []
        ys = []
        for line in lines:
            if len(line) > 1:
                x, y = line.split(',')
                xs.append(x)
                ys.append(y)
        axis.clear()
        axis.plot(xs, ys)
    
ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
'''



import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# Parameters
x_len = 200         # Number of points to display
y_range = [0, 40]  # Range of possible Y values to display

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = list(range(0, 200))
ys = [0] * x_len
ax.set_ylim(y_range)

# Create a blank line. We will update the line in animate
line, = ax.plot(xs, ys)

# Add labels
plt.title('TMP102 Temperature over Time')
plt.xlabel('Samples')
plt.ylabel('Temperature (deg C)')

original_time = time.time()

# This function is called periodically from FuncAnimation
def animate(i, ys):

    # Read temperature (Celsius) from TMP102
    time_since = time.time() - original_time
    temp_c = round(time_since, 2)

    # Add y to list
    ys.append(temp_c)

    # Limit y list to set number of items
    ys = ys[-x_len:]

    # Update line with new Y values
    line.set_ydata(ys)

    return line,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
    animate,
    fargs=(ys,),
    interval=50,
    blit=True)
plt.show()
