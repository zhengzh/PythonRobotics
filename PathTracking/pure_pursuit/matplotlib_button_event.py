import numpy as np
import matplotlib.pyplot as plt
import time

MAX_CLICK_LENGTH = 0.1 # in seconds; anything longer is a drag motion

# One way to distinguish between clicks and dragging/zooming 
# (be it right click or left click) would be to measure the time
# between the button press and the button release and 
# then carry out the actions on the button release, not the button press.
def onclick(event, ax):
    ax.time_onclick = time.time()

def onrelease(event, ax):
    # Only clicks inside this axis are valid.
    if event.inaxes == ax:
        if event.button == 1 and ((time.time() - ax.time_onclick) < MAX_CLICK_LENGTH):
            print(event.xdata, event.ydata)
            # Draw the click just made
            ax.scatter(event.xdata, event.ydata)
            ax.figure.canvas.draw()
        elif event.button == 2:
            print("scroll click")
        elif event.button == 3:
            print("right click")
        else:
            pass


fig, (ax1, ax2) = plt.subplots(1, 2)
# Plot some random scatter data
ax2.scatter(np.random.uniform(0., 10., 10), np.random.uniform(0., 10., 10))

fig.canvas.mpl_connect('button_press_event', lambda event: onclick(event, ax2))
fig.canvas.mpl_connect('button_release_event', lambda event: onrelease(event, ax2))
plt.show()