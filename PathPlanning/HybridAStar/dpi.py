import matplotlib.pyplot as plt
import numpy as np

def export_figure_matplotlib(arr, f_name, dpi=200, resize_fact=1, plt_show=False):
    """
    Export array as figure in original resolution
    :param arr: array of image to save in original resolution
    :param f_name: name of file where to save figure
    :param resize_fact: resize facter wrt shape of arr, in (0, np.infty)
    :param dpi: dpi of your screen
    :param plt_show: show plot or not
    """
    fig = plt.figure(frameon=False)
    fig.set_size_inches(arr.shape[1]/dpi, arr.shape[0]/dpi)
    ax = plt.Axes(fig, [0., 0., 1., 1.])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.imshow(arr)
    plt.savefig(f_name, dpi=(dpi * resize_fact))
    if plt_show:
        plt.show()
    else:
        plt.close()

from matplotlib import pyplot as plt

# Set the limits of the plot

# plt.autoscale(False)
# plt.axis("square")
plt.axis("equal")
# plt.xlim(-0.5, 0.5)
# plt.ylim(-0.5, 0.5)
plt.ylim(-2, 2)
plt.xlim(-2, 2)

# Don't mess with the limits!

# Plot anything you want
plt.plot([0, 1])
plt.show()