from matplotlib import pyplot as plt
from Figure_details1 import plot, FigDetails as fdc

# objects
titles, subtitles, xlabels, ylabels, xlims, ylims, legends = [fdc.titles, fdc.subtitles, fdc.xlabels, fdc.ylabels,
                                                              fdc.xlims, fdc.ylims, fdc.legends_dict]


import math as m
import numpy as np

class SpVisual():
    def __init__(self):
        pass

    def plot2d_old(self, err_data_pitch, err_data_yaw):
        # Markers
        markers = ["s", "*", ".", "+", "D", "o"]

        #######################################
        # plotting
        #######################################

        # plotting first data
        plot(err_data_pitch, subtitle=subtitles[-1], title=titles["Fig_1"][-1], xlabel=xlabels[0], ylabel=ylabels[0],
             legends=legends["Fig_1"][0],
             xlim=None, ylim=None, leg_loc="upper left", ls=None, lw=2, c=None, marker=".", mec="k", mfc="b", ms=8)
        # plt.grid(axis="both", linewidth=1, linestyle="dashed")

        # plotting second data
        plot(err_data_yaw, subtitle=subtitles[-1], title=titles["Fig_1"][-1], xlabel=xlabels[0], ylabel=ylabels[0],
             legends=legends["Fig_1"][0],
             xlim=None, ylim=None, leg_loc="upper left", ls=None, lw=2, c=None, marker=".", mec="k", mfc="r", ms=8)
        plt.grid(axis="both", linewidth=1, linestyle="dashed")
        plt.show()

    # Ploting....
    def plot(self, y, x=None, title=None, subtitle=None, xlabel=None, ylabel=None, legends=None, xlim=None, ylim=None,
             leg_loc="upper left", ls=None, lw=None, c=None, marker=None, mec=None, mfc=None, ms=None):
        # Note:   the 'DejaVu Sans' is the default font family, defalt font size for python is 10
        font_family = [None, "Helvetica", "Times New Roman", "Comic Sans MS", "Arial", "Calibri", "DejaVu Sans"]

        if x is None:
            plt.plot(y, label=legends, ls=ls, lw=lw, c=c, marker=marker, mec=mec, mfc=mfc, ms=ms)
        else:
            plt.plot(x, y, label=legends, ls=ls, lw=lw, c=c, marker=marker, mec=mec, mfc=mfc, ms=ms)
        plt.suptitle(subtitle, ha='center', fontsize=12, weight='extra bold', family=font_family[-1])
        plt.title(title, ha='center', fontsize=12, weight='normal', family=font_family[-1])
        plt.xlabel(xlabel, family=font_family[-1], fontsize=10, weight='normal')
        plt.ylabel(ylabel, family=font_family[-1], fontsize=10, weight='normal')
        plt.legend(loc=leg_loc, framealpha=1, facecolor="White", edgecolor="black", fancybox=True, shadow=False,
                   prop={'family': font_family[-1], 'stretch': 'normal', "weight": "normal", "size": 10})
        # help "axis= 'x' or 'y' or 'both'"  "which= 'major' or 'minor' "width= tick thikness""
        # direction -  in/out/inout ,  top/bottom/left/right -  True/false

        # Create a plot with major ticks every 2 units and minor ticks every 0.5 units
        # plt.set_xticks(np.arange(0, 11, 2))
        # plt.set_xticks(np.arange(0, 11, 0.5), minor=True)
        # plt.set_yticks(np.arange(-1, 1.1, 0.5))
        # plt.set_yticks(np.arange(-1, 1.1, 0.1), minor=True)
        # plt.minorticks_on()
        # plt.xticks(np.arange(0, 70, 20))
        # plt.yticks(np.arange(-100, 150, 30))

        # plt.minorticks_off
        # plt.tick_params(axis='x', which='major', labelsize=30, width=6, direction="in", top=True)
        # plt.tick_params(axis='x', which='minor', labelsize=10, width=1, direction="out", top=True)
        #
        # plt.tick_params(axis='y', which='major', labelsize=20, width=3, direction="out", right=True)
        # plt.tick_params(axis='y', which='minor', labelsize=20, width=1, direction="inout", right=True)
        # plt.tick_params(axis='both', which='both', labelsize=20, width=6)
        plt.xlim(xlim)
        plt.ylim(ylim)

        # plt.grid(axis="both", linewidth=1, linestyle="dashed")

    def plot3(self, x, y, z, title=None, subtitle=None, xlabel=None, ylabel=None, legends=None, xlim=None, ylim=None, zlim=None,
             leg_loc="upper left", ls=None, lw=None, c=None, marker=None, mec=None, mfc=None, ms=None, fig = plt.Figure()):
        # Note:   the 'DejaVu Sans' is the default font family, defalt font size for python is 10
        font_family = [None, "Helvetica", "Times New Roman", "Comic Sans MS", "Arial", "Calibri", "DejaVu Sans"]

        ax = fig.add_subplot(projection='3d')
        # # Add an axes
        # ax = fig.add_subplot(projection='3d')
        ax.plot(x, y, z, label=legends, ls=ls, lw=lw, c=c, marker=marker, mec=mec, mfc=mfc, ms=ms)
        ax.set_zlim(zlim)
        plt.suptitle(subtitle, ha='center', fontsize=12, weight='extra bold', family=font_family[-1])
        plt.title(title, ha='center', fontsize=12, weight='normal', family=font_family[-1])
        plt.xlabel(xlabel, family=font_family[-1], fontsize=10, weight='normal')
        plt.ylabel(ylabel, family=font_family[-1], fontsize=10, weight='normal')
        # plt.legend(loc=leg_loc, framealpha=1, facecolor="White", edgecolor="black", fancybox=True, shadow=False,
        #            prop={'family': font_family[-1], 'stretch': 'normal', "weight": "normal", "size": 10})
        # help "axis= 'x' or 'y' or 'both'"  "which= 'major' or 'minor' "width= tick thikness""
        # direction -  in/out/inout ,  top/bottom/left/right -  True/false

        # Create a plot with major ticks every 2 units and minor ticks every 0.5 units
        # plt.set_xticks(np.arange(0, 11, 2))
        # plt.set_xticks(np.arange(0, 11, 0.5), minor=True)
        # plt.set_yticks(np.arange(-1, 1.1, 0.5))
        # plt.set_yticks(np.arange(-1, 1.1, 0.1), minor=True)
        # plt.minorticks_on()
        # plt.xticks(np.arange(0, 70, 20))
        # plt.yticks(np.arange(-100, 150, 30))

        # plt.minorticks_off
        # plt.tick_params(axis='x', which='major', labelsize=30, width=6, direction="in", top=True)
        # plt.tick_params(axis='x', which='minor', labelsize=10, width=1, direction="out", top=True)
        #
        # plt.tick_params(axis='y', which='major', labelsize=20, width=3, direction="out", right=True)
        # plt.tick_params(axis='y', which='minor', labelsize=20, width=1, direction="inout", right=True)
        # plt.tick_params(axis='both', which='both', labelsize=20, width=6)
        # plt.xlim(xlim)
        # plt.ylim(ylim)


        # plt.grid(axis="both", linewidth=1, linestyle="dashed")
        # Ensure that the next plot doesn't overwrite the first plot
        ax.hold = True