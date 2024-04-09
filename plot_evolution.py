#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
# plt.rcParams.update({'font.size': 9.1})

#
# Customize the following parameters...
#


def main():

    filename = "mylog.txt"
    print("Loading: {}".format(filename))
    dataframe = pd.read_csv(filename, " ")
    print(dataframe.info())

    # Plot the evolution of the fitness.
    fig, axes = plt.subplots(1, 1, figsize=(10, 5))
    dataframe.plot(ax=axes, x='time', y='fitness', label='fitness', linewidth=0.5)
    axes.title.set_text("Fitness evolution")
    axes.set_xlabel("Time")

    plt.show()
    filename = "{}.pdf".format("evolution.pdf")
    fig.savefig(filename, bbox_inches='tight')

main()
