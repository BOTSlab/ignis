#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 15})

#
# Customize the following parameters...
#


def main():

    with open('last_name.dat', 'r') as file:
        base_name = file.read().replace('\n', '')
    print("Base name: {}".format(base_name))

    filename = "data/{}/optimize.dat".format(base_name)
    print("Loading: {}".format(filename))
    dataframe = pd.read_csv(filename, sep=',')
    print(dataframe.info())

    # Plot the evolution of the fitness.
    fig, axes = plt.subplots(1, 1, figsize=(10, 5))
    dataframe.plot(ax=axes, x='Gen', y='gbest', label='Best Fitness', linewidth=2)
    #dataframe.plot(ax=axes[1], x='Gen', y='Mean Vel.')
    dataframe.plot(ax=axes, x='Gen', y='Mean lbest', label='Avg. Local Best')
    #dataframe.plot(ax=axes[1], x='Gen', y='Avg. Dist.')
    axes.set_xlabel('Generation')
    axes.set_ylabel('Fitness')

    plt.show()
    pdfname = "data/{}/fitness.pdf".format(base_name)
    fig.savefig(pdfname, bbox_inches='tight')

main()
