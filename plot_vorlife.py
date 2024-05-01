#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 15})

# Customize the following parameters...
BASE_DIR = "data/vorlife_true"
TRIALS = 30

def main():
    dataframes = []

    for trial in range(TRIALS):
        filename = f"{BASE_DIR}/stats_{trial}.dat"
        print("Loading: {}".format(filename))
        df = pd.read_csv(filename, sep=' ')
        df.columns = ['stepCount', 'eval', 'cumEval', 'robotRobotCollisions']
        print(df.info())

        dataframes.append(df)

    fig, axes = plt.subplots(2, 1, figsize=(10, 5))
    for trial, df in enumerate(dataframes):
        df.plot(ax=axes[0], x='stepCount', y='eval', linewidth=1)
        df.plot(ax=axes[1], x='stepCount', y='robotRobotCollisions', linewidth=1)

    # Compute and plot the average dataframe
    avg_df = pd.concat(dataframes).groupby(level=0).mean()
    avg_df.plot(ax=axes[0], x='stepCount', y='eval', linewidth=5)
    avg_df.plot(ax=axes[1], x='stepCount', y='robotRobotCollisions', linewidth=5)

    axes[0].set_xlabel('Step Count')
    axes[1].set_xlabel('Step Count')
    axes[0].set_ylabel('SSD')
    axes[1].set_ylabel('Robot-Robot Collisions')
    
    # Disable the legend for all plots
    for ax in axes:
        ax.get_legend().remove()

    plt.show()

    filename = f"{BASE_DIR}/plot.pdf"
    print(f"Saving {filename}")
    fig.savefig(filename, bbox_inches='tight')

main()