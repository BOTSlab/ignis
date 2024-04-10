#!/usr/bin/env python

"""
Plots the positions of pucks on top of the obstacles image for specific trials.
"""

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import asin, cos, pi, sin

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 9.1})

BASE_NAME = "./data"

ARENA_NAMES = ["pso_200_cubic_2_robots_5_pucks"]

ARENA_LONG_NAMES = {"pso_200_cubic_2_robots_5_pucks":"Cubic, 2 robots, 5 pucks"}

BEST_TRIAL = False
WORST_TRIAL = False
SPECIFIC_PERFORMANCE_TRIAL = False
SPECIFIC_PERFORMANCE = 0.1
SPECIFIC_TRIAL = True
SPECIFIC_TRIAL_NUMBER = 0

DRAW_GOAL = True
DRAW_PUCKS = True
DRAW_FINAL_ROBOTS = False
DRAW_HEATMAP_ROBOTS = True

N_ROBOTS = 2
N_PUCKS = 5
SAVE_FIG = True
START_TRIAL = 0
LAST_TRIAL = 0

ROBOT_RADIUS = 10
PUCK_RADIUS = 20

GOAL_X = 300
GOAL_Y = 300

def main():
    fig, axes = plt.subplots(1, len(ARENA_NAMES), sharex='col', squeeze=False)

    subplot_column = 0
    for arena_name in ARENA_NAMES:
        puckmap_for_arena(axes[0, subplot_column], arena_name)
        subplot_column += 1

    # Add headers for subplot columns (if more than one used).
    if len(ARENA_NAMES) > 1:
        for col in range(len(ARENA_NAMES)):
            axes[0, col].set_title(ARENA_LONG_NAMES[ARENA_NAMES[col]])

    plt.show()
    if SAVE_FIG:
        #filename = f"../performance_{SPECIFIC_PERFORMANCE}.pdf"
        filename = f"puckmap.pdf"
        print(f"Saving {filename}")
        fig.savefig(filename, bbox_inches='tight')

def puckmap_for_arena(axes, arena_name):
    print(f"puckmap_for_arena for arena: {arena_name}")

    chosen_puck_df = None
    chosen_robot_df = None
    chosen_score = None
    best_score = float('inf')
    worst_score = 0
    lowest_score_difference = float('inf')
    for trial in range(START_TRIAL, LAST_TRIAL + 1):
        
        robot_df = dataframe_per_trial('robotPose', arena_name, trial)
        puck_df = dataframe_per_trial('puckPosition', arena_name, trial)

        if SPECIFIC_TRIAL and trial == SPECIFIC_TRIAL_NUMBER:
            puckmap(axes, puck_df, robot_df, arena_name)

        stats_df = dataframe_per_trial('stats', arena_name, trial)

        # Get the score for the last row
        score = stats_df.at[ len(stats_df)-1, 1 ]
        if BEST_TRIAL and score < best_score:
            best_score = score
            chosen_puck_df = puck_df
            chosen_robot_df = robot_df
            chosen_score = score
            print("Best trial so far: {}".format(trial))

        if WORST_TRIAL and score > worst_score:
            worst_score = score
            chosen_puck_df = puck_df
            chosen_robot_df = robot_df
            chosen_score = score
            print("Worst trial so far: {}".format(trial))

        if SPECIFIC_PERFORMANCE_TRIAL and abs(score - SPECIFIC_PERFORMANCE) < lowest_score_difference:
            lowest_score_difference = abs(score - SPECIFIC_PERFORMANCE)
            chosen_puck_df = puck_df
            chosen_robot_df = robot_df
            chosen_score = score
            print("Closest trial so far: {}".format(trial))

    if BEST_TRIAL or WORST_TRIAL or SPECIFIC_PERFORMANCE_TRIAL:
        print(f"Score of chosen trial: {chosen_score}")
        puckmap(axes, chosen_puck_df, chosen_robot_df, arena_name)

def draw_robot(x, y, theta, axes, transparency):

    # Now fill in the circular section 
    delta_angle = 0.1
    start_angle = 0
    stop_angle = 2*pi
    angle = start_angle + delta_angle
    circle_points = []
    while angle < stop_angle:
        circle_points.append([x + ROBOT_RADIUS * cos(angle), y + ROBOT_RADIUS * sin(angle)])
        angle += delta_angle

    point_array = np.array(circle_points)
    filled_polygon = plt.Polygon(point_array, color=(1, 0, 0, transparency))
    axes.add_patch(filled_polygon)

    outline_polygon = plt.Polygon(point_array, color=(0, 0, 0, transparency), fill=False)
    axes.add_patch(outline_polygon)

    # Draw heading line
    radius = 0.9 * ROBOT_RADIUS
    line = plt.Line2D([x, x + radius * cos(theta)], [y, y + radius * sin(theta)], color=(1, 1, 1, transparency))
    axes.add_line(line)

def puckmap(axes, puck_df, robot_df, arena_name):
    print("puckmap for arena: {}".format(arena_name))
    '''
    filename = '../images/sim_stadium_one_wall/obstacles.png'.format(arena_name)

    obstacles = cv2.imread(filename)
    axes.imshow(obstacles, cmap='gray', interpolation='none')
    '''

    axes.set_xlim(0, 1000)
    axes.set_ylim(0, 600)
    axes.set_aspect('equal')
    axes.set_xticks([])
    axes.set_yticks([])

    if DRAW_GOAL:
        d = 20
        line1 = plt.Line2D([GOAL_X - d, GOAL_X + d], [GOAL_Y - d, GOAL_Y + d], color=(0, 1, 0))
        line2 = plt.Line2D([GOAL_X - d, GOAL_X + d], [GOAL_Y + d, GOAL_Y - d], color=(0, 1, 0))
        axes.add_line(line1)
        axes.add_line(line2)

    if DRAW_PUCKS:
        row = len(puck_df) - 1 # We're always interested in the final row.
        for i in range(N_PUCKS):
            col = 1 + 2*i
            x = puck_df.at[row, col]
            y = puck_df.at[row, col + 1]
            inner_circle = plt.Circle((x, y), PUCK_RADIUS, color='g', fill=True)
            circle = plt.Circle((x, y), PUCK_RADIUS, color='b', fill=False)
            axes.add_patch(inner_circle)
            axes.add_patch(circle)

    if DRAW_FINAL_ROBOTS:
        # Draw the final position of the robots
        row = len(robot_df) - 1 # We're always interested in the final row.
        for i in range(N_ROBOTS):
            col = 1 + 3*i
            x = robot_df.at[row, col]
            y = robot_df.at[row, col + 1]
            theta = robot_df.at[row, col + 2]
            draw_robot(x, y, theta, axes, 1)

    n_rows = len(robot_df)
    if DRAW_HEATMAP_ROBOTS:
        final_proportion_to_draw = 0.1
        start_row = int(n_rows * (1 - final_proportion_to_draw))
        for row in range(start_row, n_rows):
            print(f"Drawing row {row}")
            for i in range(N_ROBOTS):
                col = 1 + 3*i
                x = robot_df.at[row, col]
                y = robot_df.at[row, col + 1]
                theta = robot_df.at[row, col + 2]
                transparency = ((row - start_row) / (n_rows * final_proportion_to_draw)) ** 2
                print(f"transparency: {transparency}")
                draw_robot(x, y, theta, axes, transparency)


    #for x, y in df:
    #df.plot(ax=axes, x='time', y=column_of_interest, label=label, linewidth=0.5)

def dataframe_per_trial(datatype, arena_name, trial):

    filename = BASE_NAME + "/"
    if len(arena_name) > 0:
        filename += arena_name + "/"
    filename += "{}_{}.dat".format(datatype, trial)

    print("Loading: {}".format(filename))
    dataframe = pd.read_csv(filename, sep=" ")

    # The columns are arbitrarily labelled by the first row.  We overwrite to
    # just use integer column names.
    dataframe.columns = [i for i in range(len(dataframe.columns))]

    return dataframe

main()
