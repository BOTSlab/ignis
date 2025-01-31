#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

# Modify font size for all matplotlib plots.
plt.rcParams.update({'font.size': 15})

def main():
    K3 = -0.775121
    K4 = 0.164315
    delta_alpha = 0.01

    alpha = np.arange(-np.pi, np.pi, delta_alpha)

    phi = K3 * np.cos(alpha - K4 * np.pi)

    plt.plot(alpha, phi, 'b')
    
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)

    plt.axvline(np.pi/2, color='grey', linestyle='dashed', linewidth=0.5)


    plt.show()

main()
