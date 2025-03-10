#! /usr/bin/env python
import numpy as np
import sys

if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit('Usage: %s archive.dat fit.dat n' % sys.argv[0])

    archive = np.loadtxt(sys.argv[1])
    fit = np.loadtxt(sys.argv[2])
    num_elites = int(sys.argv[3])
    
    # Sort the archive by fitness
    sorted_indices = np.argsort(fit)
    archive = archive[sorted_indices]
    
    # Print out the top num_elites in the archive, showing both the row from the archive and the fitness
    print("Top %d elites:" % num_elites)
    for i in range(num_elites):
        elite = archive[i]
        
        # Remap the first five values in this elite from the range [0, 1] to the range [-1, 1]
        elite[:5] = 2 * elite[:5] - 1
        
        print(fit[sorted_indices[i]], archive[i], sep=" ")
        