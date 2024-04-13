/**
 * A place to store the parameters that we are optimizing over.
 */
#pragma once

// Custom sigmoid function to convert decision variables to the interval [-1, 1].
// This comes from equation (4) in Gauci et al.
//double sigmoid(double x) {
//    return (1 - exp(-x)) / (1 + exp(-x));
//}

struct Parameters
{
    /*
    double gauci[2] = {-0.994649, 0.997238};
    double cubic[8] = {0.791813, -1, -0.93487, 1, -0.983553, 0.630351, 0.0376034, 1};
    double linear[9] = {0.593787, 0.340358, -0.445874, -0.0782425, -0.54511, -0.41046, -0.00673234, 0.0747732, 0.453639};
    double spinner[6] = {0.430803, 1, 1, -0.98424, -1, 0.97215};
    double gauci3[3] = {0, 0, 0};
    */

    std::vector<double> vec = {-0.914085, 0.800751, -0.39134, -0.623223, 0.991232};
} parameters;