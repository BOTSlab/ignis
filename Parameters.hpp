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
    double gauci[2] = {-0.994649, 0.997238};
    double cubic[8] = {0.791813, -1, -0.93487, 1, -0.983553, 0.630351, 0.0376034, 1};
    double linearVersion1[9] = {0.593787, 0.340358, -0.445874, -0.0782425, -0.54511, -0.41046, -0.00673234, 0.0747732, 0.453639};
    double linearVersion2[12] = {0.245545, 0.813016, 0.703854, 0.645675, 0.041418, 0.562237, 0.936826, 0.741854, -0.368563, -0.0778961, 0.441406, 0.99114};
} parameters;