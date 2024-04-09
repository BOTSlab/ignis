/**
 * A place to store the parameters that we are optimizing over.
 */
#pragma once

// Custom sigmoid function to convert decision variables to the interval [-1, 1].
// This comes from equation (4) in Gauci et al.
double sigmoid(double x) {
    return (1 - exp(-x)) / (1 + exp(-x));
}

struct Parameters
{
    double gauci[2] = {0.0, 0.0};
} parameters;