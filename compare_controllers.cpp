#include <fstream>
#include "forage/ForageScenario.hpp"
#include "common/Config.hpp"

void evaluate_controller(string filename)
{
    Config &config = Config::getInstance();

    // Read in the parameters from the given file
    cout << "Reading from " << filename << endl;
    std::ifstream parametersFile(filename);
    if (parametersFile.is_open()) {
        parameters.vec.clear();
        double value;
        while (parametersFile >> value)
            parameters.vec.push_back(value);
        cout << "Read in the following parameters: ";
        for (int i = 0; i < parameters.vec.size(); ++i)
            cout << parameters.vec[i] << " ";
        cout << endl;
    } else {
        cout << "Could not open " << filename << "." << endl;
    }

    double sumCumEval = 0;
    double sumRRCollisions = 0;
    int nRuns = 1000;
    #pragma omp parallel for reduction(+:sumCumEval, sumRRCollisions)
    for (int i = 0; i < nRuns; ++i)
    {
        ForageScenario scenario(i);
        for (int j = 0; j < config.stepsPerOptRun; ++j)
            scenario.step();
        sumCumEval += scenario.cumulativeEvaluation;
        sumRRCollisions += scenario.stepsWithRobotRobotCollisions;
    }


    double avgFitness = sumCumEval / nRuns;
    double avgRRCollisions = sumRRCollisions / nRuns;
    cout << "Average fitness: " << avgFitness << endl;
    cout << "Average robot-robot collisions: " << avgRRCollisions << endl;
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " <parameter file 1> <parameter file 2>" << endl;
        return 1;
    }

    string controller1 = argv[1];
    string controller2 = argv[2];

    evaluate_controller(controller1);
    cout << endl;
    evaluate_controller(controller2);
    return 0;
}