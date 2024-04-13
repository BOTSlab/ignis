#include <iostream>
#include <fstream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/pso_gen.hpp>
#include <pagmo/problem.hpp>
#include "AlifeScenario.hpp"
#include "Config.hpp"
#include "Parameters.hpp"

using namespace pagmo;
using std::cout, std::endl;

const unsigned int populationSize = 10;
const unsigned int runsPerEvaluation = 10;
const unsigned int stepsPerRun = 2000;

struct the_problem {

    vector_double fitness(const vector_double &dv) const
    {
        for (int i=0; i<parameters.vec.size(); ++i)
            parameters.vec[i] = dv[i];

        double sum = 0;
        AlifeScenario scenario;
        for (int i = 0; i < runsPerEvaluation; ++i)
        {
            for (int j = 0; j < stepsPerRun; ++j)
                scenario.step();
            sum += scenario.cumulativeEvaluation;
            scenario.reset();
        }

        return {sum / runsPerEvaluation};
    }
    std::pair<vector_double, vector_double> get_bounds() const
    {
        vector_double lower, upper;
        for (int i=0; i<parameters.vec.size(); ++i) {
            lower.push_back(-1);
            upper.push_back(1);
        }
        return {lower, upper};
    }
};

int main()
{
    // Create log and data files, prompting the user for a base name.  We also
    // create two files: last_name.dat and last_parameters.dat to store the 
    // current name and final optimized parameters.
    std::string name;
    std::cout << "Enter the base name for this run: ";
    std::cin >> name;
    std::ofstream logFile("data/" + name + ".log");
    std::ofstream datFile("data/" + name + ".dat");
    std::ofstream lastNameFile("last_name.dat");
    std::ofstream lastParametersFile("last_parameters.dat");
    lastNameFile << name << std::endl;

    problem prob{the_problem()};
    logFile << prob << std::endl;

    auto baseAlgo = pso_gen(200);
    baseAlgo.set_verbosity(1);
    
    auto startingPop = population(prob, populationSize);
    auto finalPop = baseAlgo.evolve(startingPop);
    auto champX = finalPop.champion_x();
    auto champF = finalPop.champion_f();
    logFile << "Champion decision vector: ";
    for (int i=0; i<champX.size(); ++i) {
        logFile << champX[i];
        lastParametersFile << champX[i];
        if (i < champX.size() - 1) {
            logFile << " ";
            lastParametersFile << " ";
        }
    }
    logFile << "Champion fitness: " << champF[0] << endl;

    auto log = baseAlgo.get_log();
    //datFile << "Gen Fevals Best dx df sigma" << endl;        
    datFile << "Gen,Fevals,gbest,Mean Vel.,Mean lbest,Avg. Dist." << endl;        
    for (auto entry : log) {
        //auto [Gen, Fevals, Best, dx, df, sigma] = entry;
        auto [Gen, Fevals, gbest, meanVel, meanLBest, avgDist] = entry;
        datFile << Gen << "," << Fevals << "," << gbest << "," << meanVel << "," << meanLBest << "," << avgDist << endl;
    }
    datFile.close();
}