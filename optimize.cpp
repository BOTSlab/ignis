#include <iostream>
#include <fstream>
// For mkdir
#include <sys/stat.h>
#include <sys/types.h>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/pso_gen.hpp>
#include <pagmo/problem.hpp>

#include "AlifeScenario.hpp"
#include "Config.hpp"
#include "Parameters.hpp"

using namespace pagmo;
using std::cout, std::endl;

struct the_problem {

    vector_double fitness(const vector_double &dv) const
    {
        for (int i=0; i<parameters.vec.size(); ++i)
            parameters.vec[i] = dv[i];

        double sum = 0;
        AlifeScenario scenario;
        for (int i = 0; i < config.runsPerEvaluation; ++i)
        {
            for (int j = 0; j < config.stepsPerOptRun; ++j)
                scenario.step();
            sum += scenario.cumulativeEvaluation;
            scenario.reset();
        }

        return {sum / config.runsPerEvaluation};
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
    // Prompting the user for a base name, then create a directory with that
    // name to store results of the run.  
    std::string name;
    std::cout << "Enter the base name for this run: ";
    std::cin >> name;
    std::string dataFilenameBase = "data/" + name;
    if (mkdir(dataFilenameBase.c_str(), 0777) == -1 && errno != EEXIST)
        cerr << "Error creating directory: " << dataFilenameBase << endl;
    std::ofstream logFile(dataFilenameBase + "/optimize.log");
    std::ofstream datFile(dataFilenameBase + "/optimize.dat");
    // Copy Config.hpp into the new directory
    std::string command = "cp Config.hpp " + dataFilenameBase;
    system(command.c_str());

    // We also create two files in the current directory (last_name.dat and 
    // last_parameters.dat) to store the current name and final optimized parameters.
    std::ofstream lastNameFile("last_name.dat");
    std::ofstream lastParametersFile("last_parameters.dat");
    lastNameFile << name << std::endl;

    // Make sure Parameters.vec has the right size.
    logFile << "Number of parameters: " << parameters.vec.size() << endl;

    problem prob{the_problem()};
    logFile << prob << std::endl;

    auto baseAlgo = pso_gen(config.nGenerations);
    baseAlgo.set_verbosity(1);
    
    auto startingPop = population(prob, config.populationSize);
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
    logFile << endl << "Champion fitness: " << champF[0] << endl;

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