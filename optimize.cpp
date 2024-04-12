#include <iostream>
#include <fstream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/pso_gen.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include "AlifeScenario.hpp"
#include "Config.hpp"
#include "Parameters.hpp"

using namespace pagmo;
using std::cout, std::endl;

const int runsPerEvaluation = 10;
const int stepsPerRun = 2000;

struct the_problem {

    vector_double fitness(const vector_double &dv) const
    {
        for (int i=0; i<parameters.n; ++i)
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
        for (int i=0; i<parameters.n; ++i) {
            lower.push_back(-1);
            upper.push_back(1);
        }
        return {lower, upper};
    }
};

int main()
{
    // Create log and data files, prompting the user for a base name.
    std::string baseName;
    std::cout << "Enter the base name for this run: ";
    std::cin >> baseName;
    std::ofstream logFile("data/" + baseName + ".log");
    std::ofstream datFile("data/" + baseName + ".dat");

    problem prob{the_problem()};
    logFile << prob << std::endl;

    auto baseAlgo = pso_gen(200);
    baseAlgo.set_verbosity(1);
    
    bool useArchipeligo = false;
    unsigned int populationSize = 10;

    if (useArchipeligo) {
        algorithm algo{baseAlgo};    
        algo.set_verbosity(1);

        archipelago archi{4u, algo, prob, populationSize}; // 4 islands
        archi.evolve(1);
        archi.wait_check();

        for (const auto &island : archi) {
            auto champX = island.get_population().champion_x();
            auto champF = island.get_population().champion_f();
            logFile << "Champion decision vector: ";
            for (auto d : champX)
                logFile << d << ", ";
            logFile << "  fitness: " << champF[0] << endl;
        }

    } else {
        auto startingPop = population(prob, populationSize);
        auto finalPop = baseAlgo.evolve(startingPop);
        auto champX = finalPop.champion_x();
        auto champF = finalPop.champion_f();
        logFile << "Champion decision vector: ";
            for (auto d : champX)
                logFile << d << ", ";
            logFile << "  fitness: " << champF[0] << endl;
    }

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