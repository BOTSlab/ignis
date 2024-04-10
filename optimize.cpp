#include <iostream>
#include <fstream>

#include <pagmo/algorithm.hpp>
//#include <pagmo/algorithms/cmaes.hpp>
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

struct gauci_problem {
    vector_double fitness(const vector_double &dv) const
    {
        parameters.gauci[0] = dv[0]; //sigmoid(dv[0]);
        parameters.gauci[1] = dv[1]; //sigmoid(dv[1]);

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
        return {{-1, -1}, {1, 1}};
    }
};

struct cubic_problem {
    vector_double fitness(const vector_double &dv) const
    {
        parameters.cubic[0] = dv[0]; //sigmoid(dv[0]);
        parameters.cubic[1] = dv[1];
        parameters.cubic[2] = dv[2];
        parameters.cubic[3] = dv[3];
        parameters.cubic[4] = dv[4]; //sigmoid(dv[1]);
        parameters.cubic[5] = dv[5];
        parameters.cubic[6] = dv[6];
        parameters.cubic[7] = dv[7];

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
        return {{-1, -1, -1, -1, -1, -1, -1, 1}, {1, 1, 1, 1, 1, 1, 1, 1}};
    }
};

/*
struct linear_version1_problem {
    vector_double fitness(const vector_double &dv) const
    {
        parameters.linear[0] = dv[0];
        parameters.linear[1] = dv[1];
        parameters.linear[2] = dv[2];
        parameters.linear[3] = dv[3];
        parameters.linear[4] = dv[4];
        parameters.linear[5] = dv[5];
        parameters.linear[6] = dv[6];
        parameters.linear[7] = dv[7];
        parameters.linear[8] = dv[8];

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
        return {{-1, -1, -1, -1, -1, -1, -1, -1, -1}, {1, 1, 1, 1, 1, 1, 1, 1, 1}};
    }
};
*/

struct linear_version2_problem {
    vector_double fitness(const vector_double &dv) const
    {
        parameters.linearVersion2[0] = dv[0];
        parameters.linearVersion2[1] = dv[1];
        parameters.linearVersion2[2] = dv[2];
        parameters.linearVersion2[3] = dv[3];
        parameters.linearVersion2[4] = dv[4];
        parameters.linearVersion2[5] = dv[5];
        parameters.linearVersion2[6] = dv[6];
        parameters.linearVersion2[7] = dv[7];
        parameters.linearVersion2[8] = dv[8];
        parameters.linearVersion2[9] = dv[9];
        parameters.linearVersion2[10] = dv[10];
        parameters.linearVersion2[11] = dv[11];

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
        return {{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
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

    problem *prob = nullptr;
    if (config.controlMethod == AlifeControlMethod::EvolvedGauci)
        prob = new problem{gauci_problem()};
    else if (config.controlMethod == AlifeControlMethod::EvolvedCubic)
        prob = new problem{cubic_problem()};
    else if (config.controlMethod == AlifeControlMethod::EvolvedLinearVersion2)
        prob = new problem{linear_version2_problem()};
    else
        throw std::runtime_error("Unknown control method.");

    logFile << *prob << std::endl;

    //auto algo = cmaes(500);
    //auto algo = xnes(50);
    auto algo = pso_gen(200);
    
    algo.set_verbosity(1);
    auto startingPop = population(*prob, 10);
    auto finalPop = algo.evolve(startingPop);

    auto log = algo.get_log();
    //datFile << "Gen Fevals Best dx df sigma" << endl;        
    datFile << "Gen,Fevals,gbest,Mean Vel.,Mean lbest,Avg. Dist." << endl;        
    for (auto entry : log) {
        //auto [Gen, Fevals, Best, dx, df, sigma] = entry;
        auto [Gen, Fevals, gbest, meanVel, meanLBest, avgDist] = entry;
        datFile << Gen << "," << Fevals << "," << gbest << "," << meanVel << "," << meanLBest << "," << avgDist << endl;
    }
    datFile.close();

    auto champX = finalPop.champion_x();
    auto champF = finalPop.champion_f();
    logFile << "Champion decision vector: ";
        for (auto d : champX)
            logFile << d << ", ";
        logFile << "  fitness: " << champF[0] << endl;

    delete prob;
    /*
    auto underlyingAlgo = cmaes(50);
    algorithm algo{underlyingAlgo};    
    algo.set_verbosity(1);

    // 3 - Instantiate an archipelago with 4 islands having each 10 individuals.
    archipelago archi{4u, algo, prob, 10u};

    // 4 - Run the evolution in parallel.
    archi.evolve(1);

    archi.wait_check();

    

    // 6 - Print the fitness of the best solution in each island.
    for (const auto &island : archi) {
        auto champX = island.get_population().champion_x();
        auto champF = island.get_population().champion_f();
        cout << "Champion decision vector: ";
        for (auto d : champX)
            std::cout << d << ",  ";
        cout << ", fitness: " << champF[0] << endl;
    }
    */
}