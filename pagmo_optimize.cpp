#include <iostream>
#include <fstream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include "AlifeScenario.hpp"
#include "Parameters.hpp"

using namespace pagmo;
using std::cout, std::endl;

const int runsPerEvaluation = 10;
const int stepsPerRun = 1000;

struct gauci_problem {
    // Implementation of the objective function.
    vector_double fitness(const vector_double &dv) const
    {
        parameters.gauci[0] = sigmoid(dv[0]);
        parameters.gauci[1] = sigmoid(dv[1]);

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
    // Implementation of the box bounds.
    std::pair<vector_double, vector_double> get_bounds() const
    {
        return {{-1.0, -1.0}, {1.0, 1.0}};
    }
};

int main()
{
    problem prob{gauci_problem()};
    std::cout << prob << std::endl;

    auto algo = cmaes(20);
    algo.set_verbosity(1);
    auto startingPop = population(prob, 10);
    auto finalPop = algo.evolve(startingPop);

    auto log = algo.get_log();
    // Create and open a text file
    std::ofstream myLog("mylog.txt");
    for (auto entry : log) {
        auto [Gen, Fevals, Best, dx, df, sigma] = entry;
        myLog << "Gen: " << Gen << ", Fevals: " << Fevals << ", Best: " << Best << ", dx: " << dx << ", df: " << df << ", sigma: " << sigma << endl;
    }
    myLog.close();

    auto champX = finalPop.champion_x();
    auto champF = finalPop.champion_f();
    cout << "Champion decision vector: ";
        for (auto d : champX)
            std::cout << d << ",  ";
        cout << ", fitness: " << champF[0] << endl;

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