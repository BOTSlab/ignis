#include <iostream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/problems/schwefel.hpp>

using namespace pagmo;

struct problem_v0 {
    // Implementation of the objective function.
    vector_double fitness(const vector_double &dv) const
    {
        return {dv[0] * dv[3] * (dv[0] + dv[1] + dv[2]) + dv[2]};
    }
    // Implementation of the box bounds.
    std::pair<vector_double, vector_double> get_bounds() const
    {
        return {{1., 1., 1., 1.}, {5., 5., 5., 5.}};
    }
};

int main()
{
    // 1 - Instantiate a pagmo problem constructing it from a UDP
    // (i.e., a user-defined problem, in this case the 30-dimensional
    // generalised Schwefel test function).
    // problem prob{schwefel(30)};
    problem prob{problem_v0()};

    // 2 - Instantiate a pagmo algorithm (self-adaptive differential
    // evolution, 100 generations).
    algorithm algo{sade(100)};

    // 3 - Instantiate an archipelago with 16 islands having each 20 individuals.
    archipelago archi{16u, algo, prob, 20u};

    // 4 - Run the evolution in parallel on the 16 separate islands 10 times.
    archi.evolve(10);

    // 5 - Wait for the evolutions to finish.
    archi.wait_check();

    // 6 - Print the fitness of the best solution in each island.
    for (const auto &isl : archi) {
        std::cout << isl.get_population().champion_f()[0] << '\n';
    }
}