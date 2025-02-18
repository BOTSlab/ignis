#include "map_elites.hpp"

#include "forage/ForageScenario.hpp"
#include "common/Config.hpp"
#include "forage/Parameters.hpp"

template <typename Params, typename S = double>
struct ForagingProblem {
    using indiv_t = Eigen::Matrix<S, 1, Params::dim_search_space, Eigen::RowMajor>;
    using features_t = Eigen::Matrix<S, 1, Params::dim_features, Eigen::RowMajor>;
    // keep the intermediate values to avoid reallocations
    //indiv_t _t;
    //indiv_t _c;
    features_t _features;

    const features_t& eval(const indiv_t& v, S& fit)
    {
        // Remap the first five parameters to the interval [-1, 1]
        for (int i=0; i<4; ++i)
            parameters.vec[i] = 2 * v[i] - 1;
        parameters.vec[5] = v[5];
        parameters.vec[6] = v[6];
        parameters.vec[7] = v[7];

        double sum = 0;
        double sumFeature1 = 0;
        double sumFeature2 = 0;
        #pragma omp parallel for reduction(+:sum)
        for (int i = 0; i < config.runsPerEvaluation; ++i)
        {
            ForageScenario scenario(i);
            for (int j = 0; j < config.stepsPerOptRun; ++j)
                scenario.step();
            sum += scenario.cumulativeEvaluation;

            if (i == 0) {
                // The features are defined only for the initial run which is
                // based on random seed 0. Having the features vary across runs
                // seems to impair MAP-Elites performance.

                // It makes sense to use the width below, however, the realistic
                // average distance is smaller than the width, here we choose 60% smaller.
                //sumFeature1 += scenario.cumulativeAverageRobotRobotDistance / (config.width * config.stepsPerOptRun);
                sumFeature1 = scenario.cumulativeAverageRobotRobotDistance / (0.6 * config.width * config.stepsPerOptRun);

                sumFeature2 = 0.5*(1 + scenario.cumulativeAverageRobotAngularSpeed / (config.maxAngularSpeed * config.stepsPerOptRun));
            }
        }

        fit = sum / config.runsPerEvaluation;
    
        // Choosing the last two parameters as features
        // _features[0] = 0.5 * (1 + parameters.vec[3]);
        // _features[1] = 0.5 * (1 + parameters.vec[4]);

        _features[0] = sumFeature1;
        _features[1] = sumFeature2;

        //fit = 1 - std::sqrt((v.array() - v.mean()).square().sum() / (v.size() - 1.0));
        //_t = 2 * M_PI * v.array() - M_PI;
        //_c = indiv_t::Zero();
        //std::partial_sum(_t.begin(), _t.end(), _c.begin(), std::plus<double>());
        //_features[0] = _c.array().cos().sum() / (2. * v.size()) + 0.5;
        //_features[1] = _c.array().sin().sum() / (2. * v.size()) + 0.5;
        return _features;
    }
};

struct Params {
    static constexpr int dim_features = 2;
    static constexpr int dim_search_space = 8;
    static constexpr int batch_size = 1; //8; // 128;
    static constexpr double sigma_1 = 0.15;
    static constexpr double sigma_2 = 0.01;
    static constexpr double infill_pct = 0.2;
    static constexpr bool verbose = false;
    static constexpr bool grid = false;
    static constexpr int grid_size = 64;
    static constexpr int num_cells = grid ? grid_size * grid_size : 12000; // 12000; // 8192;
};

int main()
{
    using fit_t = ForagingProblem<Params, float>;
    using map_elites_t = map_elites::MapElites<Params, fit_t, float>;

    auto start = std::chrono::high_resolution_clock::now();
    map_elites_t map_elites;

    std::ofstream qd_ofs("qd.dat");

    // With a batch_size of 1, 1000 steps takes about 4 minutes
    // if grid is false, then 1000 steps is about 6 minutes
    size_t n = 20000; // 1e6;
    for (size_t i = 0; i < n / Params::batch_size; ++i) {
        map_elites.step();
        qd_ofs << i * Params::batch_size << " " << map_elites.qd_score() << std::endl;
        if (Params::verbose)
            std::cout << i << " ";
        std::cout.flush();
    }
    auto end = std::chrono::high_resolution_clock::now();
    double t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "Total time:" << t / 1000.0 << "s" << std::endl;

    std::cout << "writing...";
    std::cout.flush();
    std::ofstream c("centroids.dat");
    c << map_elites.centroids() << std::endl;
    std::ofstream f("fit.dat");
    f << map_elites.archive_fit() << std::endl;

    std::ofstream a("archive.dat");
    a << map_elites.archive() << std::endl;

    std::cout << "done" << std::endl;
    return 0;
}