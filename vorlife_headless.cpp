#include "common/Config.hpp"
#include "common/DataLogger.hpp"
#include "vorlife/VorlifeScenario.hpp"

int main() {
    VorlifeScenario vs;
    for (int run = 0; run < config.runsPerEvaluation; ++run) {
        
        DataLogger dataLogger(run, "data/vorlife/");
        for (int i = 0; i < config.stepsPerDemoRun; ++i) {
            vs.step();

            if (i == 0 || i % 50 == 0)
                dataLogger.writeToFile(vs.simWorldState, vs.getStepCount(), vs.currentEvaluation, vs.cumulativeEvaluation);
        }

        vs.reset();
    }
    return 0;
}
