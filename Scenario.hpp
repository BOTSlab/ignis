#pragma once
#include "Sim.hpp"
#include "WorldConfig.hpp"

class Scenario {
public:
    WorldConfig config;

    std::shared_ptr<WorldState> simWorldState;

    // Simulation state variables.
    int stepCount = 0;
    bool paused = false;

    // These are not state variables, but rather flags that are set externally.
    bool doReset = false;
    bool doPause = false;
    bool doUnpause = false;
    bool doStepOnce = false;

    //
    // Subclasses must implement these two methods...
    //
    virtual void update() = 0;
    
    virtual void reset() = 0;

    void step()
    {
        if (doReset) {
            doReset = false;
            reset();
        }
        if (doPause) {
            doPause = false;
            paused = true;
        }
        if (doUnpause) {
            doUnpause = false;
            paused = false;
        }
        if (doStepOnce) {
            // We set doStepOnce to false and set paused to true at the end of this function.
            paused = false;
        }

        if (paused)
            return;

        Sim::update(simWorldState);

        update();

        stepCount++;

        if (doStepOnce) {
            doStepOnce = false;
            paused = true;
        }
    }

    void prepareToReset()
    {
        doReset = true;
    }

    void prepareToPause()
    {
        doPause = true;
    }

    void prepareToUnpause()
    {
        doUnpause = true;
    }

    void prepareToStepOnce()
    {
        doStepOnce = true;
    }

    bool isPaused() const
    {
        return paused;
    }

    int getStepCount() const
    {
        return stepCount;
    }
};