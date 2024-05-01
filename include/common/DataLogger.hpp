#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>

// For mkdir
#include <sys/stat.h>
#include <sys/types.h>

#include "common/Config.hpp"
#include "common/WorldState.hpp"

using namespace std;

class DataLogger {
    int m_trialIndex;
    ofstream m_statsStream, m_robotPoseStream, m_robotSensorAngleStream, m_puckPositionStream;

public:
    DataLogger(int trialIndex, std::string dataFilenameBase)
        : m_trialIndex(trialIndex)
    {
        if (mkdir(dataFilenameBase.c_str(), 0777) == -1 && errno != EEXIST)
            cerr << "Error creating directory: " << dataFilenameBase << endl;

        stringstream statsFilename, robotPoseFilename, robotSensorAngleFilename, puckPositionFilename;
        statsFilename << dataFilenameBase << "/stats_" << trialIndex << ".dat";
        robotPoseFilename << dataFilenameBase << "/robotPose_" << trialIndex << ".dat";
        robotSensorAngleFilename << dataFilenameBase << "/robotSensorAngle_" << trialIndex << ".dat";
        puckPositionFilename << dataFilenameBase << "/puckPosition_" << trialIndex << ".dat";

        m_statsStream = ofstream(statsFilename.str());
        m_robotPoseStream = ofstream(robotPoseFilename.str());
        m_robotSensorAngleStream = ofstream(robotSensorAngleFilename.str());
        m_puckPositionStream = ofstream(puckPositionFilename.str());
    }

    void writeToFile(shared_ptr<WorldState> worldState, double stepCount, double eval, double cumEval)
    {
        m_statsStream << stepCount << " " << eval << " " << cumEval << " " << worldState->nRobotRobotCollisions << "\n";
        m_statsStream.flush();

        m_robotPoseStream << stepCount;
        for (auto& robot : worldState->robots) {
            m_robotPoseStream << " " << (int)robot.pos.x << " " << (int)robot.pos.y << " " << ((int)(1000 * robot.theta)) / 1000.0; // Rounding angle to 3 decimals
        }
        m_robotPoseStream << "\n";
        m_robotPoseStream.flush();

        m_robotSensorAngleStream << stepCount;
        for (auto& robot : worldState->robots) {
            m_robotSensorAngleStream << " " << ((int)(1000 * robot.sensorAngle)) / 1000.0; // Rounding angle to 3 decimals
        }
        m_robotSensorAngleStream << "\n";
        m_robotSensorAngleStream.flush();

        m_puckPositionStream << stepCount;
        for (auto& puck : worldState->pucks) {
            m_puckPositionStream << " " << (int)puck.pos.x << " " << (int)puck.pos.y;
        }
        m_puckPositionStream << "\n";
        m_puckPositionStream.flush();
    }
};
