#pragma once
#include "common/CommonTypes.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>
#include <string>

enum class ControlMethod
{
    ThreeParameterGauci,
    FiveParameterCai25,
    EightParameterRandom
};

class Config
{
public:
    static Config& getInstance()
    {
        if (!instance)
        {
            instance = std::unique_ptr<Config>(new Config());
            instance->load("config.yaml");
        }
        return *instance;
    }

    ControlMethod controlMethod = ControlMethod::ThreeParameterGauci;

    int width = 800;
    int height = 400;
    int coldStartSteps = 0;
    int numberOfRobots = 5;
    int numberOfPucks = 20;
    double robotRadius = 10;
    double puckRadius = 20;
    double maxForwardSpeed = 0.25;
    double maxAngularSpeed = 1.0;

    unsigned int nGenerations = 500;
    unsigned int populationSize = 10;

    unsigned int runsPerEvaluation = 10;
    unsigned int stepsPerOptRun = 2000;

    unsigned int stepsPerDemoRun = 2000;

    double segmentSensorOffset = 0.1;
    double maxSegmentSensorLength = sqrt(width * width + height * height);
    double slowedSteps = 0;

private:
    void load(const std::string& filename)
    {
        YAML::Node config = YAML::LoadFile(filename);

        std::string controlMethodStr = config["controlMethod"].as<std::string>();
        if (controlMethodStr == "ThreeParameterGauci")
            controlMethod = ControlMethod::ThreeParameterGauci;
        else if (controlMethodStr == "FiveParameterCai25")
            controlMethod = ControlMethod::FiveParameterCai25;
        else if (controlMethodStr == "EightParameterRandom")
            controlMethod = ControlMethod::EightParameterRandom;
        else
            throw std::runtime_error("Unknown control method: " + controlMethodStr);

        width = config["width"].as<int>();
        height = config["height"].as<int>();
        coldStartSteps = config["coldStartSteps"].as<int>();
        numberOfRobots = config["numberOfRobots"].as<int>();
        numberOfPucks = config["numberOfPucks"].as<int>();
        robotRadius = config["robotRadius"].as<double>();
        puckRadius = config["puckRadius"].as<double>();
        maxForwardSpeed = config["maxForwardSpeed"].as<double>();
        maxAngularSpeed = config["maxAngularSpeed"].as<double>();

        nGenerations = config["nGenerations"].as<unsigned int>();
        populationSize = config["populationSize"].as<unsigned int>();

        runsPerEvaluation = config["runsPerEvaluation"].as<unsigned int>();
        stepsPerOptRun = config["stepsPerOptRun"].as<unsigned int>();

        stepsPerDemoRun = config["stepsPerDemoRun"].as<unsigned int>();

        segmentSensorOffset = config["segmentSensorOffset"].as<double>();
        maxSegmentSensorLength = config["maxSegmentSensorLength"].as<double>();
        slowedSteps = config["slowedSteps"].as<double>();
    }

    Config() = default;
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    static std::unique_ptr<Config> instance;
};

// Initialize the static member
std::unique_ptr<Config> Config::instance = nullptr;