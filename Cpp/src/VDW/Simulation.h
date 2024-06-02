//
// Created by 杨天越 on 29/05/2024.
//

#pragma once

#include "Eigen/Dense"
#include <vector>
#include "Container.h"
#include "Ball.h"
#include "GLFW/glfw3.h"
#include "../util/Utils.h"

class Simulation {

    double phi0 = 10000;

private:
    double containerRadius, ballRadius, ballSpeed, ballMass, rMax;
    int nRings, multi;

    vector<Ball> ballList;

    Container simContainer = Container(0, 0);

    double currentTime = 0;
    double numOfCollision = 0;

    bool logSystemMicro, logSystemMacro = false;

    std::string currentFolder;

    const double kb = 1.380649e-23;

    static Eigen::ArrayXXd positionInitialiser (double rMax, int nRings, int multi);

    static Eigen::ArrayXXd velocityInitialiser(int numOfBalls, double speed);


public:

    Simulation(const double& containerRadius, const double& ballRadius,
               const double& ballSpeed, const double& ballMass,
               const double& rMax, const int& nRings, const int& multi);

    ~Simulation();

    std::vector<Ball> balls ();

    Container container ();

    void nextTimeStep(double time);

    int run(double time, int frame);

    pair<Eigen::Vector2d, Eigen::Vector2d> verletUpdate(Ball& ball, const double& dt);

    void verletGlobalUpdate(const double& dt);

    Eigen::Vector2d vdwForce (Ball& ball, const Eigen::Vector2d& currentPos);

    void setAttractionStrength (double phi);

    double kineticEnergy ();

    Eigen::Vector2d momentum ();

    [[nodiscard]] double time () const;

    double pressure ();

    double tEquipartition ();

    double tIdeal ();

    vector<double> speeds ();

    void recordSimulationSpecs ();

    vector<Eigen::Vector2d> ballVelocities();

    vector<Eigen::Vector2d> ballPositions();

    void logSystemMacroInformation ();

    void logSystemMicroInformation ();

    void logSystemChronology ();

    void setSystemLog (const bool& micro, const bool& macro);

    void writeMacroLogHeader ();

};



