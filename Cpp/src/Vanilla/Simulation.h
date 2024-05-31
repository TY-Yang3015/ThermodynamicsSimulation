//
// Created by 杨天越 on 29/05/2024.
//

#pragma once

#include "Container.h"
#include "Ball.h"
#include "GLFW/glfw3.h"
#include <vector>

class Simulation {
private:
    double containerRadius, ballRadius, ballSpeed, ballMass, rMax;
    int nRings, multi;

    vector<Ball> ballList;

    Container simContainer = Container(0, 0);

    static Eigen::ArrayXXd positionInitialiser (double rMax, int nRings, int multi);

    static Eigen::ArrayXXd velocityInitialiser(int numOfBalls, double speed);


public:

    Simulation(const double& containerRadius, const double& ballRadius,
               const double& ballSpeed, const double& ballMass,
               const double& rMax, const int& nRings, const int& multi);

    ~Simulation();

    std::vector<Ball> balls ();

    Container container ();

    void nextCollision ();

    int runByCollision(int numOfCollision);

    void nextTimeStep(double time);

    int runByTime(double time, int frame);

    // double kineticEnergy ();

    // Eigen::Vector2d momentum ();

    // double time ();

    // double pressure ();

    // double tEquipartition ();

    // double tIdeal ();

    // Eigen::Vector2d speeds ();


};
