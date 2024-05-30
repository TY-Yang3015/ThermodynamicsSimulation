//
// Created by 杨天越 on 29/05/2024.
//

#ifndef CPP_SIMULATION_H
#define CPP_SIMULATION_H

#include "Eigen/Dense"
#include <vector>
#include "Container.h"
#include "Ball.h"
#include "GLFW/glfw3.h"

class Simulation {

    double phi0 = 10000;

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

    void nextTimeStep(double time);

    int run(double time, int frame);

    pair<Eigen::Vector2d, Eigen::Vector2d> verletUpdate(Ball& ball, const double& dt);

    Eigen::Vector2d verletGlobalUpdate(const double& dt);

    Eigen::Vector2d vdwForce (Ball& ball, const Eigen::Vector2d& currentPos);

    // double kineticEnergy ();

    // Eigen::Vector2d momentum ();

    // double time ();

    // double pressure ();

    // double tEquipartition ();

    // double tIdeal ();

    // Eigen::Vector2d speeds ();


};


#endif //CPP_SIMULATION_H
