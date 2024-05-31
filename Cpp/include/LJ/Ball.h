//
// Created by 杨天越 on 28/05/2024.
//

#pragma once
#include <vector>
#include "Eigen/Dense"
using namespace std;


class Container;


class Ball {

private:
    Eigen::Vector2d Pos, Vel;
    double Radius, Mass;

    static double determinant (const Eigen::Vector2d& rDiff,
                        const Eigen::Vector2d& vDiff,
                        const double& radDiff
                        ){
        double determinantValue = pow(rDiff.dot(vDiff), 2);
        determinantValue -= (vDiff.dot(vDiff) *
                (rDiff.dot(rDiff) - std::pow(radDiff, 2)));

        return determinantValue;
    }

public:
    Ball(Eigen::Vector2d Pos, Eigen::Vector2d Vel
    , const double& Radius, const double& Mass);

    ~Ball();

    [[nodiscard]] Eigen::Vector2d getPos() const;

    [[nodiscard]] Eigen::Vector2d getVel()const;

    [[nodiscard]] double getRadius() const;

    [[nodiscard]] double getMass() const;


    void setVel (Eigen::Vector2d inputVel);
    void setPos (Eigen::Vector2d inputPos);

    void Move (double dt);

    virtual void collide(Ball& other);

    virtual void collide(Container& other);



};



