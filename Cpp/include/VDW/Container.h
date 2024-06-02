//
// Created by 杨天越 on 28/05/2024.
//

#pragma once

#include "Ball.h"
#include "Eigen/Dense"

class Container : public Ball {
private:
    double dp = 0.;

    static double determinant (const Eigen::Vector2d& rDiff,
                               const Eigen::Vector2d& vDiff,
                               const double& radDiff
    ){
        double determinantValue = pow(rDiff.dot(vDiff), 2);
        determinantValue -= vDiff.dot(vDiff) *
                            (rDiff.dot(rDiff) - std::pow(radDiff, 2));

        return determinantValue;
    }

public:
    Container(const double &radius, const double &mass);

    ~Container();

    void addDp (double dp);

    [[nodiscard]] double dpTot () const;

    double volume ();

    double surfaceArea ();



};



