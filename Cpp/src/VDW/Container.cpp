//
// Created by 杨天越 on 28/05/2024.
//

#include "Container.h"
#include "Eigen/Dense"
#include <cmath>


Eigen::Vector2d initialVel = {0, 0};
Eigen::Vector2d initialPos = {0, 0};

Container::Container(const double &radius, const double &mass)
: Ball(initialPos, initialVel, radius, mass)
{}

Container::~Container() = default;

void Container::addDp(double increDp) {
    dp += increDp;
}

double Container::dpTot() const {
    return dp;
}



double Container::volume() {
    return pow(this->getRadius(), 2) * M_PI;
}

double Container::surfaceArea() {
    return 2 * M_PI * this->getRadius();
}

