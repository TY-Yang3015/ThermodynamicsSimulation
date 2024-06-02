//
// Created by 杨天越 on 28/05/2024.
//

#include "Container.h"
#include <cmath>
#include <iostream>

extern double posInf;

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

double Container::timeToCollision(const Ball& other) {
    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();
    double radDiff = this->getRadius() - other.getRadius();

    double det = Container::determinant(rDiff, vDiff, radDiff);
    if (det > 0) {
        double dt = sqrt(det);
        dt -= rDiff.dot(vDiff);
        dt *= 1 / vDiff.dot(vDiff);

        if (dt > 0) {
            return dt;
        } else {
            return posInf;
        }
    } else {
        return posInf;
    }
}


void Container::collide(Ball& other) {
    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();

    Eigen::Vector2d v1 = this->getVel();
    v1 -= ((2 * other.getMass() / (this->getMass() + other.getMass()))
           * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    Eigen::Vector2d v2 = other.getVel();
    v2 += ((2 * this->getMass() / (this->getMass() + other.getMass()))
           * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    Eigen::Vector2d dpChange = v1 - this->getVel();
    dpChange *= this->getMass();
    this->addDp(dpChange.norm());


    this->setVel(v1);
    other.setVel(v2);


}



double Container::volume() {
    return pow(this->getRadius(), 2) * M_PI;
}

double Container::surfaceArea() {
    return 2 * M_PI * this->getRadius();
}
