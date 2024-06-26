//
// Created by 杨天越 on 28/05/2024.
//

#include "Ball.h"
#include "Container.h"
#include <utility>
#include <limits>


double posInf = std::numeric_limits<double>::infinity();

Ball::Ball(Eigen::Vector2d Pos, Eigen::Vector2d Vel, const double& Radius, const double& Mass) :
Pos(std::move(Pos)), Vel(std::move(Vel)), Radius(Radius), Mass(Mass) {};

Ball::~Ball() = default;

[[nodiscard]] Eigen::Vector2d Ball::getPos() const {
    return Pos;
}

[[nodiscard]] Eigen::Vector2d Ball::getVel() const {
    return Vel;
}

[[nodiscard]] double Ball::getRadius() const {
    return Radius;
}

[[nodiscard]] double Ball::getMass() const {
    return Mass;
}

void Ball::setVel(Eigen::Vector2d inputVel){
    Vel = std::move(inputVel);
}

void Ball::Move(double dt) {
    Pos += Vel * dt;
}

double Ball::timeToCollision(const Ball& other) {
    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();
    double radDiff = this->getRadius() + other.getRadius();

    double det = Ball::determinant(rDiff, vDiff, radDiff);
    if (det > 0) {
        double dt = -sqrt(det);
        dt -= rDiff.dot(vDiff);
        dt *= 1 / vDiff.dot(vDiff);

        if (dt > 0){
            return dt;
        } else {
            return posInf;
        }
    } else {
        return posInf;
    }
}

double Ball::timeToCollision(const Container& other) {
    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();
    double radDiff = this->getRadius() - other.getRadius();

    double det = Ball::determinant(rDiff, vDiff, radDiff);
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

void Ball::collide(Ball& other) {
    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();

    Eigen::Vector2d v1 = this->getVel();
    v1 -= ((2. * other.getMass() / (this->getMass() + other.getMass()))
            * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    Eigen::Vector2d v2 = other.getVel();
    v2 += ((2. * this->getMass() / (this->getMass() + other.getMass()))
           * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    this->setVel(v1);
    other.setVel(v2);

}

void Ball::collide(Container& other) {

    Eigen::Vector2d rDiff = this->getPos() - other.getPos();
    Eigen::Vector2d vDiff = this->getVel() - other.getVel();

    Eigen::Vector2d v1 = this->getVel();
    v1 -= ((2 * other.getMass() / (this->getMass() + other.getMass()))
           * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    Eigen::Vector2d v2 = other.getVel();
    v2 += ((2 * this->getMass() / (this->getMass() + other.getMass()))
           * (vDiff.dot(rDiff) / rDiff.dot(rDiff))) * rDiff;

    Eigen::Vector2d dp = v2 - other.getVel();
    dp *= other.getMass();
    other.addDp(dp.norm());

    this->setVel(v1);
    other.setVel(v2);

}