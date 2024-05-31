//
// Created by 杨天越 on 29/05/2024.
//



#include "Simulation.h"
#include "Ball.h"
#include "Container.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>

// #include "glad/glad.h"  // GLAD must be included before GLFW
// #include <GLFW/glfw3.h> // GLFW for windowing and input
// #include "glm/glm.hpp"  // Core GLM functionality (vectors, matrices)
// #include "glm/gtc/matrix_transform.hpp"  // For glm::translate, glm::rotate, glm::scale
// #include "glm/gtc/type_ptr.hpp"



Simulation::Simulation(
        const double &containerRadius, const double &ballRadius, const double &ballSpeed,
        const double &ballMass, const double &rMax, const int &nRings, const int &multi) :
        containerRadius(containerRadius), ballRadius(ballRadius),
        ballSpeed(ballSpeed), ballMass(ballMass),
        rMax(rMax), nRings(nRings), multi(multi)
        {
    Eigen::ArrayXXd positions = Simulation::positionInitialiser(rMax, nRings, multi);
    Eigen::ArrayXXd velocities = Simulation::velocityInitialiser(positions.rows(), ballSpeed);
    for (int i = 0; i < positions.rows(); i++){
        ballList.emplace_back(positions.row(i), velocities.row(i), ballRadius, ballMass);
    }

    simContainer = Container(containerRadius, 10000000.);
        }

Simulation::~Simulation() = default;

Eigen::ArrayXXd Simulation::positionInitialiser(double rMax, int nRings, int multi) {
    int numOfBalls = (multi + nRings * multi) * nRings / 2;
    Eigen::ArrayXXd positionArray(numOfBalls, 2);  // Create a 2D array to store positions

    double radialIncrement = rMax / nRings;

    int k = 0;  // Initialize index k to 0
    double angularIncrement, radius, theta;
    for (int i = 1; i <= nRings; i++) {
        angularIncrement = 2 * M_PI / (multi * i);
        radius = radialIncrement * i;

        for (int j = 0; j < multi * i; j++) {
            theta = angularIncrement * j;  // Calculate the current angle
            positionArray(k, 0) = radius * std::cos(theta);  // X coordinate
            positionArray(k, 1) = radius * std::sin(theta);  // Y coordinate
            k++;
        }
    }
    return positionArray;
}

Eigen::ArrayXXd Simulation::velocityInitialiser (int numOfBalls, double speed) {
    Eigen::ArrayXXd velocityArray(numOfBalls, 2);

    std::random_device rd;  // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(0.0, 2*M_PI); // Define the distribution

    std::vector<double> random_numbers;
    random_numbers.reserve(numOfBalls);
    for (int i = 0; i < numOfBalls; ++i) {
            random_numbers.push_back(distr(gen));
        }

    Eigen::ArrayXd angles = Eigen::Map<Eigen::VectorXd>(random_numbers.data(), random_numbers.size());
    velocityArray.col(0) = angles.cos() * speed;
    velocityArray.col(1) = angles.sin() * speed;

    return velocityArray;
}

vector<Ball> Simulation::balls() {
    return ballList;
}

Container Simulation::container() {
    return simContainer;
}

void Simulation::setAttractionStrength (double phi) {
    phi0 = phi;
};

pair<Eigen::Vector2d, Eigen::Vector2d> Simulation::verletUpdate(Ball& ball, const double& dt) {

    Eigen::Vector2d pHalf, rFinal, pFinal;

    pHalf = ball.getMass() * ball.getVel()
                + 0.5 * dt * this->vdwForce(ball, ball.getPos());

    rFinal = ball.getPos() + dt * pHalf / ball.getMass();

    pFinal = pHalf + 0.5 * dt * this->vdwForce(ball, rFinal);

    return make_pair(pFinal / ball.getMass(), rFinal);

}

Eigen::Vector2d Simulation::verletGlobalUpdate(const double& dt) {
    std::vector<Eigen::Vector2d> newVelocities, newPositions;
    for (size_t i = 0; i < this->balls().size(); ++i) {
        auto update = this->verletUpdate(this->ballList[i], dt);
        newVelocities.push_back(update.first);
        newPositions.push_back(update.second);
    }

    for (size_t i = 0; i < this->balls().size(); ++i) {
        this->ballList[i].setVel(newVelocities[i]);
        this->ballList[i].setPos(newPositions[i]);
    }
}

Eigen::Vector2d Simulation::vdwForce (Ball& ball, const Eigen::Vector2d& currentPos){
    Eigen::Vector2d force = Eigen::Vector2d::Zero();
    Eigen::Vector2d r;
    // Boundary forces (if any), assuming container center at (0, 0)

    for (size_t i = 0; i < this->balls().size(); ++i) {
        if (&ball != &ballList[i]) {
            r = ballList[i].getPos() - currentPos;

            if (r.norm() > 2 * ball.getRadius()) {
                force += (6 * phi0 / ball.getRadius())
                         * pow((ball.getRadius() / r.norm()), 7)
                         * (r / r.norm());
            }

            // else {
            //    ballList[i].collide(ball);
            //    return Eigen::Vector2d::Zero();
            // }
            //force -= (12 * phi0 / ball.getRadius())
            //             * pow((ball.getRadius() / r.norm()), 13)
            //             * (r / r.norm());
        }
    }

    //if (currentPos.norm() > containerRadius - ball.getRadius()) {
    //    Eigen::Vector2d toCenter = -currentPos;  // Vector pointing towards the center
     //   force += 10000000. * (toCenter.normalized()); // Apply force towards center
    //}


    return force;
}

void Simulation::nextTimeStep(double frameTime) {
    for (size_t j = 0; j < ballList.size(); ++j) {
        for (size_t i = j + 1; i < ballList.size(); ++i) { // Start from j+1 to avoid self-comparison and redundant checks
            Eigen::Vector2d dist = (ballList[i].getPos() - ballList[j].getPos());
            if ((dist.norm() > 0) && (dist.norm() < 2*ballList[j].getRadius())) {
                ballList[j].setPos(-2*ballList[j].getRadius() * dist.normalized() + ballList[i].getPos());
                ballList[j].collide(ballList[i]);
            }
        }

        if (ballList[j].getPos().norm() > simContainer.getRadius() - ballList[j].getRadius()) {
            //Eigen::Vector2d vr = (ballList[j].getPos()/ballList[j].getPos().norm()).dot(ballList[j].getVel())
            //                     * (ballList[j].getPos()/ballList[j].getPos().norm());
            //ballList[j].setVel(ballList[j].getVel() - 2.*vr);
            ballList[j].collide(simContainer);
            ballList[j].setPos(ballList[j].getPos().normalized() * (simContainer.getRadius() - ballList[j].getRadius()));

        }
    }
    this->verletGlobalUpdate(frameTime);
}

void drawCircle(float cx, float cy, float r, int num_segments) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy); // Center of circle
    glColor3f(1.0f, 1.0f, 1.0f);
    for(int i = 0; i <= num_segments; i++) {
        float theta = 2.0f * 3.1415926f * float(i) / float(num_segments);
        float x = r * cosf(theta);
        float y = r * sinf(theta);
        glVertex2f(x + cx, y + cy);
    }
    glEnd();
}


void drawHollowCircle(float cx, float cy, float radius, int num_segments) {
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < num_segments; i++) {
        float theta = 2.0f * M_PI * float(i) / float(num_segments); // Current angle
        float x = radius * cosf(theta); // Calculate the x component
        float y = radius * sinf(theta); // Calculate the y component
        glVertex2f(x + cx, y + cy); // Output vertex
    }
    glEnd(); // End of GL_LINE_LOOP
}



int Simulation::run(double time, int frame) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 800, "Colliding Balls Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Set up viewport and orthogonal projection to match window size
    glViewport(0, 0, 1600, 1600);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1600, 1600.0, -1600, 1600.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int i = 0;
    double frameTime = time / frame;
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        nextTimeStep(frameTime); // Process the next step of the simulation

        // Draw all elements
        i++;
        for (auto& ball : balls()) {
            drawCircle(ball.getPos().x(), ball.getPos().y(), ball.getRadius(), 20);
            drawHollowCircle(ball.getPos().x(), ball.getPos().y(), ball.getRadius(), 20);
        }
        drawHollowCircle(simContainer.getPos().x(), simContainer.getPos().y(),
                         simContainer.getRadius(), 80);

        glfwSwapBuffers(window);
        glfwPollEvents();

        if (i > frame) {
            break;
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
