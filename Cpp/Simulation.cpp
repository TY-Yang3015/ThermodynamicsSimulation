//
// Created by 杨天越 on 29/05/2024.
//

#include "Simulation.h"
#include "Ball.h"
#include "Container.h"
#include <random>
#include <algorithm>
#include <iostream>

// #include "glad/glad.h"  // GLAD must be included before GLFW
#include <GLFW/glfw3.h> // GLFW for windowing and input
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
        };

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

void Simulation::nextCollision() {
    double dtNext = std::numeric_limits<double>::infinity();
    std::vector<std::pair<Ball*, Ball*>> collisionBalls;
    double dtCurrent;

    for (size_t j = 0; j < ballList.size(); ++j) {
        for (size_t i = j + 1; i < ballList.size(); ++i) { // Start from j+1 to avoid self-comparison and redundant checks
            dtCurrent = ballList[j].timeToCollision(ballList[i]);

            if (dtCurrent < dtNext) {
                dtNext = dtCurrent;
                collisionBalls.clear(); // Clear previous pairs
                collisionBalls.emplace_back(&ballList[j], &ballList[i]);
            } else if ((dtCurrent == dtNext) && (dtCurrent != std::numeric_limits<double>::infinity())) {
                // Check if the reversed pair is already in the list before adding
                // if (std::find(collisionBalls.begin(), collisionBalls.end(), std::make_pair(&ballList[i], &ballList[j])) == collisionBalls.end()) {
                collisionBalls.emplace_back(&ballList[j], &ballList[i]);
                //}
            }
        }
    }

    std::vector<std::pair<Ball*, Container*>> collisionWithContainer;
    for (size_t j = 0; j < ballList.size(); ++j) {
        dtCurrent = ballList[j].timeToCollision(simContainer);
        if (dtCurrent < dtNext) {
            collisionBalls.clear();
            collisionWithContainer.clear();
            dtNext = dtCurrent;
            collisionWithContainer.emplace_back(&ballList[j], &simContainer);
        } else if ((dtCurrent == dtNext) && (dtCurrent != std::numeric_limits<double>::infinity())) {
            collisionWithContainer.emplace_back(&ballList[j], &simContainer);
        }
    }

    for (size_t j = 0; j < ballList.size(); ++j) {
        ballList[j].Move(dtNext);
    }
    simContainer.Move(dtNext);

    if (!collisionBalls.empty()) {
        for (size_t j = 0; j < collisionBalls.size(); ++j) {
            collisionBalls[j].first->collide(*collisionBalls[j].second);
        }
    } else {
        for (size_t j = 0; j < collisionWithContainer.size(); ++j) {
            collisionWithContainer[j].second->collide(*collisionWithContainer[j].first);
        }
    }
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


// Function to draw a hollow circle using OpenGL's legacy functions
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


int Simulation::run(int numOfCollision) {
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
    glViewport(400, 400, 800, 800);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1600, 1600.0, -1600, 1600.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int i = 0;
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        nextCollision(); // Process the next step of the simulation

        // Draw all elements
        i += 1;
        for (auto& ball : balls()) {
            drawCircle(ball.getPos().x(), ball.getPos().y(), ball.getRadius(), 20);
            drawHollowCircle(ball.getPos().x(), ball.getPos().y(), ball.getRadius(), 20);
        }
        drawHollowCircle(simContainer.getPos().x(), simContainer.getPos().y(),
                         simContainer.getRadius(), 80);

        glfwSwapBuffers(window);
        glfwPollEvents();

        if (i == numOfCollision) {
            break;
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
