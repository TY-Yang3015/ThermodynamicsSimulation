//
// Created by 杨天越 on 29/05/2024.
//


#include "Simulation.h"
#include "Ball.h"
#include "Container.h"
#include <random>
#include <algorithm>
#include <iostream>


Simulation::Simulation(
        const double &containerRadius, const double &ballRadius, const double &ballSpeed,
        const double &ballMass, const double &rMax, const int &nRings, const int &multi) :
        containerRadius(containerRadius), ballRadius(ballRadius),
        ballSpeed(ballSpeed), ballMass(ballMass),
        rMax(rMax), nRings(nRings), multi(multi)
        {
    Eigen::ArrayXXd positions = Simulation::positionInitialiser(rMax, nRings, multi);
    Eigen::ArrayXXd velocities = Simulation::velocityInitialiser(int(positions.rows()), ballSpeed);
    for (int i = 0; i < positions.rows(); i++){
        ballList.emplace_back(positions.row(i), velocities.row(i), ballRadius, ballMass);
    }

    simContainer = Container(containerRadius, 10000000.);
        }

Simulation::~Simulation() = default;

Eigen::ArrayXXd Simulation::positionInitialiser(double rMax, int nRings, int multi) {
    int numOfBalls = (multi + nRings * multi) * nRings / 2;
    Eigen::ArrayXXd positionArray(numOfBalls, 2);

    double radialIncrement = rMax / nRings;

    int k = 0;
    double angularIncrement, radius, theta;
    for (int i = 1; i <= nRings; i++) {
        angularIncrement = 2 * M_PI / (multi * i);
        radius = radialIncrement * i;

        for (int j = 0; j < multi * i; j++) {
            theta = angularIncrement * j;
            positionArray(k, 0) = radius * std::cos(theta);
            positionArray(k, 1) = radius * std::sin(theta);
            k++;
        }
    }
    return positionArray;
}

Eigen::ArrayXXd Simulation::velocityInitialiser (int numOfBalls, double speed) {
    Eigen::ArrayXXd velocityArray(numOfBalls, 2);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(0.0, 2*M_PI);

    std::vector<double> random_numbers;
    random_numbers.reserve(numOfBalls);
    for (int i = 0; i < numOfBalls; ++i) {
            random_numbers.push_back(distr(gen));
        }

    Eigen::ArrayXd angles = Eigen::Map<Eigen::VectorXd>(random_numbers.data(), int(random_numbers.size()));
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
        for (size_t i = j + 1; i < ballList.size(); ++i) {
            dtCurrent = ballList[j].timeToCollision(ballList[i]);

            if (dtCurrent < dtNext) {
                dtNext = dtCurrent;
                collisionBalls.clear();
                collisionBalls.emplace_back(&ballList[j], &ballList[i]);
            } else if ((dtCurrent == dtNext) && (dtCurrent != std::numeric_limits<double>::infinity())) {
                collisionBalls.emplace_back(&ballList[j], &ballList[i]);
            }
        }
    }

    std::vector<std::pair<Ball*, Ball*>> collisionWithContainer;
    for (auto & ball : ballList) {
        dtCurrent = ball.timeToCollision(simContainer);
        if (dtCurrent < dtNext) {
            collisionBalls.clear();
            collisionWithContainer.clear();
            dtNext = dtCurrent;
            collisionWithContainer.emplace_back(&ball, &simContainer);
        } else if ((dtCurrent == dtNext) && (dtCurrent != std::numeric_limits<double>::infinity())) {
            collisionWithContainer.emplace_back(&ball, &simContainer);
        }
    }

    for (auto & ball : ballList) {
        ball.Move(dtNext);
    }
    simContainer.Move(dtNext);

    if (!collisionBalls.empty()) {
        for (auto & collisionBall : collisionBalls) {
            collisionBall.first->collide(*collisionBall.second);
            numOfCollision += 1;
        }
    }
    if (!collisionWithContainer.empty()) {
        for (auto &containerCollision: collisionWithContainer) {
            containerCollision.second->collide(*containerCollision.first);
            numOfCollision += 1;
        }
    }

    currentTime += dtNext;
}

void Simulation::nextTimeStep(double frameTime) {
    double dtNext = frameTime;
    double timeCount = frameTime;
    std::vector<std::pair<Ball*, Ball*>> collisionBalls;
    double dtCurrent;


    while (timeCount > 0) {
        for (size_t j = 0; j < ballList.size(); ++j) {
            for (size_t i = j + 1; i < ballList.size(); ++i) {
                dtCurrent = ballList[j].timeToCollision(ballList[i]);

                if (dtCurrent < dtNext) {
                    dtNext = dtCurrent;
                    collisionBalls.clear();
                    collisionBalls.emplace_back(&ballList[j], &ballList[i]);
                } else if (dtCurrent == dtNext) {
                    collisionBalls.emplace_back(&ballList[j], &ballList[i]);
                }
            }
        }

        for (auto & ball : ballList) {
            dtCurrent = ball.timeToCollision(simContainer);
            if (dtCurrent < dtNext) {
                collisionBalls.clear();
                dtNext = dtCurrent;
                collisionBalls.emplace_back(&ball, &simContainer);
            } else if (dtCurrent == dtNext) {
                collisionBalls.emplace_back(&ball, &simContainer);
            }
        }

        if (!collisionBalls.empty()) {
            for (auto & collisionBall : collisionBalls) {
                for (auto & ball : ballList) {
                    ball.Move(dtNext);
                }
                simContainer.Move(dtNext);
                collisionBall.second->collide(*collisionBall.first);
                numOfCollision += 1;
            }

            timeCount -= dtNext;

        } else {
            for (auto & ball : ballList) {
                ball.Move(timeCount);
            }
            simContainer.Move(timeCount);

            break;
        }

        collisionBalls.clear();
        dtNext = timeCount;
    }

    currentTime += frameTime;
}

void drawCircle(double cx, double cy, double r, int num_segments) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2d(cx, cy);
    glColor3f(1.0f, 1.0f, 1.0f);
    for(int i = 0; i <= num_segments; i++) {
        double theta = 2. * M_PI * i / num_segments;
        double x = r * cos(theta);
        double y = r * sin(theta);
        glVertex2d(x + cx, y + cy);
    }
    glEnd();
}


void drawHollowCircle(double cx, double cy, double radius, int num_segments) {
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0f, 0.0f, 0.0f);
    for (int i = 0; i < num_segments; i++) {
        double theta = 2. * M_PI * i / num_segments;
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        glVertex2d(x + cx, y + cy);
    }
    glEnd();
}


int Simulation::runByCollision(int numOfCollision) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 800, "Colliding Balls Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glViewport(0, 0, 1600, 1600);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1600, 1600.0, -1600, 1600.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int i = 0;
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        nextCollision();

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



int Simulation::runByTime(double time, int frame) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 800, "Colliding Balls Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glViewport(400, 400, 800, 800);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1600, 1600.0, -1600, 1600.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int i = 0;
    double frameTime = time / frame;

    if (logSystemMacro) {
        writeMacroLogHeader();
    }

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        nextTimeStep(frameTime);

        if (logSystemMicro) {
            this->logSystemMicroInformation();
        }

        if (logSystemMacro) {
            this->logSystemMacroInformation();
        }

        this->logSystemChronology();

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



double Simulation::time() const {
    return this->currentTime;
}

double Simulation::pressure() {
    double momenta = abs(simContainer.dpTot()) / this->time();
    momenta /= simContainer.surfaceArea();

    return momenta;
}

vector<double> Simulation::speeds() {
    vector<double> speedList;
    speedList.reserve(ballList.size());
    for (const auto & ball : ballList) {
            speedList.emplace_back(ball.getVel().norm());
        }

    return speedList;

}

double Simulation::tEquipartition() {
    double kineticEnergy = 0.;
    for (auto& ball : ballList) {
        kineticEnergy += 0.5 * ball.getMass() * ball.getVel().dot(ball.getVel());
    }
    return kineticEnergy/double(ballList.size())/this->kb;
}

double Simulation::tIdeal() {
    double pV = this->pressure() * this->container().volume();
    double Nk = double(ballList.size()) * this->kb;

    return pV/Nk;
}

double Simulation::kineticEnergy() {
    double kineticEnergy = 0;
    for (auto& ball: ballList) {
        kineticEnergy += ball.getVel().dot(ball.getVel()) * ball.getMass() * 0.5;
    }

    kineticEnergy += simContainer.getVel().dot(simContainer.getVel()) * simContainer.getMass() * 0.5;

    return kineticEnergy;
}

Eigen::Vector2d Simulation::momentum() {
    Eigen::Vector2d momenta = Eigen::Vector2d::Zero();
    for (auto& ball: ballList) {
        momenta += ball.getVel() * ball.getMass();
    }

    momenta += simContainer.getVel() * simContainer.getMass();

    return momenta;
}

vector<Eigen::Vector2d> Simulation::ballVelocities() {
    vector<Eigen::Vector2d> velList;
    velList.reserve(ballList.size());
    for (const auto & ball : ballList) {
         velList.emplace_back(ball.getVel());
    }

    return velList;

}

vector<Eigen::Vector2d> Simulation::ballPositions() {
    vector<Eigen::Vector2d> posList;
    posList.reserve(ballList.size());
    for (const auto & ball : ballList) {
        posList.emplace_back(ball.getPos());
    }

    return posList;

}

void Simulation::setSystemLog(const bool &micro, const bool &macro) {
    this->logSystemMicro = micro;
    this->logSystemMacro = macro;
}

void Simulation::writeMacroLogHeader () {

    if (currentFolder.empty()) {
        cout << "you are advised to register the simulation spec. " << endl;
        this->currentFolder = createSimulationFolder();
    }

    vector<std::string> macroHeader;
    macroHeader.reserve(5);
    macroHeader.emplace_back("pressure");
    macroHeader.emplace_back("t_equipartition");
    macroHeader.emplace_back("t_ideal");
    macroHeader.emplace_back("total_momentum");
    macroHeader.emplace_back("total_ke");

    writeCSVHeader(currentFolder + "/macro_information.csv", macroHeader);
}

void Simulation::logSystemMacroInformation() {


    vector<double> MacroInformation;

    MacroInformation.reserve(5);
    MacroInformation.push_back(this->pressure());
    MacroInformation.push_back(this->tEquipartition());
    MacroInformation.push_back(this->tIdeal());
    MacroInformation.push_back(this->momentum().norm());
    MacroInformation.push_back(this->kineticEnergy());

    writeDoublesToCSV(currentFolder + "/macro_information.csv", MacroInformation);

}

void Simulation::logSystemMicroInformation() {

    if (currentFolder.empty()) {
        cout << "you are advised to register the simulation spec. " << endl;
        this->currentFolder = createSimulationFolder();
    }

    vector<Eigen::Vector2d> velocities = ballVelocities();
    vector<Eigen::Vector2d> positions = ballPositions();


    writeVectorsToCSV(currentFolder + "/ball_velocities.csv", velocities);
    writeVectorsToCSV(currentFolder + "/ball_positions.csv", positions);

    velocities.clear();
    velocities.push_back(container().getVel());
    positions.clear();
    positions.push_back(container().getPos());

    writeVectorsToCSV(currentFolder + "/container_velocities.csv", velocities);
    writeVectorsToCSV(currentFolder + "/container_positions.csv", positions);


}

void Simulation::logSystemChronology() {
    vector<double> chrono;
    chrono.push_back(time());
    chrono.push_back(numOfCollision);

    writeDoublesToCSV(currentFolder + "/system_event_track.csv", chrono);
}

void Simulation::recordSimulationSpecs() {
    vector<string> specHeaders;
    vector<double> specValues;

    currentFolder = createSimulationFolder();

    specHeaders.emplace_back("containerRadius");
    specValues.emplace_back(containerRadius);
    specHeaders.emplace_back("ballRadius");
    specValues.emplace_back(ballRadius);
    specHeaders.emplace_back("ballSpeed");
    specValues.emplace_back(ballSpeed);
    specHeaders.emplace_back("ballMass");
    specValues.emplace_back(ballMass);
    specHeaders.emplace_back("numOfBalls");
    specValues.emplace_back(double(ballList.size()));

    writeCSVHeader(currentFolder+"/vanilla_simulation_spec.csv", specHeaders);
    writeDoublesToCSV(currentFolder+"/vanilla_simulation_spec.csv", specValues);

    cout << "system spec registered." << endl;
}