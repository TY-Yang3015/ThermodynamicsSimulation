//
// Created by 杨天越 on 29/05/2024.
//

#include "Simulation.h"
#include "Ball.h"
#include "Container.h"
#include "../util/Utils.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>

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

    for (size_t i = 0; i < this->balls().size(); ++i) {
        if (&ball != &ballList[i]) {
            r = ballList[i].getPos() - currentPos;


            force += (6 / (ball.getRadius() * 2))
                     * pow(((2 * ball.getRadius()) / r.norm()), 7)
                     * (r / r.norm());

            force -= (12 / (ball.getRadius() * 2))
                     * pow(((2 * ball.getRadius()) / r.norm()), 13)
                     * (r / r.norm());

            force *= 4 * phi0;

        }
    }

    return force;
}

void Simulation::nextTimeStep(double frameTime) {

    for (auto & j : ballList) {
        if (j.getPos().norm() > simContainer.getRadius() - j.getRadius()) {
            //Eigen::Vector2d vr = (ballList[j].getPos()/ballList[j].getPos().norm()).dot(ballList[j].getVel())
            //                     * (ballList[j].getPos()/ballList[j].getPos().norm());
            //ballList[j].setVel(ballList[j].getVel() - 2.*vr);
            j.collide(simContainer);
            j.setPos(j.getPos().normalized() * (simContainer.getRadius() - j.getRadius()));

        }
    }
    this->verletGlobalUpdate(frameTime);

    currentTime += frameTime;
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

    if (logSystemMacro) {
        writeMacroLogHeader();
    }

    int i = 0;
    double frameTime = time / frame;
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        nextTimeStep(frameTime); // Process the next step of the simulation

        if (logSystemMicro) {
            this->logSystemMicroInformation();
        }

        if (logSystemMacro) {
            this->logSystemMacroInformation();
        }

        this->logSystemChronology();

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
    macroHeader.emplace_back("total_momentum_x");
    macroHeader.emplace_back("total_momentum_y");
    macroHeader.emplace_back("total_ke");

    writeCSVHeader(currentFolder + "/macro_information.csv", macroHeader);
}

void Simulation::logSystemMacroInformation() {


    vector<double> MacroInformation;

    MacroInformation.reserve(6);
    MacroInformation.push_back(this->pressure());
    MacroInformation.push_back(this->tEquipartition());
    MacroInformation.push_back(this->tIdeal());
    Eigen::Vector2d totMom = momentum();
    MacroInformation.push_back(totMom.x());
    MacroInformation.push_back(totMom.y());
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
    specHeaders.emplace_back("attractionStrength");
    specValues.emplace_back(phi0);

    writeCSVHeader(currentFolder+"/lj_simulation_spec.csv", specHeaders);
    writeDoublesToCSV(currentFolder+"/lj_simulation_spec.csv", specValues);

    cout << "system spec registered." << endl;
}