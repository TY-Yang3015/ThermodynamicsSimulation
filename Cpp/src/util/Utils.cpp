//
// Created by 杨天越 on 31/05/2024.
//

#include "Utils.h"
#include "Eigen/Dense"
#include "GLFW/glfw3.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>
#include <iomanip>

void writeDoublesToCSV(const std::string& filename, const std::vector<double>& data) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "error: could not open file " << filename << std::endl;
        return;
    }

    if (file.tellp() != 0) {
        file << std::endl;
    }

    if (!data.empty()) {
        auto it = data.begin();
        file << *it;
        for (++it; it != data.end(); ++it) {
            file << "," << *it;
        }
    }

    file.close();
}

void writeCSVHeader(const std::string& filename, const std::vector<std::string>& header) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "error: could not open file " << filename << std::endl;
        return;
    }

    if (file.tellp() != 0) {
        file << std::endl;
    }

    if (!header.empty()) {
        auto it = header.begin();
        file << *it;
        for (++it; it != header.end(); ++it) {
            file << "," << *it;
        }
    }

    file.close();
}

void writeVectorsToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& data) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "error: could not open file " << filename << std::endl;
        return;
    }

    if (file.tellp() != 0) {
        file << std::endl;
    }

    if (!data.empty()) {
        for (auto it = data.begin(); it != data.end(); ++it) {
            if (it != data.begin()) {
                file << ",";
            }
            file << "[" << (*it)(0) << "," << (*it)(1) << "]";
        }
    }

    file.close();
}


std::string createSimulationFolder() {
    namespace fs = std::filesystem;

    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&time_t_now);

    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d_%H-%M-%S") << "Simulation";

    fs::path dir = fs::path("../bin/") / oss.str();

    if (!fs::exists(dir)) {
        // Create the directory
        if (fs::create_directory(dir)) {
            std::cout << "simulation log directory created: " << dir << std::endl;
        } else {
            std::cerr << "failed to create directory: " << dir << std::endl;
        }
    } else {
        std::cout << "directory already exists: " << dir << std::endl;
    }

    return dir.string();
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