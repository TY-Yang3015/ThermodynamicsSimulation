//
// Created by 杨天越 on 31/05/2024.
//

#include "Utils.h"
#include "Eigen/Dense"
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>
#include <iomanip>

void writeDoublesToCSV(const std::string& filename, const std::vector<double>& data) {
    // Open file in append mode
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Check if the file is empty to determine if a newline is needed
    if (file.tellp() != 0) { // File pointer is not at the start, hence file is not empty
        file << std::endl;
    }

    // Check if data is not empty to avoid trailing comma issues
    if (!data.empty()) {
        auto it = data.begin();
        // Write the first element without a leading comma
        file << *it;
        // Write the rest of the elements with a leading comma
        for (++it; it != data.end(); ++it) {
            file << "," << *it;
        }
    }

    file.close();
}

void writeCSVHeader(const std::string& filename, const std::vector<std::string>& header) {
    // Open file in append mode
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Check if the file is empty to determine if a newline is needed
    if (file.tellp() != 0) { // File pointer is not at the start, hence file is not empty
        file << std::endl;
    }

    // Check if header is not empty to avoid trailing comma issues
    if (!header.empty()) {
        auto it = header.begin();
        // Write the first element without a leading comma
        file << *it;
        // Write the rest of the elements with a leading comma
        for (++it; it != header.end(); ++it) {
            file << "," << *it;
        }
    }

    file.close();
}

void writeVectorsToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& data) {
    // Open file in append mode
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // Check if the file is empty to determine if a newline is needed
    if (file.tellp() != 0) { // File pointer is not at the start, hence file is not empty
        file << std::endl;
    }

    // Check if data is not empty to avoid trailing comma issues
    if (!data.empty()) {
        for (auto it = data.begin(); it != data.end(); ++it) {
            // Write each vector's components wrapped in brackets and separated by a comma
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

    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm bt = *std::localtime(&time_t_now);

    // Create a time string from the current time
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d_%H-%M-%S") << "Simulation";

    // Construct the directory path
    fs::path dir = fs::path("../bin/") / oss.str();

    // Check if the directory exists
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


