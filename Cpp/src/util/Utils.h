//
// Created by 杨天越 on 31/05/2024.
//

#pragma once

#include <fstream>
#include "Eigen/Dense"
#include <vector>
#include <iostream>

// Function to write doubles to a CSV file, each on a new line
void writeDoublesToCSV(const std::string& filename, const std::vector<double>& data);
void writeCSVHeader(const std::string& filename, const std::vector<std::string>& header);
void writeVectorsToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& data);
std::string createSimulationFolder();