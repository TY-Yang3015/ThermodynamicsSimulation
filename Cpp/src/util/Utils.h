//
// Created by 杨天越 on 31/05/2024.
//

#pragma once

#include <fstream>
#include "Eigen/Dense"
#include <vector>
#include <iostream>

void writeDoublesToCSV(const std::string& filename, const std::vector<double>& data);
void writeCSVHeader(const std::string& filename, const std::vector<std::string>& header);
void writeVectorsToCSV(const std::string& filename, const std::vector<Eigen::Vector2d>& data);
std::string createSimulationFolder();
void drawCircle(double cx, double cy, double r, int num_segments);
void drawHollowCircle(double cx, double cy, double radius, int num_segments);