#pragma once

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 

#include <MotionData.hpp>

void writeCsvFile(std::string fileName, int steps, double dT, MotionData dataToWrite, int numberOfStates);