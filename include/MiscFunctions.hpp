#ifndef MISC_FUNCTIONS_HPP
#define MISC_FUNCTIONS_HPP

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 

#include "../include/MotionData.hpp"

void writeCsvFile(std::string fileName, int steps, double dT, MotionData dataToWrite, int numberOfStates);

#endif // MISC_FUNCTIONS_HPP
