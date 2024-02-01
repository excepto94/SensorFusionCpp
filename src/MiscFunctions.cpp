#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 

#include <MotionData.hpp>

void writeCsvFile(std::string fileName, int steps, double dT, MotionData dataToWrite, int numberOfStates) {
    std::ofstream file(fileName);
    if (file.is_open()) {
        file << "time,px,py,vx,vy,ax,ay,r,a\n";
        double T = 0;
        for (int i = 0; i < steps; ++i) {
            file << T;
            file << ",";
            file << dataToWrite.px[i];
            file << ",";
            file << dataToWrite.py[i];
            file << ",";
            file << dataToWrite.vx[i];
            file << ",";
            file << dataToWrite.vy[i];
            file << ",";
            file << dataToWrite.ax[i];
            file << ",";
            file << dataToWrite.ay[i];
            file << ",";
            file << dataToWrite.r[i];
            file << ",";
            file << dataToWrite.a[i];
            file << "\n";
            T += dT;
        }
        file.close();
        std::cout << "Generated data saved to '" << fileName << "'\n";
    }
    else {
        std::cerr << "Unable to open file\n";
    }
}

