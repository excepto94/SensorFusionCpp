#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 

#include <MotionData.hpp>
#include <SensorModel.hpp>
#include <KalmanFilter.hpp>
#include <MotionModel.hpp>
#include <MiscFunctions.hpp>

int main() {
    int numberOfMeasurementTypes = 8;
    int stateSpaceSize = 2;

    Eigen::VectorXd x_init(stateSpaceSize); // State vector [px, py, vx, vy, ax, ay]

    double T = 0.001; // Time step
    double processNoise = 0.5;
    double measurementNoise = 1;
    Eigen::VectorXd sensorNoise(numberOfMeasurementTypes);
    sensorNoise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.001;
    bool printEstimates = false;
    int steps = 10000;

    if (stateSpaceSize == 2) {
        x_init << 0, 0;
    }
    if (stateSpaceSize == 4) {
        x_init << 0, 0, 0, 0;
    }
    if (stateSpaceSize == 6) {
        x_init << 1, 1, 0, 0, 0, 0;
    }

    Eigen::MatrixXd P_init(stateSpaceSize, stateSpaceSize); // Covariance matrix
    for (int i = 0; i < stateSpaceSize; ++i) {
        for (int j = 0; j < stateSpaceSize; ++j) {
            P_init(i, j) = 0.01;
        }
    }

    MotionModel motionModel = cpmodel(T, processNoise);
    SensorModel sensorModel = RadarSensorModelCp(T, measurementNoise);

    //MotionModel motionModel = cvmodel(T, processNoise);
    //SensorModel sensorModel = velocitySensorModel(T, measurementNoise);

    //MotionModel motionModel = camodel(T, processNoise);
    //SensorModel sensorModel = VASensorModel(T, measurementNoise)
    //SensorModel sensorModel = RadarSensorModelCa(T, measurementNoise);

    MotionData trueVelocity = generateMotionData(steps, T);
    writeCsvFile("dataGroundTruth.csv", steps, T, trueVelocity, stateSpaceSize);
    MotionData noisyVelocity = generateMeasurement(steps, trueVelocity, sensorNoise);
    writeCsvFile("dataMeasured.csv", steps, T, noisyVelocity, stateSpaceSize);
    KalmanFilter kalmanFilter = KalmanFilter(motionModel, sensorModel, noisyVelocity, noisyVelocity);
    kalmanFilter.estimate(noisyVelocity, x_init, P_init, steps, printEstimates);
    MotionData estimatedVelocity = kalmanFilter.estimatedData;
    writeCsvFile("dataEstimated.csv", steps, T, estimatedVelocity, stateSpaceSize);

    return 0;
}