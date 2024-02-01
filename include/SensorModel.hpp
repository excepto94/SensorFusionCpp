#pragma once

#include <Eigen/Dense>
#include <functional>
#include <MotionData.hpp>

struct SensorModel {
    int d1;
    int d2;
    std::function<Eigen::MatrixXd(Eigen::VectorXd)> H;
    Eigen::MatrixXd R;
    std::function<Eigen::VectorXd(Eigen::VectorXd)> h;
    std::function<Eigen::VectorXd(MotionData, int)> getRelevantMeasurements;

    // Constructor to set dimensions and initialize matrices
    SensorModel(int dimension1, int dimension2) : d1(dimension1), d2(dimension2), R(dimension2, dimension2) {

        H = [this](Eigen::VectorXd x) {
            assert(x.size() == this->d2 && "Input size mismatch in function H");
            Eigen::MatrixXd result;
            return result;
        };

        h = [this](Eigen::VectorXd x) {
            assert(x.size() == this->d2 && "Input size mismatch in function h");
            Eigen::VectorXd result;
            return result;
        };

        getRelevantMeasurements = [this](MotionData measurement, int i) {
            Eigen::VectorXd result(d2);
            return result;
        };
    }
};

SensorModel velocitySensorModel(double T, double sigma);

SensorModel VASensorModel(double T, double sigma);

SensorModel RadarSensorModelCp(double T, double sigma);

SensorModel RadarSensorModelCa(double T, double sigma);