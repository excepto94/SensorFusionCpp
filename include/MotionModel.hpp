#pragma once

#include <Eigen/Dense>
#include <functional>

struct MotionModel {
    int d;
    Eigen::MatrixXd F;
    Eigen::MatrixXd Q;
    std::function<Eigen::VectorXd(Eigen::VectorXd)> f;

    MotionModel(int dimension);
};

MotionModel cpmodel(double T, double sigma);

MotionModel cvmodel(double T, double sigma);

MotionModel camodel(double T, double sigma);
