#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "MotionModel.hpp"
#include "SensorModel.hpp"
#include "MotionData.hpp"
#include <iostream>

struct KalmanFilter {
    MotionModel motionModel;
    SensorModel sensorModel;
    MotionData motionData;
    MotionData estimatedData;

    KalmanFilter(const MotionModel& mm, const SensorModel& sm, const MotionData& md, const MotionData& ed);

    void estimate(MotionData measurement, Eigen::VectorXd x, Eigen::MatrixXd P, int steps, bool printEstimates);
};

#endif // KALMAN_FILTER_HPP
