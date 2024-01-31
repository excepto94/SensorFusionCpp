#include "../include/MotionModel.hpp"
#include "../include/SensorModel.hpp"
#include "../include/MotionData.hpp"
#include <iostream>

struct KalmanFilter {
    MotionModel motionModel;
    SensorModel sensorModel;
    MotionData motionData;
    MotionData estimatedData;

    KalmanFilter(const MotionModel& mm, const SensorModel& sm, const MotionData& md, const MotionData& ed)
        : motionModel(mm), sensorModel(sm), motionData(md), estimatedData(ed) {
        }

    void estimate(MotionData measurement, Eigen::VectorXd x, Eigen::MatrixXd P, int steps, bool printEstimates) {

        for (int i = 0; i < steps; ++i) {
            // Prediction step
            x = motionModel.f(x);
            P = motionModel.F * P * motionModel.F.transpose() + motionModel.Q;
            // Update step
            Eigen::MatrixXd H = sensorModel.H(x);
            Eigen::VectorXd currentMeasurement = sensorModel.getRelevantMeasurements(measurement, i);
            Eigen::VectorXd y = currentMeasurement - sensorModel.h(x);
            Eigen::MatrixXd S = H * P * H.transpose() + sensorModel.R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            x = x + K * y;
            P = (Eigen::MatrixXd::Identity(motionModel.d, motionModel.d) - K * H) * P;
            estimatedData = fillDataFromCaModel(estimatedData, x, i);
            if (printEstimates) {
                std::cout
                    << estimatedData.px[i] << ", "
                    << estimatedData.py[i] << ", "
                    << std::endl;
                /*
                std::cout
                    << estimatedData.px[i] << ", "
                    << estimatedData.py[i] << ", "
                    << estimatedData.vx[i] << ", "
                    << estimatedData.vy[i] << ", "
                    << estimatedData.ax[i] << ", "
                    << estimatedData.ay[i] << ", "
                    << estimatedData.r[i] << ", "
                    << estimatedData.a[i] << std::endl;
                    */
            }
        }
    }
};
