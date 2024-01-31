#include <Eigen/Dense>
#include "../include/MotionModel.hpp"

struct MotionData {
protected:
    int lengthOfData;

public:
    Eigen::VectorXi t;
    Eigen::VectorXd px;
    Eigen::VectorXd py;
    Eigen::VectorXd vx;
    Eigen::VectorXd vy;
    Eigen::VectorXd ax;
    Eigen::VectorXd ay;
    Eigen::VectorXd r;
    Eigen::VectorXd a;

    MotionData(int n) :
        lengthOfData(n),
        t(n),
        px(n),
        py(n),
        vx(n),
        vy(n),
        ax(n),
        ay(n),
        r(n),
        a(n)
    {}
};

MotionData fillDataFromCaModel(MotionData estimatedVelocity, Eigen::VectorXd x, int i) {
    estimatedVelocity.px[i] = x[0];
    estimatedVelocity.py[i] = x[1];
    if (x.size() > 2) {
        estimatedVelocity.vx[i] = x[2];
        estimatedVelocity.vy[i] = x[3];
    }
    if (x.size() > 4) {
        estimatedVelocity.ax[i] = x[4];
        estimatedVelocity.ay[i] = x[5];
    }
    return estimatedVelocity;
}

MotionData generateMeasurement(int steps, MotionData unnoisyMeasurement, Eigen::VectorXd noise) {
    MotionData noisyMeasurement = unnoisyMeasurement; // Initialize with true velocity
    for (int i = 0; i < steps; ++i) {
        noisyMeasurement.px[i] = unnoisyMeasurement.px[i] + noise[0] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.py[i] = unnoisyMeasurement.py[i] + noise[1] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.vx[i] = unnoisyMeasurement.vx[i] + noise[2] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.vy[i] = unnoisyMeasurement.vy[i] + noise[3] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.ax[i] = unnoisyMeasurement.ax[i] + noise[4] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.ay[i] = unnoisyMeasurement.ay[i] + noise[5] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.r[i] = unnoisyMeasurement.r[i] + noise[6] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.a[i] = unnoisyMeasurement.a[i] + noise[7] * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
    }
    return noisyMeasurement;
}

MotionData generateMotionData(int steps, double T) {
    int stateSpaceSize = 6;
    MotionModel motionModel = camodel(T, 1);
    MotionData motionData = MotionData(steps);
    Eigen::VectorXd x(stateSpaceSize);
    x << -5, 5, 1, 0, 0, 0;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(stateSpaceSize, stateSpaceSize);

    Eigen::MatrixXd data(stateSpaceSize, steps);

    for (int i = 0; i < steps; ++i) {
        // Prediction step (simulate motion)
        x = motionModel.f(x);

        motionData.px[i] = x[0];
        motionData.py[i] = x[1];
        motionData.vx[i] = x[2];
        motionData.vy[i] = x[3];
        motionData.ax[i] = x[4];
        motionData.ay[i] = x[5];
        motionData.r[i] = sqrt(motionData.px[i] * motionData.px[i] + motionData.py[i] + motionData.py[i]);
        motionData.a[i] = atan2(motionData.py[i], motionData.px[i]);
    }

    return motionData;
}