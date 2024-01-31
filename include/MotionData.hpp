#ifndef MOTION_DATA_HPP
#define MOTION_DATA_HPP

#include <Eigen/Dense>

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

MotionData fillDataFromCaModel(MotionData estimatedVelocity, Eigen::VectorXd x, int i);

MotionData generateMeasurement(int steps, MotionData unnoisyMeasurement, Eigen::VectorXd noise);

MotionData generateMotionData(int steps, double T);

#endif // MOTION_DATA_HPP