#include <Eigen/Dense>
#include <functional>

struct MotionModel {
    int d;
    Eigen::MatrixXd F;
    Eigen::MatrixXd Q;
    std::function<Eigen::VectorXd(Eigen::VectorXd, double, double)> f;

    // Constructor to set dimensions and initialize matrices
    MotionModel(int dimension) : d(dimension), F(dimension, dimension), Q(dimension, dimension) {
        // Ensure that f has the correct size
        f = [this](Eigen::VectorXd x, double arg1 = 0.0, double arg2 = 0.0) {
            // Check if the input size matches d
            assert(x.size() == this->d && "Input size mismatch in function f");

            // Placeholder implementation of f, you can replace it with your desired function
            Eigen::VectorXd result = F * x;

            return result;
        };
    }
};

MotionModel cpmodel(double T, double sigma) {
    int d = 2;

    MotionModel motionModel(d);
    motionModel.d = d;

    motionModel.F <<
        1, 0,
        0, 1;

    motionModel.f = [motionModel](Eigen::VectorXd x, double arg1 = 0, double arg2 = 0) {Eigen::VectorXd result = motionModel.F * x; return result;};

    double T2 = T * T;

    motionModel.Q <<
        T2, 0,
        0,  T2;
    motionModel.Q = motionModel.Q * sigma;

    return motionModel;
}

MotionModel cvmodel(double T, double sigma) {
    int d = 4;

    MotionModel motionModel(d);
    motionModel.d = d;

    motionModel.F <<
        1, 0, T, 0,
        0, 1, 0, T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    motionModel.f = [motionModel](Eigen::VectorXd x, double arg1 = 0, double arg2 = 0) {Eigen::VectorXd result = motionModel.F * x; return result;};

    double T4 = T * T * T * T / 4;
    double T3 = T * T * T / 2;
    double T2 = T * T;

    motionModel.Q <<
        T4, 0,  T3, 0,
        0,  T4, 0,  T3,
        T3, 0,  T2, 0,
        0,  T3, 0,  T2;
    motionModel.Q = motionModel.Q * sigma;

    return motionModel;
}

MotionModel camodel(double T, double sigma) {
    int d = 6;

    MotionModel motionModel(d);
    motionModel.d = d;

    double T4 = T * T * T * T / 4;
    double T3 = T * T * T / 2;
    double T2 = T * T;

    motionModel.F <<
        1, 0, T, 0, 0.5 * T2, 0,
        0, 1, 0, T, 0, 0.5 * T2,
        0, 0, 1, 0, T, 0,
        0, 0, 0, 1, 0, T,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    motionModel.f = [motionModel](Eigen::VectorXd x, double arg1 = 0, double arg2 = 0) {Eigen::VectorXd result = motionModel.F * x; return result;};

    motionModel.Q <<
        T4, 0,  T3, 0, 0.5 * T2, 0,
        0,  T4, 0,  T3, 0, 0.5 * T2,
        T3, 0,  T2, 0, T, 0,
        0,  T3, 0,  T2, 0, T,
        0.5 * T2, 0, T, 0, T2, 0,
        0, 0.5 * T2, 0, T, 0, T2;

    motionModel.Q = motionModel.Q * sigma;

    return motionModel;
}