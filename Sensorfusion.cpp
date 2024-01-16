#include <iostream>
#include <cmath>
#include <vector>
#include <functional>
#include <cstdlib> // Include for rand() function
#include <Eigen/Dense>
#include <fstream> // Include for file handling

struct MotionModel {
    int d;
    Eigen::MatrixXd F;
    Eigen::MatrixXd Q;
    std::function<Eigen::VectorXd(Eigen::VectorXd)> f;

    // Constructor to set dimensions and initialize matrices
    MotionModel(int dimension) : d(dimension), F(dimension, dimension), Q(dimension, dimension) {
        // Ensure that f has the correct size
        f = [this](Eigen::VectorXd x) {
            // Check if the input size matches d
            assert(x.size() == this->d && "Input size mismatch in function f");

            // Placeholder implementation of f, you can replace it with your desired function
            Eigen::VectorXd result = F * x;
            return result;
        };
    }
};

struct SensorModel {
    int d1;
    int d2;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    std::function<Eigen::VectorXd(Eigen::VectorXd)> h;

    // Constructor to set dimensions and initialize matrices
    SensorModel(int dimension1, int dimension2) : d1(dimension1), d2(dimension2), H(dimension2, dimension1), R(dimension2, dimension2) {
        // Ensure that h has the correct size
        h = [this](Eigen::VectorXd x) {
            // Check if the input size matches d2
            assert(x.size() == this->d2 && "Input size mismatch in function h");

            // Placeholder implementation of h, you can replace it with your desired function
            Eigen::VectorXd result = H * x;
            return result;
        };
    }
};

class MotionData {
    private:
        static const int MAXSIZE = 4096;
    protected:
        int lengthOfData;
    public:
        int t[MAXSIZE];
        double position_x[MAXSIZE];
        double position_y[MAXSIZE];
        double velocity_x[MAXSIZE];
        double velocity_y[MAXSIZE];
        double acceleration_x[MAXSIZE];
        double acceleration_y[MAXSIZE];

    MotionData(int n) {
        lengthOfData = n;
        if (n > MAXSIZE) {
            std::cerr << "Error: Size exceeds the maximum length." << std::endl;
            n = MAXSIZE;
        }

        for (int i = 0; i < n; ++i) {
            t[i] = 0;
            position_x[i] = 0.0;
            position_y[i] = 0.0;
            velocity_x[i] = 0.0;
            velocity_y[i] = 0.0;
            acceleration_x[i] = 0.0;
            acceleration_y[i] = 0.0;
        }
    }
};

MotionModel cvmodel(double T, double sigma) {
    int d = 4;
    
    MotionModel motionModel(d);
    motionModel.d = d;

    motionModel.F <<
        1, 0, T, 0,
        0, 1, 0, T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    motionModel.f = [motionModel](Eigen::VectorXd x) -> Eigen::VectorXd {return motionModel.F * x;};

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

SensorModel velocitySensorModel(double T, double sigma) {
    int d1 = 4;
    int d2 = 2;
    SensorModel sensorModel(d1, d2);

    sensorModel.H <<
        0, 0, 1, 0,
        0, 0, 0, 1;

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {return sensorModel.H * x;};

    double sigma2 = sigma*sigma;
    sensorModel.R << 
        sigma2, 0,
        0, sigma2;

    return sensorModel;
}

Eigen::MatrixXd generateMeasurement(const Eigen::MatrixXd& trueVelocity, double noise) {
    Eigen::MatrixXd noisyVelocity = trueVelocity; // Initialize with true velocity

    for (int i = 0; i < trueVelocity.cols(); ++i) {
        for (int j = 0; j < trueVelocity.rows(); ++j) {
            double randomNoise = static_cast<double>(rand()) / RAND_MAX; // Random noise between 0 and 1
            double noisyVal = trueVelocity(j, i) + noise * (2.0 * randomNoise - 1.0); // Adding noise to true velocity element
            noisyVelocity(j, i) = noisyVal; // Update the noisy velocity
        }
    }

    return noisyVelocity;
}

Eigen::MatrixXd runKalmanFilter(MotionModel motionModel, SensorModel sensorModel, Eigen::MatrixXd noisyVelocity, Eigen::VectorXd x, Eigen::MatrixXd P, int steps) {
    Eigen::MatrixXd estimatedVelocity = noisyVelocity;
    Eigen::VectorXd measurement(sensorModel.d2);

    for (int i = 0; i < steps; ++i) { // Loop for a few iterations
        
        measurement << noisyVelocity(2,i), noisyVelocity(3,i);

        // Prediction step
        x = motionModel.f(x);
        P = motionModel.F * P * motionModel.F.transpose() + motionModel.Q;

        // Update step
        Eigen::VectorXd y = measurement - sensorModel.h(x);
        Eigen::MatrixXd S = sensorModel.H * P * sensorModel.H.transpose() + sensorModel.R;
        Eigen::MatrixXd K = P * sensorModel.H.transpose() * S.inverse();

        x = x + K * y;
        P = (Eigen::MatrixXd::Identity(motionModel.d, motionModel.d) - K * sensorModel.H) * P;

        for (int j = 0; j < motionModel.d; j++) {
            estimatedVelocity(j, i) = x(j);
        }
        std::cout
            << estimatedVelocity(0,i) << " , "
            << estimatedVelocity(1,i) << " , "
            << estimatedVelocity(2,i) << " , "
            << estimatedVelocity(3,i) << std::endl;
    }
    
    return estimatedVelocity;
}

void writeCsvFile(std::string fileName, int steps, double T, Eigen::MatrixXd velocityData) {
    std::ofstream file(fileName);
    if (file.is_open()) {
        file << "time,position_x,position_y,velocity_x,velocity_y\n";
        double currentT = 0;
        for (int i = 0; i < steps; ++i) {
            file << currentT;
            file << ",";
            for (int j = 0; j < 4; ++j) {
                file << velocityData(j, i);
                if (j < 3) {
                    file << ",";
                }
            }
            currentT += T;
            file << "\n";
        }
        file.close();
        std::cout << "Generated data saved to '" << fileName << "'\n";
    }
    else {
        std::cerr << "Unable to open file\n";
    }
}

Eigen::MatrixXd generateMotionData(int steps, double T, double noise) {
    // Define CV model matrices
    MotionModel motionModel = cvmodel(T, 1);

    Eigen::VectorXd x(4); // State vector [position_x, position_y, velocity_x, velocity_y]
    x << 0, 0, 1, 1;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd data(4, steps);

    for (int i = 0; i < steps; ++i) {
        // Prediction step (simulate motion)
        x = motionModel.f(x);
        // Add process noise to the state
        for (int j = 0; j < x.size(); ++j) {
            x(j) += noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        }
        data.col(i) = x;
    }
    return data;
}

int main() {
    Eigen::VectorXd x_init(4); // State vector [position_x, position_y, velocity_x, velocity_y]
    x_init << 0, 0, 1, 1;

    Eigen::MatrixXd P_init(4, 4); // Covariance matrix
    P_init <<
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

    double T = 0.01; // Time step

    double processNoise = 0.05; // Adjust this value based on the noise characteristics of your sensor
    double measurementNoise = 0.05; // Adjust this value based on the noise characteristics of your sensor

    int steps = 1000;

    MotionModel motionModel = cvmodel(T, processNoise);
    SensorModel sensorModel = velocitySensorModel(T, measurementNoise);

    Eigen::MatrixXd trueVelocity = generateMotionData(steps, T, 0);
    Eigen::MatrixXd noisyVelocity = generateMeasurement(trueVelocity, measurementNoise);
    Eigen::MatrixXd estimatedVelcity = runKalmanFilter(motionModel, sensorModel, noisyVelocity, x_init, P_init, steps);

    writeCsvFile("dataGroundTruth.csv", steps, T, trueVelocity);
    writeCsvFile("dataMeasured.csv", steps, T, noisyVelocity);
    writeCsvFile("dataEstimated.csv", steps, T, estimatedVelcity);


    return 0;
}