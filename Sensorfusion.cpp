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
};

class MotionData {
    private:
        static const int MAXSIZE = 4095;
    protected:
        int lengthOfData;
    public:
        int t[MAXSIZE];
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
            velocity_x[i] = 0.0;
            velocity_y[i] = 0.0;
            acceleration_x[i] = 0.0;
            acceleration_y[i] = 0.0;
        }
    }
};

MotionModel cvmodel(double T, double sigma) {
    int d = 4;

    Eigen::MatrixXd F(4, 4);
    F << 1, 0, T, 0,
        0, 1, 0, T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    auto f = [F](Eigen::VectorXd x) -> Eigen::VectorXd {
        return F * x;
        };

    double T4 = T * T * T * T / 3;
    double T3 = T * T * T / 2;
    double T2 = T * T / 1;

    Eigen::MatrixXd Q(4, 4);
    Q << T4, 0,  T3, 0,
         0,  T4, 0,  T3,
         T3, 0,  T2, 0,
         0,  T3, 0,  T2;

    return { d, F, Q, f };
}

Eigen::VectorXd calculateX(const Eigen::VectorXd& x, double T) {
    Eigen::VectorXd newX = x; // Copy the state vector

    newX(0) += T * x(2); // Update position_x
    newX(1) += T * x(3); // Update position_y

    return newX;
}

Eigen::MatrixXd calculateP(const Eigen::MatrixXd& P, double T) {
    Eigen::MatrixXd newP = P; // Copy the covariance matrix

    newP(0, 0) += T * (P(2, 2) + P(0, 2)) + T * P(0, 2);
    newP(1, 1) += T * (P(3, 3) + P(1, 3)) + T * P(1, 3);
    newP(0, 1) += T * P(2, 3);
    newP(1, 0) += T * P(3, 2);

    newP(2, 0) += T * P(2, 2);
    newP(3, 1) += T * P(3, 3);

    newP(0, 2) += T * P(2, 2);
    newP(1, 3) += T * P(3, 3);

    newP(2, 0) += T * P(2, 2);
    newP(3, 1) += T * P(3, 3);

    newP(2, 2) += T * T;
    newP(3, 3) += T * T;

    return newP;
}

Eigen::MatrixXd generateRandomMeasurement(const Eigen::MatrixXd& trueVelocity, double noise) {
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

Eigen::MatrixXd generateMotionData(int steps, double T, double noise) {
    // Define CV model matrices
    Eigen::MatrixXd F(4, 4); // State transition matrix
    F << 1, 0, T, 0,
        0, 1, 0, T,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::VectorXd x(4); // State vector [position_x, position_y, velocity_x, velocity_y]
    x << 0, 0, 1, 1;

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4);

    Eigen::MatrixXd data(4, steps);

    for (int i = 0; i < steps; ++i) {
        // Prediction step (simulate motion)
        x = F * x;
        // Add process noise to the state
        for (int j = 0; j < x.size(); ++j) {
            x(j) += noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        }

        data.col(i) = x;
    }

    return data;
}

void kalmanFilter() {
    Eigen::VectorXd x(4); // State vector [position_x, position_y, velocity_x, velocity_y]
    x << 0, 0, 1, 1;

    Eigen::MatrixXd P(4, 4); // Covariance matrix
    P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::MatrixXd R(2, 2); // Measurement noise covariance matrix R (velocity)
    R << 0.1, 0,
        0, 0.1;

    double T = 0.1; // Time step

    double measurementNoise = 0.05; // Adjust this value based on the noise characteristics of your sensor

    int steps = 30;

    Eigen::MatrixXd trueVelocity = generateMotionData(steps, T, 0);
    Eigen::MatrixXd noisyVelocity = generateRandomMeasurement(trueVelocity, measurementNoise);

    std::ofstream file("generated_data.csv");
    if (file.is_open()) {
        file << "time,position_x,position_y,velocity_x,velocity_y\n";
        double currentT = 0;
        for (int i = 0; i < steps; ++i) {
            file << currentT;
            file << ",";
            for (int j = 0; j < 4; ++j) {
                file << noisyVelocity(j, i);
                if (j < 3) {
                    file << ",";
                }
            }
            currentT += T;
            file << "\n";
        }
        file.close();
        std::cout << "Generated data saved to 'generated_data.csv'\n";
    }
    else {
        std::cerr << "Unable to open file\n";
    }



    /*
    for (int i = 0; i < 5; ++i) { // Loop for a few iterations
        // Prediction Step (Time Update)
        x = calculateX(x, T);
        P = calculateP(P, T);

        // Generate simulated noisy velocity measurements


        // Measurement Update Step
        Eigen::VectorXd z = x.segment(2, 2) + Eigen::VectorXd::Random(2) * measurementNoise; // Simulated noisy measurement of velocity

        Eigen::MatrixXd H(2, 4); // Measurement matrix for velocity
        H << 0, 0, 1, 0,
             0, 0, 0, 1;

        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * (z - H * x);
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
    }
    */


}

int main() {
    double T = 0.01;
    double sigma = 0.1;

    MotionModel cv = cvmodel(T, sigma);

    // Example vector x for testing matrix-vector multiplication
    Eigen::VectorXd x(4);
    x << 1.0, 2.0, 3.0, 4.0;

    // Calculate F * x using the defined function f
    Eigen::VectorXd result = cv.f(x);

    // Display the result
    std::cout << "Result of F * x: " << result.transpose() << std::endl;

    kalmanFilter(); // Call your Kalman Filter function here

    return 0;
}
