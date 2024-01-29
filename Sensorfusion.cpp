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

    MotionData(int n) :
        lengthOfData(n),
        t(n),
        px(n),
        py(n),
        vx(n),
        vy(n),
        ax(n),
        ay(n) 
    {}
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

MotionData fillDataFromCvModel(MotionData estimatedVelocity, Eigen::VectorXd x, int i) {
    estimatedVelocity.px[i] = x[0];
    estimatedVelocity.py[i] = x[1];
    estimatedVelocity.vx[i] = x[2];
    estimatedVelocity.vy[i] = x[3];
    estimatedVelocity.ax[i] = x[4];
    estimatedVelocity.ay[i] = x[5];
    return estimatedVelocity;
}

SensorModel velocitySensorModel(double T, double sigma) {
    int d1 = 6;
    int d2 = 2;
    SensorModel sensorModel(d1, d2);

    sensorModel.H <<
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {return sensorModel.H * x;};

    double sigma2 = sigma*sigma;
    sensorModel.R << 
        sigma2, 0,
        0, sigma2;

    return sensorModel;
}

SensorModel VASensorModel(double T, double sigma) {
    int d1 = 6;
    int d2 = 4;
    SensorModel sensorModel(d1, d2);

    sensorModel.H <<
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {return sensorModel.H * x;};

    double sigma2 = sigma*sigma;
    sensorModel.R << 
        sigma2, 0, 0, 0,
        0, sigma2, 0, 0,
        0, 0, sigma2, 0,
        0, 0, 0, sigma2;

    return sensorModel;
}

MotionData generateMeasurement(int steps, MotionData unnoisyMeasurement, double noise) {
    MotionData noisyMeasurement = unnoisyMeasurement; // Initialize with true velocity
    for (int i = 0; i < steps; ++i) {
        noisyMeasurement.px[i] = unnoisyMeasurement.px[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.py[i] = unnoisyMeasurement.py[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.vx[i] = unnoisyMeasurement.vx[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.vy[i] = unnoisyMeasurement.vy[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.ax[i] = unnoisyMeasurement.ax[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
        noisyMeasurement.ay[i] = unnoisyMeasurement.ay[i] + noise * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
    }
    return noisyMeasurement;
}

void writeCsvFile(std::string fileName, int steps, double dT, MotionData dataToWrite, int numberOfStates) {
    std::ofstream file(fileName);
    if (file.is_open()) {
        file << "time,px,py,vx,vy,ax,ay\n";
        double T = 0;
        for (int i = 0; i < steps; ++i) {
            file << T;
            file << ",";
            file << dataToWrite.px[i];
            file << ",";
            file << dataToWrite.py[i];
            file << ",";
            file << dataToWrite.vx[i];
            file << ",";
            file << dataToWrite.vy[i];
            file << ",";
            file << dataToWrite.ax[i];
            file << ",";
            file << dataToWrite.ay[i];
            file << "\n";
            T += dT;
        }
        file.close();
        std::cout << "Generated data saved to '" << fileName << "'\n";
    }
    else {
        std::cerr << "Unable to open file\n";
    }
}

MotionData generateMotionData(int stateSpaceSize, int steps, double T, double noise) {
    MotionModel motionModel = camodel(T, 1);
    MotionData motionData = MotionData(steps);
    Eigen::VectorXd x(stateSpaceSize);

    if (stateSpaceSize == 4) {
        x << 0, 0, 1, 1;
    }
    if (stateSpaceSize == 6) {
        x << 0, 0, 1, 1, 1, 1;
    }

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(stateSpaceSize, stateSpaceSize);
    Eigen::MatrixXd data(stateSpaceSize, steps);

    for (int i = 0; i < steps; ++i) {
        // Prediction step (simulate motion)
        x = motionModel.f(x, 0, 0);

        motionData.px[i] = x[0];
        motionData.py[i] = x[1];
        motionData.vx[i] = x[2];
        motionData.vy[i] = x[3];
        if (stateSpaceSize == 6) {
            motionData.ax[i] = x[4];
            motionData.ay[i] = x[5];
        }
    }

    return motionData;
}

struct KalmanFilter {
    MotionModel motionModel;
    SensorModel sensorModel;
    MotionData motionData;
    MotionData estimatedData;

    KalmanFilter(const MotionModel& mm, const SensorModel& sm, const MotionData& md, const MotionData& ed)
        : motionModel(mm), sensorModel(sm), motionData(md), estimatedData(ed) {
        }

    void estimate(MotionData measurement, Eigen::VectorXd x_init, Eigen::MatrixXd P_init, int steps, bool printEstimates) {
        Eigen::VectorXd measurement_current(sensorModel.d2);
        Eigen::VectorXd x = x_init;
        Eigen::MatrixXd P = P_init;

        for (int i = 0; i < steps; ++i) {
            if (sensorModel.d2 == 2) {
                measurement_current << measurement.vx[i], measurement.vy[i];
            }
            if (sensorModel.d2 == 4) {
                measurement_current << measurement.vx[i], measurement.vy[i], measurement.ax[i], measurement.ay[i];
            }
            // Prediction step
            x = motionModel.f(x,0,0);
            P = motionModel.F * P * motionModel.F.transpose() + motionModel.Q;
            // Update step
            Eigen::VectorXd y = measurement_current - sensorModel.h(x);
            Eigen::MatrixXd S = sensorModel.H * P * sensorModel.H.transpose() + sensorModel.R;
            Eigen::MatrixXd K = P * sensorModel.H.transpose() * S.inverse();
            x = x + K * y;
            P = (Eigen::MatrixXd::Identity(motionModel.d, motionModel.d) - K * sensorModel.H) * P;
            estimatedData = fillDataFromCvModel(estimatedData, x, i);
            if (printEstimates) {
                std::cout
                    << estimatedData.px[i] << ", "
                    << estimatedData.py[i] << ", "
                    << estimatedData.vx[i] << ", "
                    << estimatedData.vy[i] << ", "
                    << estimatedData.ax[i] << ", "
                    << estimatedData.ay[i] << std::endl;
            }
        }
    }
};

int main() {
    int stateSpaceSize = 6;

    Eigen::VectorXd x_init(stateSpaceSize); // State vector [px, py, vx, vy, ax, ay]
    

    double T = 0.01; // Time step

    double processNoise = 0.1;
    double measurementNoise = 0.05;
    double sensorNoise = 0.1;
    bool printEstimates = true;
    int steps = 1000;



    if (stateSpaceSize == 4) {
        x_init << 0, 0, 0, 0;
    }
    if (stateSpaceSize == 6) {
        x_init << 0, 0, 0, 0, 0, 0;
    }
    
    Eigen::MatrixXd P_init(stateSpaceSize, stateSpaceSize); // Covariance matrix
    for (int i = 0; i < stateSpaceSize; ++i) {
        for (int j = 0; j < stateSpaceSize; ++j) {
            P_init(i, j) = 0.001;
        }
    }

    MotionModel motionModel = camodel(T, processNoise);
    
    //SensorModel sensorModel = velocitySensorModel(T, measurementNoise);
    SensorModel sensorModel = VASensorModel(T, measurementNoise);

    MotionData trueVelocity = generateMotionData(stateSpaceSize, steps, T, 0);
    writeCsvFile("dataGroundTruth.csv", steps, T, trueVelocity, stateSpaceSize);
    MotionData noisyVelocity = generateMeasurement(steps, trueVelocity, sensorNoise);
    writeCsvFile("dataMeasured.csv", steps, T, noisyVelocity, stateSpaceSize);
    KalmanFilter kalmanFilter = KalmanFilter(motionModel, sensorModel, noisyVelocity, noisyVelocity);
    kalmanFilter.estimate(noisyVelocity, x_init, P_init, steps, printEstimates);
    MotionData estimatedVelocity = kalmanFilter.estimatedData;
    writeCsvFile("dataEstimated.csv", steps, T, estimatedVelocity, stateSpaceSize);

    return 0;
}