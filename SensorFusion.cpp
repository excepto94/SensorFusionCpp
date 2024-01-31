#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 
#include "MotionModels.cpp"
#include "SensorModels.cpp"

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

void writeCsvFile(std::string fileName, int steps, double dT, MotionData dataToWrite, int numberOfStates) {
    std::ofstream file(fileName);
    if (file.is_open()) {
        file << "time,px,py,vx,vy,ax,ay,r,a\n";
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
            file << ",";
            file << dataToWrite.r[i];
            file << ",";
            file << dataToWrite.a[i];
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
        x = motionModel.f(x, 0, 0);

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
            
            //measurement_current << measurement.vx[i], measurement.vy[i];
            //measurement_current << measurement.vx[i], measurement.vy[i], measurement.ax[i], measurement.ay[i];
            measurement_current << measurement.r[i], measurement.a[i];

            // Prediction step
            x = motionModel.f(x,0,0);
            P = motionModel.F * P * motionModel.F.transpose() + motionModel.Q;
            // Update step
            Eigen::MatrixXd H = sensorModel.H(x);
            Eigen::VectorXd y = measurement_current - sensorModel.h(x);
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

int main() {
    int numberOfMeasurementTypes = 8;
    int stateSpaceSize = 2;

    Eigen::VectorXd x_init(stateSpaceSize); // State vector [px, py, vx, vy, ax, ay]

    double T = 0.001; // Time step
    double processNoise = 0.5;
    double measurementNoise = 1;
    Eigen::VectorXd sensorNoise(numberOfMeasurementTypes);
    sensorNoise << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.001;
    bool printEstimates = false;
    int steps = 10000;

    if (stateSpaceSize == 2) {
        x_init << 0, 0;
    }
    if (stateSpaceSize == 4) {
        x_init << 0, 0, 0, 0;
    }
    if (stateSpaceSize == 6) {
        x_init << 1, 1, 0, 0, 0, 0;
    }

    Eigen::MatrixXd P_init(stateSpaceSize, stateSpaceSize); // Covariance matrix
    for (int i = 0; i < stateSpaceSize; ++i) {
        for (int j = 0; j < stateSpaceSize; ++j) {
            P_init(i, j) = 0.01;
        }
    }

    MotionModel motionModel = cpmodel(T, processNoise);
    SensorModel sensorModel = RadarSensorModelCp(T, measurementNoise);

    //MotionModel motionModel = cvmodel(T, processNoise);
    //SensorModel sensorModel = velocitySensorModel(T, measurementNoise);

    //MotionModel motionModel = camodel(T, processNoise);
    //SensorModel sensorModel = VASensorModel(T, measurementNoise)
    //SensorModel sensorModel = RadarSensorModelCa(T, measurementNoise);

    MotionData trueVelocity = generateMotionData(steps, T);
    writeCsvFile("dataGroundTruth.csv", steps, T, trueVelocity, stateSpaceSize);
    MotionData noisyVelocity = generateMeasurement(steps, trueVelocity, sensorNoise);
    writeCsvFile("dataMeasured.csv", steps, T, noisyVelocity, stateSpaceSize);
    KalmanFilter kalmanFilter = KalmanFilter(motionModel, sensorModel, noisyVelocity, noisyVelocity);
    kalmanFilter.estimate(noisyVelocity, x_init, P_init, steps, printEstimates);
    MotionData estimatedVelocity = kalmanFilter.estimatedData;
    writeCsvFile("dataEstimated.csv", steps, T, estimatedVelocity, stateSpaceSize);

    return 0;
}
