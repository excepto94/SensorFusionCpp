#include <Eigen/Dense>
#include <MotionData.hpp>
#include <SensorModel.hpp>

SensorModel velocitySensorModel(double T, double sigma) {
    int d1 = 6;
    int d2 = 2;
    SensorModel sensorModel(d1, d2);

    sensorModel.H = [sensorModel](Eigen::VectorXd x) -> Eigen::MatrixXd {
        Eigen::MatrixXd H;
        H << 
            0, 0, 1, 0, 0, 0, 
            0, 0, 0, 1, 0, 0;
        return H;
    };

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {
        return sensorModel.H(x) * x;
    };
    
    sensorModel.getRelevantMeasurements = [sensorModel](MotionData measurement, int i) -> Eigen::VectorXd {
        Eigen::VectorXd relevantMeasurements(sensorModel.d2);
        relevantMeasurements << measurement.vx[i], measurement.vy[i];
        return relevantMeasurements;
    };

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

    sensorModel.H = [sensorModel](Eigen::VectorXd x) -> Eigen::MatrixXd {
        Eigen::MatrixXd H(4,6);
        H <<
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        return H;
    };

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {
        Eigen::MatrixXd H(4,6);
        H <<
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        return H * x;
    };

    sensorModel.getRelevantMeasurements = [sensorModel](MotionData measurement, int i) -> Eigen::VectorXd {
        Eigen::VectorXd relevantMeasurements(sensorModel.d2);
        relevantMeasurements << measurement.vx[i], measurement.vy[i], measurement.ax[i], measurement.ay[i];
        return relevantMeasurements;
    };

    double sigma2 = sigma*sigma;
    sensorModel.R <<
        sigma2, 0, 0, 0,
        0, sigma2, 0, 0,
        0, 0, sigma2, 0,
        0, 0, 0, sigma2;

    return sensorModel;
}

SensorModel RadarSensorModelCp(double T, double sigma) {
    int d1 = 2;
    int d2 = 2;
    SensorModel sensorModel(d2, d2);

    sensorModel.H = [sensorModel](Eigen::VectorXd x) -> Eigen::MatrixXd {
        Eigen::MatrixXd H(2, 2);
        double px = x[0];
        double py = x[1];
        double rS = (px * px + py * py);
        rS = rS > 0.001 ? rS : 0.001;
        H <<
            px/rS, py/rS,
            -py/rS, px/rS;
        return H;
    };

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {
        Eigen::VectorXd z(2);
        double px = x[0];
        double py = x[1];
        z <<
            sqrt(px * px + py * py),
            atan2(py, px);
        return z;
    };

    sensorModel.getRelevantMeasurements = [sensorModel](MotionData measurement, int i) -> Eigen::VectorXd {
        Eigen::VectorXd relevantMeasurements(sensorModel.d2);
        relevantMeasurements << measurement.r[i], measurement.a[i];
        return relevantMeasurements;
    };

    double sigma2 = sigma*sigma;
    sensorModel.R <<
        sigma2, 0,
        0, sigma2;

    return sensorModel;
}

SensorModel RadarSensorModelCa(double T, double sigma) {
    int d1 = 6;
    int d2 = 2;
    SensorModel sensorModel(d2, d2);

    sensorModel.H = [sensorModel](Eigen::VectorXd x) -> Eigen::MatrixXd {
        Eigen::MatrixXd H(2, 6);
        double px = x[0];
        double py = x[1];
        double rS = (px * px + py * py);
        rS = rS > 0.001 ? rS : 0.001;
        H <<
            px/rS, py/rS, 0, 0, 0, 0,
            -py/rS, px/rS, 0, 0, 0, 0;
        return H;
    };

    sensorModel.h = [sensorModel](Eigen::VectorXd x) -> Eigen::VectorXd {
        Eigen::VectorXd z(2);
        double px = x[0];
        double py = x[1];
        z <<
            sqrt(px * px + py * py),
            atan2(py, px);
        return z;
    };

    sensorModel.getRelevantMeasurements = [sensorModel](MotionData measurement, int i) -> Eigen::VectorXd {
        Eigen::VectorXd relevantMeasurements(sensorModel.d2);
        relevantMeasurements << measurement.r[i], measurement.a[i];
        return relevantMeasurements;
    };

    double sigma2 = sigma*sigma;
    sensorModel.R <<
        sigma2, 0,
        0, sigma2;

    return sensorModel;
}