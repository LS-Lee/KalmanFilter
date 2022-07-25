//
// Created by li on 22-7-25.
//

#ifndef KALMANFILTER_KALMAN_FILTER_H
#define KALMANFILTER_KALMAN_FILTER_H


#include "ros/ros.h"
#include "eigen3/Eigen/Dense"


class KalmanFilter {
public:
    KalmanFilter();
    ~KalmanFilter();
    void setF(Eigen::MatrixXd F_in);
    void setP(Eigen::MatrixXd P_in);
    void setQ(Eigen::MatrixXd Q_in);
    void setH(Eigen::MatrixXd H_in);
    void setR(Eigen::MatrixXd R_in);
    void predictionUpdaete();
    void measurementUpdate(const Eigen::VectorXd &z);

private:
    // flag of init
    bool is_init;

    // State vector
    Eigen::VectorXd x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};
#endif //KALMANFILTER_KALMAN_FILTER_H
