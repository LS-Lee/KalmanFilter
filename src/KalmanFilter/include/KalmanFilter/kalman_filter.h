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

    Eigen::VectorXd getX();
    void initSystem(Eigen::VectorXd &x_in);
    void setF(Eigen::MatrixXd F_in);
    void setP(Eigen::MatrixXd P_in);
    void setQ(Eigen::MatrixXd Q_in);
    void setH(Eigen::MatrixXd H_in);
    void setR(Eigen::MatrixXd R_in);
    void predictionUpdaete();
    void measurementUpdate(const Eigen::VectorXd &z);

private:

    bool is_init;       // flag of init

    // State vector
    Eigen::VectorXd x_; // state vector
    Eigen::MatrixXd F_; // state transistion matrix
    Eigen::MatrixXd P_; // state covariance matrix
    Eigen::MatrixXd Q_; // process covariance matrix
    Eigen::MatrixXd H_; // measure matrix
    Eigen::MatrixXd R_; // measure covariance matrix
};
#endif //KALMANFILTER_KALMAN_FILTER_H
