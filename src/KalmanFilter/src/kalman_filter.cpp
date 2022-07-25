//
// Created by li on 22-7-25.
//

#include "KalmanFilter/kalman_filter.h"

KalmanFilter::KalmanFilter() : is_init(false) {}

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::setF(Eigen::MatrixXd F_in)  {
    F_ = std::move(F_in);
}

void KalmanFilter::setP(Eigen::MatrixXd P_in) {
    P_ = std::move(P_in);
}

void KalmanFilter::setQ(Eigen::MatrixXd Q_in) {
    Q_ = std::move(Q_in);
}

void KalmanFilter::setH(Eigen::MatrixXd H_in) {
    H_ = std::move(H_in);
}

void KalmanFilter::setR(Eigen::MatrixXd R_in) {
    R_ = std::move(R_in);
}

void KalmanFilter::predictionUpdaete() {
    x_ = F_ * x_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::measurementUpdate(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H_ * x_;
}
