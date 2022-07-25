#include <utility>

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"

class KalmanFilter {
public:
    KalmanFilter() : is_init(false) {}

    ~KalmanFilter() = default;

    void setF(Eigen::MatrixXd F_in)  {
        F_ = std::move(F_in);
    }

    void setP(Eigen::MatrixXd P_in) {
        P_ = std::move(P_in);
    }

    void setQ(Eigen::MatrixXd Q_in) {
        Q_ = std::move(Q_in);
    }

    void setH(Eigen::MatrixXd H_in) {
        H_ = std::move(H_in);
    }

    void setR(Eigen::MatrixXd R_in) {
        R_ = std::move(R_in);
    }

    void predictionUpdaete() {
        x_ = F_ * x_;
        Eigen::MatrixXd Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
    }

    void measurementUpdate(const Eigen::VectorXd &z) {
        Eigen::VectorXd y = z - H_ * x_;
    }

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "KalmanFilter_node");
    ros::NodeHandle nh;
    return 0;
}
