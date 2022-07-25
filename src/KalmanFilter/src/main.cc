#include "KalmanFilter/kalman_filter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "KalmanFilter_node");
    ros::NodeHandle nh;

    KalmanFilter kf;
    return 0;
}
