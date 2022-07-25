cmake_minimum_required(VERSION 3.0.2)
project(KalmanFilter)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES KalmanFilter
        #  CATKIN_DEPENDS roscpp rospy
        #  DEPENDS system_lib
)

include_directories(
        include
        /usr/include
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/main.cc src/kalman_filter.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )