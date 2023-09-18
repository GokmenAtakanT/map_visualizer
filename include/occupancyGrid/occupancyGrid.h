//
// Created by deniz on 16.08.2023.
//

#ifndef SRC_OCCUPANCYGRID_H
#define SRC_OCCUPANCYGRID_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class OccupancyGridPublisher : public rclcpp::Node {
public:
    OccupancyGridPublisher() : Node("occupancy_grid_publisher") {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("carla/ego_vehicle/odometry",3, std::bind(&OccupancyGridPublisher::odomCallback, this, std::placeholders::_1));
        osm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/array", 10, std::bind(&OccupancyGridPublisher::float64MultiArrayCallback, this, std::placeholders::_1));
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&OccupancyGridPublisher::goalCallback, this, std::placeholders::_1));
        initial_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&OccupancyGridPublisher::initialCallback, this, std::placeholders::_1));

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid",10);
        obsx_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsx",10);
        obsy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsy",10);
        angle_ind_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("angle_ind",10);
        grid_size_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gridsize",10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
    }

private:
    void publishOccupancyGrid();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr t_goal_msg);

    void initialCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_init_msg);

    void float64MultiArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr t_msg);

    std::vector<int8_t> createMatrixWithMod(std::vector<int32_t>& int_vector_x, std::vector<int32_t>& int_vector_y, int& rows,
                        int& columns, int& min_x, int& min_y, std::vector<int32_t>& int_vector_angle, std::vector<double>& angle_data);

    double getDistance(int &x1,int &x2,int &y1,int &y2);


    double getDistance(const int x1, const int x2, const int y1, const int y2);


    std::tuple<std::vector<int32_t>, std::vector<int32_t>, std::vector<double>> linearInterpolation(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int num_points, double angle);

    std::pair<double, double> findClosestAngle(const geometry_msgs::msg::Pose &t_target_pose);

    int findClosestAngle(const double &t_target_angle);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr osm_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsx_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsy_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angle_ind_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grid_size_publisher_;
    std_msgs::msg::Float64MultiArray x_msg_;
    std_msgs::msg::Float64MultiArray y_msg_;
    std_msgs::msg::Float64MultiArray grid_msg_;
    std_msgs::msg::Float64MultiArray angle_msg_;
    geometry_msgs::msg::PoseStamped goal_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped init_msg_;
    bool flag1 =false;
    bool flag2 =false;
    int count_{0};
    int prev_count_{0};


    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    std::shared_ptr<nav_msgs::msg::Odometry> vehicle_odom_;

    std::vector<double> array_data_left_x;
    std::vector<double> array_data_left_y;
    std::vector<double> array_data_right_x;
    std::vector<double> array_data_right_y;
    std::vector<double> array_data_angle;
    bool first_{true};
    bool first{true};

    double roll_;
    double pitch_;
    double yaw_;
    std::vector<std::vector<double>> coordinates_ = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0},{-1, -1}, {0, -1}, {1, -1}};
    std::vector<double> angles_ = { 0, M_PI/4, M_PI/2, 3*M_PI/4, M_PI, -3*M_PI/4, -M_PI/2, -M_PI/4};
};




#endif //SRC_OCCUPANCYGRID_H
