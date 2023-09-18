//
// Created by deniz on 16.08.2023.
//

#include "../include/occupancyGrid/occupancyGrid.h"

bool interrupted = false;

// Signal handler function for Ctrl+C
void signalHandler(int signum)
{
    // Set the interrupted flag to true
    interrupted = true;
    rclcpp::shutdown();

}

// Main function
int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create an instance of the PathPlanning class
    auto node = std::make_shared<OccupancyGridPublisher>();

    // Set up the signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Spin the node until interrupted
    if (!interrupted && rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    // Shutdown ROS

    return 0;
}