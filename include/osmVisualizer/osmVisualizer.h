
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace std::chrono_literals;

class OsmVisualizer : public rclcpp::Node
{
  public:
    OsmVisualizer();

  private:
    void timer_callback();
    bool readParameters();
    void writeToFile(const std_msgs::msg::Float64MultiArray& multi_array);
    void fill_marker(lanelet::LaneletMapPtr &t_map);
    void fill_array(lanelet::LaneletMapPtr &t_map);
    void fill_array_with_left_right(lanelet::LaneletMapPtr &t_map);
    void fill_min_max_values(const lanelet::Lanelet &line);
    double getDistance(const lanelet::ConstLanelet &line2line , size_t i);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr t_goal_msg);

    void initialCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_init_msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr max_min_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std_msgs::msg::Float64MultiArray m_array;
    visualization_msgs::msg::MarkerArray m_marker_array;

    //params
    std::string map_path_; 
    bool enable_inc_path_points_;
    double interval_;
    bool flag1 =false;
    bool flag2 =false;
    int count_{0};
    int prev_count_{0};
    lanelet::LaneletMapPtr map_;
    geometry_msgs::msg::PoseStamped goal_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped init_msg_;
    std::shared_ptr<nav_msgs::msg::Path> path_msg = std::make_shared<nav_msgs::msg::Path>();
    lanelet::routing::RoutingGraphUPtr routingGraph_;
    double x_d_;
    double y_d_;
    double x_i_;
    double y_i_;
};