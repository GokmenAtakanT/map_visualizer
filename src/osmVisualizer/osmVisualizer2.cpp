#include "osmVisualizer/osmVisualizer.h"

OsmVisualizer::OsmVisualizer() : Node("OsmVisualizer")
{
    this->declare_parameter("map_path", "/home/atakan/vuran_ws/src/vuran_simulation/mapping/map_visualizer/osm/Town10.osm");
    this->declare_parameter("enable_inc_path_points", true);
    this->declare_parameter("interval", 2.0);
    if (!readParameters())
        rclcpp::shutdown();

    goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&OsmVisualizer::goalCallback, this, std::placeholders::_1));
    initial_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&OsmVisualizer::initialCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map", 10);
    array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/array", 10);
    max_min_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/max_min_values", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);

    timer_ = this->create_wall_timer( 500ms, std::bind(&OsmVisualizer::timer_callback, this));

    map_ = lanelet::load(map_path_, lanelet::projection::UtmProjector(lanelet::Origin({15, 30})));

    for (auto point : map_->pointLayer)
    {
        point.x() = point.attribute("local_x").asDouble().value();
        point.y() = point.attribute("local_y").asDouble().value();
    }

    fill_marker(map_);
    fill_array_with_left_right(map_);
    writeToFile(m_array);

    lanelet::traffic_rules::TrafficRulesPtr trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    double laneChangeCost = 1;
    lanelet::routing::RoutingCostPtrs costPtrs{std::make_shared<lanelet::routing::RoutingCostDistance>(laneChangeCost)};
    lanelet::routing::RoutingGraph::Configuration routingGraphConf;

    routingGraphConf.emplace(std::make_pair(lanelet::routing::RoutingGraph::ParticipantHeight, lanelet::Attribute("2.")));
    routingGraph_ = lanelet::routing::RoutingGraph::build(*map_, *trafficRules);
}

bool OsmVisualizer::readParameters()
{
    if (!this->get_parameter("map_path", map_path_))
    {
        std::cout << "Failed to read parameter 'map_path' " << '\n';
        return false;
    }
    if (!this->get_parameter("enable_inc_path_points", enable_inc_path_points_))
    {
        std::cout << "Failed to read parameter 'interval' to increase the path points" << '\n';
        return false;
    }
    if (!this->get_parameter("interval", interval_))
    {
        std::cout << "Failed to read parameter 'interval' to increase the path points" << '\n';
        return false;
    }
    return true;
}

void OsmVisualizer::initialCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_init_msg) {
    init_msg_ = *t_init_msg;
    x_i_ = init_msg_.pose.pose.position.x;
    y_i_ = init_msg_.pose.pose.position.y;
    flag1=true;
}

void OsmVisualizer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr t_goal_msg) {
    goal_msg_ = *t_goal_msg;
    x_d_ = goal_msg_.pose.position.x;
    y_d_ = goal_msg_.pose.position.y;
    flag2=true;
    count_ ++;
}



void OsmVisualizer::timer_callback(){

    if(flag2 && flag1){
        std::cout<<"xd:"<<x_d_<<std::endl;
        std::cout<<"yd:"<<y_d_<<std::endl;

        std::cout<<"xi:"<<x_i_<<std::endl;
        std::cout<<"yi:"<<y_i_<<std::endl;

        lanelet::BasicPoint2d fromPoint = {x_i_, y_i_};
        lanelet::BasicPoint2d toPoint = {x_d_, y_d_};

        // Find the nearest lanelets
        //auto fromCandidates = lanelet::geometry::findNearest(map_->laneletLayer, fromPoint, 1);
        //auto toCandidates = lanelet::geometry::findNearest(map_->laneletLayer, toPoint, 1);

        auto fromCandidates = lanelet::geometry::findNearest(map_->laneletLayer, fromPoint, 100); // Increase the limit to 5 or an appropriate value.
        auto toCandidates = lanelet::geometry::findNearest(map_->laneletLayer, toPoint, 100);

        if (fromCandidates.empty() || toCandidates.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find nearest lanelets.");
            // Handle the error appropriately
        } else {
            lanelet::ConstLanelet fromLanelet = fromCandidates[0].second;
            lanelet::ConstLanelet toLanelet = toCandidates[0].second;

            std::cout << "fromLanelet id: " << fromLanelet.id() << std::endl;
            std::cout << "toLanelet id: " << toLanelet.id() << std::endl;
            // Continue with finding the shortest path
            lanelet::Optional<lanelet::routing::LaneletPath> shortestPath = routingGraph_->shortestPath(fromLanelet, toLanelet);

            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";

            if (shortestPath) {
                // Path found, do something with it
                int spath_size=0;
                for (const auto& lanelet : *shortestPath) {
                    for (const auto& point : lanelet.centerline2d()) {
                        geometry_msgs::msg::PoseStamped pose_stamped;
                        pose_stamped.header.stamp = this->now();
                        pose_stamped.header.frame_id = "map"; // Assuming waypoints are in the "map" frame

                        pose_stamped.pose.position.x = point.x();
                        pose_stamped.pose.position.y = point.y();
                        pose_stamped.pose.orientation.w = 1.0;

                        path_msg.poses.push_back(pose_stamped);
                        spath_size++;
                    }
                }
                std::cout<<spath_size<<std::endl;
                // Publish the path
                path_publisher_->publish(path_msg);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to find a valid path.");
                // Handle the error appropriately
            }
        }
    }

    publisher_->publish(m_marker_array);
    array_publisher_->publish(m_array);
}

void OsmVisualizer::fill_array(lanelet::LaneletMapPtr &t_map){
    // ROS_msg intialize
    // Create multi array message for Rviz.
    m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    m_array.layout.dim[0].label = "rows";
    m_array.layout.dim[0].size = 100000;
    m_array.layout.dim[0].stride = 100000*2;
    m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    m_array.layout.dim[1].label = "cols";
    m_array.layout.dim[1].size = 2;
    m_array.layout.dim[1].stride = 2;

    // For filling center line
    for (const auto &line2line : t_map->laneletLayer){
        for(size_t i = 0; i < line2line.centerline2d().size()-1; i++){

            if(getDistance(line2line,i) > 2 && enable_inc_path_points_){
                double dist = getDistance(line2line,i);
                int num_points = dist;

                for(int k = 0 ; k<num_points;k++){
                    m_array.data.push_back(((line2line.centerline2d()[i+1].x()-line2line.centerline2d()[i].x()) / num_points) * k + line2line.centerline2d()[i].x());
                    m_array.data.push_back(((line2line.centerline2d()[i+1].y()-line2line.centerline2d()[i].y()) / num_points) * k + line2line.centerline2d()[i].y());
                }
            }
            else{
                m_array.data.push_back(line2line.centerline2d()[i].x());
                m_array.data.push_back(line2line.centerline2d()[i].y());
            }
        }
    }
}


void OsmVisualizer::writeToFile(const std_msgs::msg::Float64MultiArray& multi_array)
{
    std::ofstream file("MapOccupationData.txt");
    if (file.is_open())
    {
        for (size_t i = 0; i < multi_array.data.size(); ++i)
        {
            file << multi_array.data[i] << ",";
            if ((i + 1) % (multi_array.layout.dim[0].size) == 0)
                file << "\n";
            if ((i + 1) % (multi_array.layout.dim[1].size) == 0)
                file << "\n";
        }
        file.close();
    }
}

void OsmVisualizer::fill_array_with_left_right(lanelet::LaneletMapPtr &t_map)
{
    // ROS_msg intialize
    // Create multi array message for Rviz.
    m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    m_array.layout.dim[0].label = "rows";
    m_array.layout.dim[0].size = t_map->laneletLayer.size();
    m_array.layout.dim[0].stride = t_map->laneletLayer.size()*4;
    m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    m_array.layout.dim[1].label = "cols";
    m_array.layout.dim[1].size = 4;
    m_array.layout.dim[1].stride = 4;

    for (const auto &line2line : t_map->laneletLayer)
    {
        std::vector<lanelet::ConstLineString3d> bounds;
        bounds.push_back(line2line.leftBound());
        bounds.push_back(line2line.rightBound());

        size_t size = (bounds[0].size() < bounds[1].size()) ? bounds[0].size() : bounds[1].size();
        for(size_t i = 0; i < size-1 ; i++)
        {
            m_array.data.push_back(bounds[0][i].x());
            m_array.data.push_back(bounds[0][i].y());
            m_array.data.push_back(bounds[1][i].x());
            m_array.data.push_back(bounds[1][i].y());
            m_array.data.push_back(atan2((bounds[1][i+1].y() - bounds[1][i].y()),(bounds[1][i+1].x() - bounds[1][i].x()))); // [0 180] & [-180 0]
        }
    }
}


double OsmVisualizer::getDistance(const lanelet::ConstLanelet &line2line , size_t i)
{
    return std::sqrt(std::pow(line2line.centerline2d()[i].x() - line2line.centerline2d()[i+1].x(),2)+std::pow(line2line.centerline2d()[i].y()-line2line.centerline2d()[i+1].y(),2));
}

void OsmVisualizer::fill_marker(lanelet::LaneletMapPtr &t_map)
{
    size_t i =  0;

    for (const auto &line2line : t_map->laneletLayer)
    {
        // arrows for directions For Visualizing
        lanelet::ConstLineString3d center_line = line2line.centerline3d();
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock{}.now();
        marker.ns = "lanelet";
        marker.id = i;
        i++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 0.8;
        marker.color.r = 0.7;
        marker.color.g = 1.0;
        marker.color.b = 0.8;
        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;
        p1.x = center_line[center_line.size()/2 - 2].x();
        p1.y = center_line[center_line.size()/2 - 2].y();
        p1.z = 0;
        p2.x = center_line[center_line.size()/2 + 2].x();
        p2.y = center_line[center_line.size()/2 + 2].y();
        p2.z = 0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        m_marker_array.markers.push_back(marker);

        // lane For Visualizing
        std::vector<lanelet::ConstLineString3d> bounds;
        bounds.push_back(line2line.leftBound());
        bounds.push_back(line2line.rightBound());

        for (const auto &bound : bounds)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = rclcpp::Clock{}.now();
            marker.ns = "lanelet";
            marker.id = i;
            i++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 232;
            marker.color.g = 44;
            marker.color.b = 44;
            for (const auto &point : bound)
            {
                geometry_msgs::msg::Point p;
                p.x = point.x();
                p.y = point.y();
                p.z = 0;
                marker.points.push_back(p);
            }
            m_marker_array.markers.push_back(marker);
        }
    }
}
