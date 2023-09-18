#include "../include/occupancyGrid/occupancyGrid.h"
#include "../include/osmVisualizer/dilation.h"

void OccupancyGridPublisher::publishOccupancyGrid()
{


    if((first_ && !array_data_right_y.empty()  && flag2) || prev_count_ != count_)
    {
        int min_x = std::min(*min_element(array_data_left_x.begin(), array_data_left_x.end()),
                            *min_element(array_data_right_x.begin(), array_data_right_x.end()));
        int max_x = std::max(*max_element(array_data_left_x.begin(), array_data_left_x.end()),
                            *max_element(array_data_right_x.begin(), array_data_right_x.end()));
        int min_y = std::min(*min_element(array_data_left_y.begin(), array_data_left_y.end()),
                            *min_element(array_data_right_y.begin(), array_data_right_y.end()));
        int max_y = std::max(*max_element(array_data_left_y.begin(), array_data_left_y.end()),
                            *max_element(array_data_right_y.begin(), array_data_right_y.end()));

        int width = max_x - min_x; // max_x - min_x
        int height = max_y - min_y; // max_y - min_y

        std::vector<int32_t> int_vector_x;
        std::vector<int32_t> int_vector_y;
        std::vector<int32_t> int_angle_ind;

        std::vector<double> angle_data(width * height, 8);

        for (size_t i = 0; i < array_data_left_x.size(); ++i)
        {
            // Calculate dist, interval, and num_points
            double dist = getDistance(array_data_left_x[i], array_data_right_x[i], array_data_left_y[i], array_data_right_y[i]);
            double interval = 0.1;
            int num_points = round(dist / interval);
            int x2 = array_data_right_x[i];
            int x1 = array_data_left_x[i];
            int y2 = array_data_right_y[i];
            int y1 = array_data_left_y[i];

            auto interpolated_points = linearInterpolation(x1, y1, x2, y2, num_points, array_data_angle[i]);
            int_vector_x.insert(int_vector_x.end(), std::get<0>(interpolated_points).begin(), std::get<0>(interpolated_points).end());
            int_vector_y.insert(int_vector_y.end(), std::get<1>(interpolated_points).begin(), std::get<1>(interpolated_points).end());
            int_angle_ind.insert(int_angle_ind.end(), std::get<2>(interpolated_points).begin(), std::get<2>(interpolated_points).end());

        }

        occupancy_grid_msg.header.frame_id = "map";  // Doldurmak istediğiniz frame_id'yi belirtin.
        occupancy_grid_msg.info.width = width;
        occupancy_grid_msg.info.height = height;
        occupancy_grid_msg.info.origin.position.x = min_x;
        occupancy_grid_msg.info.origin.position.y = min_y;
        occupancy_grid_msg.info.origin.position.z = 0.0;
        occupancy_grid_msg.info.resolution = 1.0;

        occupancy_grid_msg.data = createMatrixWithMod( int_vector_x , int_vector_y , width , height , min_x , min_y, int_angle_ind, angle_data);
        first_ = false;
        double x_i = vehicle_odom_->pose.pose.position.x - min_x;
        double y_i = vehicle_odom_->pose.pose.position.y - min_y;
        std::pair<double, double> closest_angle_ind_init = findClosestAngle(vehicle_odom_->pose.pose);
        std::pair<double, double> closest_angle_ind_goal = findClosestAngle(goal_msg_.pose);

        double x_e = goal_msg_.pose.position.x - min_x;
        double y_e = goal_msg_.pose.position.y - min_y;

        grid_msg_.data = { static_cast<double>(width), static_cast<double>(height), x_i, y_i, x_e, y_e, static_cast<double>(min_x), static_cast<double>(min_y), closest_angle_ind_init.second, closest_angle_ind_goal.second};

        prev_count_ = count_;

        for (const auto& point : angle_data)
        {
            angle_msg_.data.push_back(point);
        }
    }

    obsx_publisher_->publish(x_msg_);
    obsy_publisher_->publish(y_msg_);
    grid_size_publisher_->publish(grid_msg_);
    occupancy_grid_publisher_->publish(occupancy_grid_msg);
    angle_ind_publisher_->publish(angle_msg_);

}

void OccupancyGridPublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    vehicle_odom_ = msg;
}

void OccupancyGridPublisher::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr t_goal_msg) {
    goal_msg_ = *t_goal_msg;
    flag2=true;
    count_ ++;
}

void OccupancyGridPublisher::initialCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_init_msg) {
    init_msg_ = *t_init_msg;
    // flag1=true;
}

void OccupancyGridPublisher::float64MultiArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr t_msg)
{
    if(first)
    {
        for(size_t i = 0 ; i < t_msg->data.size()/5 ; i++)
        {
            array_data_left_x.push_back(round(t_msg->data[5*i]));
            array_data_left_y.push_back(round(t_msg->data[5*i+1]));
            array_data_right_x.push_back(round(t_msg->data[5*i+2]));
            array_data_right_y.push_back(round(t_msg->data[5*i+3]));
            array_data_angle.push_back(round(t_msg->data[5*i+4]));
        }
        first = false;
    }
}

std::vector<int8_t> OccupancyGridPublisher::createMatrixWithMod(std::vector<int32_t>& int_vector_x, std::vector<int32_t>& int_vector_y, int& rows, int& columns, int& min_x, int& min_y, std::vector<int32_t>& int_vector_angle, std::vector<double>& angle_data)
{
    std::cout << int_vector_y.size()<< std::endl;
    std::vector<int8_t> matrix_data(rows * columns, 100);

    std::vector<std::vector<int>> dil_matrix(rows, std::vector<int>(columns, 0));
    for (size_t i = 0; i < int_vector_x.size(); ++i)
    {
        int32_t x = int_vector_x[i] - min_x;
        int32_t y = int_vector_y[i] - min_y;

        if (x >= 0 && x < rows && y >= 0 && y < columns) {
            matrix_data[(y * rows) + x] = 0;
            angle_data[(y * rows) + x] = int_vector_angle[i];
        }
    }

    for( int i = 0 ; i < columns ; i++ )
    {
        for( int j = 0 ; j < rows ; j++ )
        {
            if(matrix_data[(i * rows) + j]==100)
            {
                x_msg_.data.push_back(j);
                y_msg_.data.push_back(i);
            }
            else
            {
                dil_matrix[j][i]=1;
            }
        }
    }
    dil_matrix=applyDilation(dil_matrix);

    std::vector<int8_t> dil ;
    for( int i = 0 ; i < columns ; i++ )
    {
        for( int j = 0 ; j < rows ; j++ )
        {
            if (dil_matrix[j][i] == 1)
            {
                dil.push_back(0);
            }
            else
            {
                dil.push_back(100);
            }
        }
    }
    return dil;
}

double OccupancyGridPublisher::getDistance(int &x1,int &x2,int &y1,int &y2)
{
    return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1 - y2, 2));
}

double OccupancyGridPublisher::getDistance(const int x1, const int x2, const int y1, const int y2)
{
    return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1 - y2, 2));
}

std::tuple<std::vector<int32_t>, std::vector<int32_t>, std::vector<double>> OccupancyGridPublisher::linearInterpolation(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int num_points, double angle)
{
std::vector<int32_t> int_vector_x;
std::vector<int32_t> int_vector_y;
std::vector<double> int_vector_angle;

int ind = findClosestAngle(angle);
for (int k = 0; k < num_points; ++k) {
    double t = static_cast<double>(k) / (num_points - 1);
    int32_t x = static_cast<int32_t>(round(x1 + t * (x2 - x1)));
    int32_t y = static_cast<int32_t>(round(y1 + t * (y2 - y1)));
    int_vector_x.push_back(x);
    int_vector_y.push_back(y);
    int_vector_angle.push_back(ind);
}

return std::make_tuple(int_vector_x, int_vector_y, int_vector_angle);
}

std::pair<double, double> OccupancyGridPublisher::findClosestAngle(const geometry_msgs::msg::Pose &t_target_pose)
{
    tf2::Quaternion quaternion =  tf2::Quaternion(t_target_pose.orientation.x,
                                                  t_target_pose.orientation.y,
                                                  t_target_pose.orientation.z,
                                                  t_target_pose.orientation.w);

    tf2::getEulerYPR(quaternion, yaw_, pitch_, roll_);

    double min_distance = std::numeric_limits<double>::max();
    double closest_angle = 0;
    double closest_index = 0;

    for (size_t i = 0; i < angles_.size(); ++i) {
        double distance = std::abs(yaw_ - angles_[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_angle = angles_[i];
            closest_index = i;
        }
    }
    return std::make_pair(closest_angle, closest_index);
}

int OccupancyGridPublisher::findClosestAngle(const double &t_target_angle)
{
    double min_distance = std::numeric_limits<double>::max();
    double closest_index = 0;

    for (size_t i = 0; i < angles_.size(); ++i) {
        double distance = std::abs(t_target_angle - angles_[i]);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_index = i;
        }
    }
    return closest_index;
}