#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include "visualization_msgs/msg/marker_array.hpp"
#include <dynamic_msgs/msg/dynamic_obstacles.hpp>
#include "obstacle_kf.hpp"

#include <cmath>  // isinf, sqrt
#include <string>
#include <vector>


class DynamicObstacleDetector : public rclcpp::Node {
public: 
    DynamicObstacleDetector() : Node("dynamic_obstacle_detector") {
        this->declare_parameter("input_scan_topic", std::string("/scan"));
        this->declare_parameter("odom_frame", std::string("odom"));
        this->declare_parameter("scan_buffer_size", 10);
        this->declare_parameter("thres_point_dist", 0.1);
        this->declare_parameter("thresh_min_points", 10);
        this->declare_parameter("thresh_max_points", 100);
        this->declare_parameter("min_vel_tracked", 0.2);
        this->declare_parameter("max_vel_tracked", 1.0);
        this->declare_parameter("track_distance", 5.0);
        this->declare_parameter("track_timeout", 5.0);

        this->get_parameter("input_scan_topic", input_scan_topic_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("scan_buffer_size", scan_buffer_size_);
        this->get_parameter("thres_point_dist", thres_point_dist_);
        this->get_parameter("thresh_min_points", thresh_min_points_);
        this->get_parameter("thresh_max_points", thresh_max_points_);
        this->get_parameter("min_vel_tracked", min_vel_tracked_);
        this->get_parameter("max_vel_tracked", max_vel_tracked_);
        this->get_parameter("track_distance", track_distance_);
        this->get_parameter("track_timeout", track_timeout_);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstacleDetector::scanCallback, this, std::placeholders::_1));
      

        obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
        dyn_obs_pub_ = this->create_publisher<dynamic_msgs::msg::DynamicObstacles>("dynamic_obstacles", 10);
        points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
        obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", 10);

        obstacle_count_ = 0;
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        buffer_->setCreateTimerInterface(timer_interface);
    }

    
    struct Point {
      int id;
      float x;
      float y;
    
      Point() : id(-1), x(0.0), y(0.0) {}
    };
    
    struct Obstacle {
      int id;
      rclcpp::Time t; 
      Point center;
      float width;
      int seen;
      bool matched;
      std::vector<Point> points;
    
      Obstacle() : id(-1), t(rclcpp::Time(0)), width(0.0), seen(1), matched(false) {}
    };
    
    struct TrackedObstacle {
      int id;
      std::vector<Point> traj;
      std::vector<rclcpp::Time> time;  
    };
    
private: 
    std::string input_scan_topic_; 
    std::string odom_frame_; 
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    laser_geometry::LaserProjection projector_;  

    int scan_buffer_size_;
    float thres_point_dist_;
    int thresh_min_points_;
    int thresh_max_points_;
    float min_vel_tracked_;
    float max_vel_tracked_;

    float track_distance_;
    float track_timeout_;
    int obstacle_count_;  

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<dynamic_msgs::msg::DynamicObstacles>::SharedPtr dyn_obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){ 

    }
}