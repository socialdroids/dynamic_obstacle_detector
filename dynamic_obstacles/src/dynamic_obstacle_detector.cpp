#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include "visualization_msgs/msg/marker_array.hpp"
#include <people_msgs/msg/people.hpp>
#include <people_msgs/msg/person.hpp>

// #include <leg_detector_msgs/msg/leg.hpp>
// #include <leg_detector_msgs/msg/leg_array.hpp>
#include <std_msgs/msg/header.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"


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
        this->declare_parameter("dist_between_obstacles", 0.2);
        this->declare_parameter("time_to_keep_obstacle", 3.0); 
        this->declare_parameter("keep_obstacle", true);

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
        this->get_parameter("dist_between_obstacles", dist_between_obstacles);
        this->get_parameter("time_to_keep_obstacle", time_to_keep_obstacle_);
        this->get_parameter("keep_obstacle", keep_obstacle_); 
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DynamicObstacleDetector::scanCallback, this, std::placeholders::_1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/keepout_filter_mask", rclcpp::QoS(10),
          std::bind(&DynamicObstacleDetector::mapCallback, this, std::placeholders::_1));

        obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacles/static_markers", 10);
        //leg_pub_ = this->create_publisher<leg_detector_msgs::msg::LegArray>("detected_leg_clusters", 20);
        dyn_obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dynamic_obstacles/dynamic_markers", 10);
        // points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic_obstacles/points", 10);
        obstacles_pub_ = this->create_publisher<people_msgs::msg::People>("/people", 10);

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

    std::vector<ObstacleKF> tracked_obstacles_;
    std::vector<std::vector<Obstacle>> obstacles_;
    
    nav_msgs::msg::OccupancyGrid map_;
    
    laser_geometry::LaserProjection projector_;  

    bool keep_obstacle_; 
    int scan_buffer_size_;
    float thres_point_dist_;
    float dist_between_obstacles; 
    int thresh_min_points_;
    int thresh_max_points_;
    float min_vel_tracked_;
    float max_vel_tracked_;
    float time_to_keep_obstacle_; 

    float track_distance_;
    float track_timeout_;
    int obstacle_count_;  

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dyn_obs_pub_;
    //rclcpp::Publisher<leg_detector_msgs::msg::LegArray>::SharedPtr leg_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr obstacles_pub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = *msg; 
    }
  bool isPointOnObstacle(const double &px, const double &py, double radius) {
    if (map_.data.empty()) return false;

    geometry_msgs::msg::PointStamped point_in, point_out;
    point_in.header.frame_id = "odom";  
    point_in.point.x = px;
    point_in.point.y = py;
    point_in.point.z = 0.0;

    try {
        buffer_->transform(point_in, point_out, "map", tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Falha ao transformar ponto para 'map': %s", ex.what());
        return false;
    }

    double transformed_x = point_out.point.x;
    double transformed_y = point_out.point.y;

    double resolution = map_.info.resolution; 
    double map_origin_x = map_.info.origin.position.x;  
    double map_origin_y = map_.info.origin.position.y;  

    int map_x = (transformed_x - map_origin_x) / resolution;
    int map_y = (transformed_y - map_origin_y) / resolution;

    if (map_x < 0 || map_x >= map_.info.width || map_y < 0 || map_y >= map_.info.height) {
        return false;
    }

    int map_value = map_.data[map_y * map_.info.width + map_x];

    if (map_value != 0) {
        return true;
    }

    int radius_in_cells = static_cast<int>(radius / resolution);  

    for (int dx = -radius_in_cells; dx <= radius_in_cells; ++dx) {
        for (int dy = -radius_in_cells; dy <= radius_in_cells; ++dy) {
            int neighbor_x = map_x + dx;
            int neighbor_y = map_y + dy;

            if (neighbor_x >= 0 && neighbor_x < map_.info.width &&
                neighbor_y >= 0 && neighbor_y < map_.info.height) {
                int neighbor_value = map_.data[neighbor_y * map_.info.width + neighbor_x];

                if (neighbor_value != 0) {
                    return true;  
                }
            }
        }
    }

    return false;
  }

  
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){ 
      std::vector<Point> points;
      float angle_min = msg->angle_min;
      float angle = angle_min + (msg->angle_increment / 2.0);
      geometry_msgs::msg::PointStamped ps;
      ps.header.frame_id = msg->header.frame_id;
      //ros::Time(0); --> msg->time
      ps.header.stamp =  rclcpp::Time(0);
      for (unsigned int i = 0; i < msg->ranges.size(); i++) {
        if (!isinf(msg->ranges[i]) && !isnan(msg->ranges[i])) {
          Point p;
          p.id = i + 1;
          ps.point.x = msg->ranges[i] * cos(angle);
          ps.point.y = msg->ranges[i] * sin(angle);
          ps.point.z = 0.0;
          geometry_msgs::msg::PointStamped psn;
          try {
            psn = buffer_->transform(ps, odom_frame_);
          } catch (tf2::TransformException &ex) {
           RCLCPP_ERROR(this->get_logger(),"Could NOT transform point to %s: %s", odom_frame_.c_str(),
                     ex.what());
            return;
          }
          p.x = psn.point.x;
          p.y = psn.point.y;
          points.push_back(p);
        }
        angle = angle + msg->angle_increment;
      }
  
      if (!points.empty()) {
        // Publish the points in RViz
        // publish_points(points);
        // Find obstacles candidates in the point set
        std::vector<Obstacle> obs = findObs(points, msg->header.stamp);
        // printf("found %i obstacles!\n", (int)obs.size());
        publish_obs(obs, std::string("obstacles"), 2, 0.1, msg->header);

        // Track with the KFs
        trackMovingObstaclesKF(obs);
      }
    }

    void trackMovingObstaclesKF(std::vector<Obstacle> &obs) {
      // printf("\nNew obstacles size: %i\n\n", (int)obs.size());
      // Vector with flags to consider when a detection have been used
      std::vector<bool> used(obs.size(), false);
  
      double posDev = 0.2; // 0.1; //laser
  
      // Update tracked obstacles with the detections according to min distance
      int k;
      double x1, y1, x2, y2, dist, minDist;
      for (int i = 0; i < (int)tracked_obstacles_.size(); i++) {
        // if(tracked_obstacles_[i].updated)
        //	continue;
  
        // Search the closest detection to every person tracked
        k = 0;
        x1 = tracked_obstacles_[i].x(0, 0);
        y1 = tracked_obstacles_[i].x(1, 0);
        minDist = 1000000.0;
        for (int j = 0; j < (int)obs.size(); j++) {
          if (!used[j]) {
            x2 = obs[j].center.x;
            y2 = obs[j].center.y;
            dist = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            if (dist < minDist) {
              k = j;
              minDist = dist;
            }
          }
        }
  
        // Check if the distant is too high
        if (minDist < track_distance_) {
          // Predict the position of the obstacle to this instant
          auto now_time = this->get_clock()->now();
          double now_seconds = now_time.seconds();
          double tStamp_seconds = tracked_obstacles_[i].tStamp.seconds();


          tracked_obstacles_[i].predict(
              (now_seconds - tStamp_seconds));
  
          // Update the filter
          tracked_obstacles_[i].update(obs[k].center.x, obs[k].center.y, posDev,
                                       this->get_clock()->now() );
          // Mark the detection as used
          used[k] = true;
          // std::cout << "Done" << std::endl;
          // std::cout << "Updating obstacle " << i << " with detection " << k <<
          // std::endl;
        }
      }
  
      // Add new persons to the list
      for (int i = 0; i < (int)obs.size(); i++) {
        if (!used[i]) {
          // std::cout << "New obstacle added to the list: " <<
          // tracked_obstacles_.size() << std::endl;
          ObstacleKF obstacle;
          obstacle.init(obstacle_count_++, obs[i].center.x, obs[i].center.y, 0,
                        0);
          tracked_obstacles_.push_back(obstacle);
          // std::cout << "New obstacle added! size: " <<
          // tracked_obstacles_.size() << std::endl;
          used[i] = true;
        }
      }
  
      // Remove too old estimations without update and static obstacles
      std::vector<ObstacleKF> temp;

      for (int i = 0; i < (int)tracked_obstacles_.size(); i++) {
  
        auto now_time = this->get_clock()->now();
        double now_seconds = now_time.seconds();
        double tStamp_seconds = tracked_obstacles_[i].tStamp.seconds();

        if (now_seconds - tStamp_seconds < track_timeout_) { //&& linvel >= min_vel_tracked_
          temp.push_back(tracked_obstacles_[i]);
        }else{ 
          if(keep_obstacle_){
            double lost_seconds = (now_seconds - tStamp_seconds);
            if (lost_seconds < time_to_keep_obstacle_) {
              temp.push_back(tracked_obstacles_[i]);
            }
          }
        }
      }
      tracked_obstacles_.clear();
      tracked_obstacles_ = temp;
  
      // Publish dynamic obstacles
      // publishDynObsMarker();
  
      // Publish DynamicObstacles message
      publishDynamicObstacles();
    }
    void publishDynamicObstacles() {

      people_msgs::msg::People dyn_obs;
      people_msgs::msg::Person ob;
  
      visualization_msgs::msg::MarkerArray obsMarkers;
      visualization_msgs::msg::Marker marker;
  
      // Get marker color
      float r, g, b;
      r = 0.0;
      g = 1.0;
      b = 0.0;
  
      dyn_obs.header.frame_id = odom_frame_;
      dyn_obs.header.stamp = this->get_clock()->now();
      marker.header.frame_id = dyn_obs.header.frame_id;
      marker.header.stamp = dyn_obs.header.stamp;
      marker.ns = this->get_name();
      for (int i = 0; i < (int)tracked_obstacles_.size(); i++) {
        if (tracked_obstacles_[i].updated) {
          double time = (this->get_clock()->now() - tracked_obstacles_[i].tStamp).seconds();
          double vx = tracked_obstacles_[i].x(2, 0);
          double vy = tracked_obstacles_[i].x(3, 0);
          double linvel = sqrt((vx * vx) + (vy * vy));
          double x_arrow = vx * time;
          double y_arrow = vy * time;
          double yaw = atan2(vy, vx);
          // printf("Tracked obs %i, linvel: %.3f\n", tracked_obstacles_[i].id,
          // linvel); printf("tracked ob %i linvel: %.3f, min_vel_tracked:
          // %.3f\n", tracked_obstacles_[i].id, linvel, min_vel_tracked_);
  
          if (linvel >= min_vel_tracked_) {
  
            // Dynamic obstacle
            ob.name = std::to_string(tracked_obstacles_[i].id);
            ob.position.x = tracked_obstacles_[i].x(0, 0);
            ob.position.y = tracked_obstacles_[i].x(1, 0);
            ob.position.z = 0.0;
            ob.velocity.z = 0.0;
            ob.velocity.x = vx;
            ob.velocity.y = vy;
            ob.reliability = 0.8;
            dyn_obs.people.push_back(ob);
  
            // Cylinder
            marker.id = 20 * tracked_obstacles_[i].id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = ob.position;
            marker.pose.position.z = 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.25;
            marker.scale.y = 0.25;
            marker.scale.z = 1.0;
            marker.color.a = 1.0;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            obsMarkers.markers.push_back(marker);
  
            // Arrow
            marker.id = 20 * tracked_obstacles_[i].id + 1;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = marker.pose.position;
  
            // marker.points.push_back(marker.pose.position);
            // geometry_msgs::Point end;
            // end.x = x_arrow;
            // end.y = y_arrow;
            // end.z = marker.pose.position.z;
            // marker.points.push_back(end);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw); 
            marker.pose.orientation = tf2::toMsg(q);
            // marker.pose.orientation.x = 0.0;
            // marker.pose.orientation.y = 0.0;
            // marker.pose.orientation.z = 0.0;
            // marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.2 * (linvel / max_vel_tracked_);
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            obsMarkers.markers.push_back(marker);
          }
        }
        // else
        // {
        // 	// Delete body marker
        // 	marker.id = tracked_obstacles_[i].id;
        // 	marker.type = visualization_msgs::Marker::CYLINDER;
        // 	marker.action = visualization_msgs::Marker::DELETE;
        // 	obsMarkers.markers.push_back(marker);
        // }
      }
  
      // publish obstacle
      obstacles_pub_->publish(dyn_obs);
      // publish marker:
      dyn_obs_pub_->publish(obsMarkers);
    }

    std::vector<Obstacle> findMovingObs3() {
      // use the obs of the first scan as an initial set of tracked obs
      std::vector<TrackedObstacle> tobs;
      int id = 1;
      for (unsigned int i = 0; i < obstacles_[0].size(); i++) {
        TrackedObstacle to;
        to.id = id;
        to.traj.push_back(obstacles_[0][i].center);
        to.time.push_back(obstacles_[0][i].t);
        tobs.push_back(to);
        id++;
      }
      // printf("initial set of obstacles: %i\n", (int)tobs.size());
  
      // Look for the obstacles in the next scan and update the trajectory
      float min_total_dist = 0.0;
      float max_total_dist = 0.0;
      for (unsigned int c = 1; c < scan_buffer_size_; c++) {
        float dt = (obstacles_[c][0].t - obstacles_[c - 1][0].t).seconds();
        float max_dist_step = max_vel_tracked_ * dt;
        float min_dist_step = min_vel_tracked_ * dt;
        min_total_dist += min_dist_step;
        max_total_dist += max_dist_step;
        for (unsigned int i = 0; i < tobs.size(); i++) {
          TrackedObstacle *candidate = &tobs[i];
          Obstacle min;
          float min_dist = 1000;
          for (unsigned int j = 0; j < obstacles_[c].size(); j++) {
            float d = dist(candidate->traj[c - 1], obstacles_[c][j].center);
            if (d < min_dist) {
              min_dist = d;
              min = obstacles_[c][j];
            }
          }
          // printf("TrackedOb %i with ob %i. min_dist: %.2f\n", candidate->id,
          // min.id, min_dist);
          if (min_dist > min_dist_step && min_dist < max_dist_step) {
            candidate->traj.push_back(min.center);
            candidate->time.push_back(obstacles_[c][0].t);
          }
        }
      }
      std::vector<Obstacle> movingobs;
      for (TrackedObstacle tracked : tobs) {
  
        // if not tracked in all (or nearly all) the scans, discard it
        if (tracked.traj.size() < scan_buffer_size_ - 1) {
          continue;
        }
        // printf("\nTracked ob: %i. traj size: %i\n", tracked.id,
        // (int)tracked.traj.size()); printf("\tdist: %.3f", dist(tracked.traj[0]
        // , tracked.traj[tracked.traj.size()-1]));
  
        // check the trajectory is not noise
        if (dist(tracked.traj[0], tracked.traj[tracked.traj.size() - 1]) >=
            min_total_dist) {
          Obstacle o;
          o.id = tracked.id;
          o.center = tracked.traj[tracked.traj.size() - 1];
          movingobs.push_back(o);
        }
      }
      return movingobs;
    }

    std::vector<Obstacle> findObs(const std::vector<Point> &points, rclcpp::Time t) {
      std::vector<Obstacle> obs;
      unsigned int id = 1;
      unsigned int i = 0;
  
      while (i < points.size() - 1) {
  
          Obstacle o;
  
          o.points.push_back(points[i]);
          while ((i + 1) < points.size() &&
                 dist(points[i], points[i + 1]) <= thres_point_dist_) {
              o.points.push_back(points[i + 1]);
              i++;
          }
  
          if ((int)o.points.size() > thresh_min_points_ &&
              (int)o.points.size() < thresh_max_points_) {
  
              o.id = id;
              o.t = t;
              o.width = dist(o.points[0], o.points[o.points.size() - 1]);
              o.center.id = id;
              id++;
              o.seen = 1;
  
              for (Point p : o.points) {
                  o.center.x += p.x;
                  o.center.y += p.y;
              }
              o.center.x /= (float)o.points.size();
              o.center.y /= (float)o.points.size();
              // Ignorando 30 cm de cada lado e ppegando a media de pontos
              // bool valid = true;
              bool valid = !isPointOnObstacle(o.center.x, o.center.y, 0.3);
              if (valid) {
                  bool merged = false;
                  for (Obstacle &existing : obs) {
                      if (dist(o.center, existing.center) <= dist_between_obstacles) {
                          existing.points.insert(existing.points.end(),
                                                 o.points.begin(), o.points.end());
  
                          existing.center.x = 0;
                          existing.center.y = 0;
                          for (Point p : existing.points) {
                              existing.center.x += p.x;
                              existing.center.y += p.y;
                          }
                          existing.center.x /= (float)existing.points.size();
                          existing.center.y /= (float)existing.points.size();
  
                          merged = true;
                          break; 
                      }
                  }
  
                  if (!merged && valid) {
                      // RCLCPP_INFO(this->get_logger(), "Sem obs: %f %f", o.center.x, o.center.y); 
                      obs.push_back(o);
                  }
              }
          }
  
          o.points.clear();
          i++; 
      }
  
      return obs;
    }
  

    float dist(const Point &p1, const Point &p2) {
      return std::hypotf((p1.x - p2.x), (p1.y - p2.y));
    }
  
    void publish_obs(const std::vector<Obstacle> &obs, std::string namespc,
                     int color, double time, std_msgs::msg::Header &header) {
      visualization_msgs::msg::MarkerArray ma;
      visualization_msgs::msg::Marker m;
      //leg_detector_msgs::msg::Leg leg;
      //leg_detector_msgs::msg::LegArray detected_leg_clusters;

      // detected_leg_clusters.header.frame_id = header.frame_id;
      // detected_leg_clusters.header.stamp = header.stamp;

      m.header.frame_id = odom_frame_;
      m.header.stamp = rclcpp::Time(0, 0, RCL_SYSTEM_TIME); 
      m.ns = namespc;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = 0.0;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.2;
      m.scale.y = 0.2;
      m.scale.z = 0.4;
      switch (color) {
      case 0:
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        break;
      case 1:
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        break;
      case 2:
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        break;
      default:
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        break;
      }
      m.color.a = 1.0;
      m.lifetime = rclcpp::Duration::from_seconds(time);
      for (Obstacle o : obs) {
        m.id = o.id;
        m.pose.position.x = o.center.x;
        m.pose.position.y = o.center.y;
        m.pose.position.z = 0.3;
        ma.markers.push_back(m);

        // leg.position.x = o.center.x;
        // leg.position.y = o.center.y;
        // leg.confidence = 0.8;
        // detected_leg_clusters.legs.push_back(leg);
      }
    obs_pub_->publish(ma);
    //leg_pub_->publish(detected_leg_clusters);
  }

  // void publish_points(const std::vector<Point> &points) {
  //   visualization_msgs::msg::Marker m;
  //   m.header.frame_id = odom_frame_;
  //   m.header.stamp = this->get_clock()->now();
  //   m.ns = "dyn_points";
  //   m.type = visualization_msgs::msg::Marker::POINTS;
  //   m.action = visualization_msgs::msg::Marker::ADD;
  //   m.pose.orientation.x = 0.0;
  //   m.pose.orientation.y = 0.0;
  //   m.pose.orientation.z = 0.0;
  //   m.pose.orientation.w = 1.0;
  //   m.scale.x = 0.03;
  //   m.scale.y = 0.03;
  //   m.scale.z = 0.03;
  //   m.color.r = 1.0;
  //   m.color.g = 0.0;
  //   m.color.b = 0.0;
  //   m.color.a = 1.0;
  //   m.id = points[0].id;
  //   m.lifetime = rclcpp::Duration::from_seconds(0.1);
  //   for (Point p : points) {
  //     // if(p.id == -1)
  //     //  continue;
  //     geometry_msgs::msg::Point pt;
  //     pt.x = p.x;
  //     pt.y = p.y;
  //     pt.z = 0.03;
  //     m.points.push_back(pt);
  //   }
  //   points_pub_->publish(m);
  // }


}; 


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
