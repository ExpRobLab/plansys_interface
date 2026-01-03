
// #include <memory>
// #include <string>
// #include <cmath>
// #include <fstream>
// #include "plansys2_executor/ActionExecutorClient.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "lifecycle_msgs/msg/transition.hpp"

// #include "bme_gazebo_basics/action/scan.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "aruco_opencv_msgs/msg/aruco_detection.hpp"


// using namespace std::chrono_literals;
// class MoveToDetect : public plansys2::ActionExecutorClient
// {
// public:
//   MoveToDetect()
//   : plansys2::ActionExecutorClient("move_to_detect", 500ms)
//   {
//     // Subscription odometry
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1));

//     // Client Nav2
//     nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
//       this, "navigate_to_pose");

//     scan_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Scan>(
//       this, "scan_environment");


//     // Setup Aruco Listener
//     aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
//       "/aruco_detections", 10, std::bind(&MoveToDetect::detection_callback, this, std::placeholders::_1));

//     // TF
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
     
    
//     // inizializza i flag
//     nav2_done_ = false;
//     nav2_success_ = false;

//     current_wp_idx_ = 0;
//   }
//   void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
//     for (auto & marker : msg->markers) {
//       if (detected_markers_.find(marker.marker_id) == detected_markers_.end()) {
//         try {
//             std::string frame_id = "marker_" + std::to_string(marker.marker_id);
            
//             geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
//               "map", frame_id, tf2::TimePointZero);
            
//             detected_markers_[marker.marker_id] = {t.transform.translation.x, t.transform.translation.y};
            
//             RCLCPP_INFO(get_logger(), "Mapped Marker %ld", (long)marker.marker_id);
//         } catch (const tf2::TransformException & ex) {}
//       }
//     }
//   }
// private:
//   // Resetta stato ogni volta che l’azione viene attivata
//   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//   on_activate(const rclcpp_lifecycle::State & previous_state) override
//   {
//     RCLCPP_INFO(get_logger(), "[MoveToDetect] Attivata");
//     current_wp_idx_ = 0;
//     goal_sent_ = false;
//     nav2_done_ = false;
//     nav2_success_ = false;
//     detected_markers_.clear();
//     return plansys2::ActionExecutorClient::on_activate(previous_state);
//   }

//   void do_work() override
//   {

//     auto args = get_arguments();
//     if (args.size() < 2) {
//         finish(false, 0.0, "Argomenti insufficienti");
//         return;
//     }

//     std::string marker_to = args[1];
//     double gx, gy;

//     if (marker_to == "marker1") { gx=-6; gy=-6; }
//     else if (marker_to == "marker2") { gx=-6; gy=6; }
//     else if (marker_to == "marker3") { gx=6; gy=-6; }
//     else if (marker_to == "marker4") { gx=6; gy=6; }
//     else {
//         finish(false, 0.0, "Marker ignoto");
//         return;
//     }

//     if (!goal_sent_) {
//         if (!nav2_client_->wait_for_action_server(1s)) return;

//         nav2_msgs::action::NavigateToPose::Goal goal_msg;
//         goal_msg.pose.header.frame_id = "map";
//         goal_msg.pose.header.stamp = now();
//         goal_msg.pose.pose.position.x = gx;
//         goal_msg.pose.pose.position.y = gy;
//         goal_msg.pose.pose.orientation.w = 1.0;

//         rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
//         options.result_callback = [this](auto result){
//             nav2_done_ = true;
               
//             nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
//         };

//         nav2_client_->async_send_goal(goal_msg, options);
//         goal_sent_ = true;
//     }

//     // finish SOLO qui
//     if (nav2_done_) {
//         current_wp_idx_++;
//         if ((size_t)current_wp_idx_ >= 4) {
//           // Save markers to file for the next action
//           std::ofstream file("/tmp/detected_markers.csv");
//           for(auto const& [id, pos] : detected_markers_) {
//               file << id << "," << pos.first << "," << pos.second << "\n";
//           }
//           file.close();
//           finish(true, 1.0, "Exploration Complete");
//           return;
//         }


//         finish(nav2_success_, 1.0, "Completed");
//         return;
//     }

//     double dist = std::hypot(gx - current_x_, gy - current_y_);
//     send_feedback(std::max(0.0, 1.0 - dist / 10.0),
//                   "In movimento verso " + marker_to);
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     current_x_ = msg->pose.pose.position.x;
//     current_y_ = msg->pose.pose.position.y;
//   }

//   // Stato interno
//   bool goal_sent_ = false;
//   bool nav2_done_ = false;     
//   bool nav2_success_ = false;   
//   int current_wp_idx_ =0;
//   double goal_x_ = 0.0, goal_y_ = 0.0;
//   double current_x_ = 0.0, current_y_ = 0.0;

//   std::map<long, std::pair<double, double>> detected_markers_;

//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
//   rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SharedPtr scan_client_;
//   rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
//   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// };


// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MoveToDetect>();
//   node->set_parameter(rclcpp::Parameter("action_name", "move_to_detect"));

//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

//   rclcpp::spin(node->get_node_base_interface());
//   rclcpp::shutdown();
//   return 0;
// }


#include <memory>
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <iostream>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "bme_gazebo_basics/action/scan.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class MoveToDetec : public plansys2::ActionExecutorClient
{
public:
  MoveToDetec()
  : plansys2::ActionExecutorClient("move_to_detection_area", 500ms)
  {
    // --- 1. Hardcoded Waypoint Definitions ---
    // These must match the names in problem.pddl (wp1, wp2...)
    waypoints_["wp1"] = {-6.0, -7.0};
    waypoints_["wp2"] = {-4.0, 6.0};
    waypoints_["wp3"] = {7.0, -7.0};
    waypoints_["wp4"] = {7.0, 6.0};

    // --- 2. Action Clients ---
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    scan_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Scan>(this, "scan_environment");

    // --- 3. Detection Listener ---
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 10, std::bind(&MoveToDetec::detection_callback, this, std::placeholders::_1));

    // --- 4. TF Buffer ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
    if (step_ != 1) return; // Only record during the scanning phase

    for (auto & marker : msg->markers) {
      if (detected_markers_.find(marker.marker_id) == detected_markers_.end()) {
        try {
            std::string frame_id = "marker_" + std::to_string(marker.marker_id);
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
              "map", frame_id, tf2::TimePointZero);
            
            detected_markers_[marker.marker_id] = {t.transform.translation.x, t.transform.translation.y};
            
            // Append to shared CSV file
            std::ofstream file("/tmp/detected_markers.csv", std::ios::app);
            file << marker.marker_id << "," << t.transform.translation.x << "," << t.transform.translation.y << "\n";
            file.close();
            
            RCLCPP_INFO(get_logger(), "Detected & Saved Marker %ld", (long)marker.marker_id);
        } catch (...) {}
      }
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    step_ = 0;
    nav_goal_sent_ = false;
    scan_goal_sent_ = false;
    
    // Clean CSV at start of mission if needed, or handle in launch
    // std::ofstream file("/tmp/detected_markers.csv", std::ios::trunc);

    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    std::vector<std::string> args = get_arguments();
    // Expected PDDL Args: [robot_name, waypoint_name] e.g. [mogi_bot, wp1]
    if (args.size() < 2) { finish(false, 0.0, "Missing arguments"); return; }
    
    std::string wp_id = args[1]; 
    
    if (waypoints_.find(wp_id) == waypoints_.end()) {
        finish(false, 0.0, "Unknown waypoint ID: " + wp_id);
        return;
    }
    auto target = waypoints_[wp_id];

    // --- Step 0: Navigate ---
    if (step_ == 0) {
        if (!nav_goal_sent_) {
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = target.first;
            goal.pose.pose.position.y = target.second;
            goal.pose.pose.orientation.w = 1.0;
            
            auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            opts.result_callback = [this](auto) { step_ = 1; }; // Success -> Scan
            nav_client_->async_send_goal(goal, opts);
            nav_goal_sent_ = true;
            RCLCPP_INFO(get_logger(), "Navigating to %s (%.1f, %.1f)", wp_id.c_str(), target.first, target.second);
        }
    }
    // --- Step 1: Scan ---
    else if (step_ == 1) {
        if (!scan_goal_sent_) {
             auto goal = bme_gazebo_basics::action::Scan::Goal();
             goal.target_angle = 6.28; // 360 degrees
             auto opts = rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SendGoalOptions();
             opts.result_callback = [this](auto) { finish(true, 1.0, "Area Scanned"); };
             scan_client_->async_send_goal(goal, opts);
             scan_goal_sent_ = true;
             RCLCPP_INFO(get_logger(), "Scanning area %s...", wp_id.c_str());
        }
    }
  }

private:
  int step_ = 0;
  bool nav_goal_sent_ = false;
  bool scan_goal_sent_ = false;
  
  std::map<std::string, std::pair<double, double>> waypoints_;
  std::map<long, std::pair<double, double>> detected_markers_;
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToDetec>();
  node->set_parameter(rclcpp::Parameter("action_name", "move_to_detection_area"));
  
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
