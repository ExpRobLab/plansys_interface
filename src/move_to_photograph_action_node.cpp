// #include "plansys2_executor/ActionExecutorClient.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "lifecycle_msgs/msg/transition.hpp"
// #include <memory>
// #include <chrono>
// #include <string>
// #include <algorithm>
// #include <cmath>

// using namespace std::chrono_literals;

// class MoveToPhotograph : public plansys2::ActionExecutorClient
// {
// public:
//   MoveToPhotograph()
//   : plansys2::ActionExecutorClient("move_to_photo", 500ms), goal_sent_(false), progress_(0.0)
//   {
//     odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&MoveToPhotograph::odom_callback, this, std::placeholders::_1)
//     );

//     nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
//     nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
//       nav2_node_, "navigate_to_pose"
//     );
//   }

// private:
//   void do_work() override
//   {

//     RCLCPP_WARN(get_logger(), "MoveToPhotograph running");

//     auto args = get_arguments();
//     if (args.size() < 3) {
//       RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
//       finish(true, 0.0, "Insufficient arguments");
//       return;
//     }

//     std::string marker_to = args[1];
//     std::string marker_from = args[2];
//     std::string robot = args[0];
//     double goal_x, goal_y;

//     if (marker_to == "marker2") {
//       goal_x = 6.0;
//       goal_y = 6.0;
//     } else if (marker_to == "marker3") {
//       goal_x = 6.0;
//       goal_y = -6.0;
//     } else if (marker_to == "marker4") {
//       goal_x = 6.0;
//       goal_y = 6.0;
//     } else {
//       RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", marker_to.c_str());
//       finish(false, 0.0, "Unknown waypoint");
//       return;
//     }

//     if (!goal_sent_) {
//       if (!nav2_client_->wait_for_action_server(1s)) {
//         RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
//         return;
//       }

//       geometry_msgs::msg::PoseStamped goal_pose;
//       goal_pose.header.frame_id = "map";
//       goal_pose.pose.position.x = goal_x;
//       goal_pose.pose.position.y = goal_y;
//       goal_pose.pose.orientation.w = 1.0;

//       auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
//       goal_msg.pose = goal_pose;

//       rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
//       send_goal_options.result_callback =
//         [this, marker_to](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
//         {
//           if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
//             RCLCPP_ERROR(get_logger(), "Navigation failed: %s", marker_to.c_str());
//             finish(false, 1.0, "Move failed");
//           }
//         };

//       nav2_client_->async_send_goal(goal_msg, send_goal_options);
//       goal_sent_ = true;

//       start_x_ = current_x_;
//       start_y_ = current_y_;
//     }

//     double total_dist = std::hypot(goal_x - start_x_, goal_y - start_y_);
//     double rem_dist   = std::hypot(goal_x - current_x_, goal_y - current_y_);
//     progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

//     send_feedback(progress_, "Moving to " + marker_to);

//     if (rem_dist < 0.6) {
//       goal_sent_= false;
//       progress_ = 1.0;
//       send_feedback(progress_, "Moving to " + marker_to);
//       RCLCPP_INFO(get_logger(), "Reached waypoint: %s", marker_to.c_str());
//       finish(true, 1.0, "Move completed");
//     }

//     rclcpp::spin_some(nav2_node_);
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     current_x_ = msg->pose.pose.position.x;
//     current_y_ = msg->pose.pose.position.y;
//   }

//   float progress_;
//   bool goal_sent_;
//   double start_x_ = 0.0, start_y_ = 0.0;
//   double current_x_ = 0.0, current_y_ = 0.0;

//   rclcpp::Node::SharedPtr nav2_node_;
//   rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MoveToPhotograph>();

//   node->set_parameter(rclcpp::Parameter("action_name", "move_to_photo"));
//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

//   rclcpp::spin(node->get_node_base_interface());

//   rclcpp::shutdown();

//   return 0;
// }


#include <memory>
#include <fstream>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "bme_gazebo_basics/action/center.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cstdlib>
#include <filesystem>
#include <cv_bridge/cv_bridge.h>


using namespace std::chrono_literals;

class CaptureAction : public plansys2::ActionExecutorClient
{
public:
  CaptureAction()
  : plansys2::ActionExecutorClient("capture", 500ms)
  {
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    center_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Center>(this, "center_on_marker");
    
    auto qos = rclcpp::SensorDataQoS();
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", qos, 
        [this](const sensor_msgs::msg::Image::SharedPtr msg){ last_img_ = msg; });
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    // Load markers from file
    std::ifstream file("/tmp/detected_markers.csv");
    markers_.clear();
    long id; char comma; double x, y;
    while(file >> id >> comma >> x >> comma >> y) {
        markers_.push_back({id, {x, y}});
    }
    // Sort by ID
    std::sort(markers_.begin(), markers_.end());
    
    current_idx_ = 0;
    step_ = 0;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work()
  {
    if (current_idx_ >= markers_.size()) {
        finish(true, 1.0, "All Captured");
        return;
    }

    long mid = markers_[current_idx_].first;
    double mx = markers_[current_idx_].second.first;
    double my = markers_[current_idx_].second.second;

    if (step_ == 0) { // Navigate
        if (!nav_goal_sent_) {
            auto goal = nav2_msgs::action::NavigateToPose::Goal();
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = now();
            goal.pose.pose.position.x = mx;
            goal.pose.pose.position.y = my;
            goal.pose.pose.orientation.w = 1.0;
            
            auto opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            opts.result_callback = [this](auto) { nav_done_ = true; };
            nav_client_->async_send_goal(goal, opts);
            nav_goal_sent_ = true;
            nav_done_ = false;
            RCLCPP_INFO(get_logger(), "Going to Marker %ld", mid);
        }
        if (nav_done_) { nav_goal_sent_ = false; step_ = 1; }
    }
    else if (step_ == 1) { // Center
        if (!center_goal_sent_) {
            auto goal = bme_gazebo_basics::action::Center::Goal();
            goal.marker_id = mid;
            auto opts = rclcpp_action::Client<bme_gazebo_basics::action::Center>::SendGoalOptions();
            opts.result_callback = [this](auto r) { 
                center_success_ = (r.code == rclcpp_action::ResultCode::SUCCEEDED); 
                center_done_ = true; 
            };
            center_client_->async_send_goal(goal, opts);
            center_goal_sent_ = true;
            center_done_ = false;
            RCLCPP_INFO(get_logger(), "Centering on %ld", mid);
        }
        if (center_done_) {
            center_goal_sent_ = false;
            if (center_success_) step_ = 2; // Capture
            else { 
                current_idx_++; step_ = 0; // Skip if centering failed
            }
        }
    }
    else if (step_ == 2) { // Save Image
        if (last_img_) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
                cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 
                           50, CV_RGB(0, 255, 0), 3);
                
                const char* home = std::getenv("HOME");
                if (home) {
                    // Construct path: ~/assignment1_bundle/resources/marker_X.png
                    std::string dir_path = std::string(home) + "/assignment1_bundle/resources/";
                    std::string file_path = dir_path + "marker_" + std::to_string(mid) + ".png";
                    
                    try {
                        std::filesystem::create_directories(dir_path);
                        if (cv::imwrite(file_path, cv_ptr->image)) {
                            RCLCPP_INFO(get_logger(), "Saved to %s", file_path.c_str());
                        } else {
                            RCLCPP_ERROR(get_logger(), "Failed to write image to %s", file_path.c_str());
                        }
                    } catch (const std::filesystem::filesystem_error& e) {
                        RCLCPP_ERROR(get_logger(), "Filesystem error: %s", e.what());
                    }
                } else {
                    RCLCPP_ERROR(get_logger(), "Could not determine HOME directory to save images.");
                }
            } catch(...) {}
        }
        current_idx_++;
        step_ = 0;
    }
  }

private:
  int step_ = 0;
  size_t current_idx_ = 0;
  bool nav_goal_sent_ = false; bool nav_done_ = false;
  bool center_goal_sent_ = false; bool center_done_ = false; bool center_success_ = false;
  
  std::vector<std::pair<long, std::pair<double, double>>> markers_;
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<bme_gazebo_basics::action::Center>::SharedPtr center_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  sensor_msgs::msg::Image::SharedPtr last_img_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CaptureAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "capture"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}