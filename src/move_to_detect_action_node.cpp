// // /////////////////////////FUNZIONANTE MA SI BLOCCA IL SECONDO ARUCO E BOM

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

// class MoveToDetect : public plansys2::ActionExecutorClient
// {
// public:
//   MoveToDetect()
//   :plansys2::ActionExecutorClient("move_to_detect", 500ms), goal_sent_(false), progress_(0.0)
//   {
//     odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1)
//     );

//     nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
//     nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
//       nav2_node_, "navigate_to_pose"
//     );
//   }

// private:
//   void do_work() override
//   {

//     RCLCPP_WARN(get_logger(), "MoveToDetect running");
//     auto args = get_arguments();
//     if (args.size() < 4) {
//       RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
//       finish(false, 0.0, "Insufficient arguments");
//       return;
//     }

// //  auto state = get_current_state();
// // std::string state_name = state.label(); // oppure state.id() se vuoi id numerico

// // if (state_name != last_action_state_) {
// //     goal_sent_ = false;
// //     progress_ = 0.0;
// //     nav_failed_ = false;
// //     start_x_ = current_x_;
// //     start_y_ = current_y_;
// //     last_action_state_ = state_name;
// // }
//     std::string robot = args[0];
//     std::string marker_to = args[1];
//     std::string marker_base = args[2];

//     double goal_x, goal_y;
//     if (marker_to == "marker1") {
//       goal_x = -6.0;
//       goal_y = -6.0;
//     } else if (marker_to == "marker2") {
//       goal_x = -6.0;
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
//       if (!nav2_client_->wait_for_action_server(10s)) {
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
//             // finish(true, 1.0, "Move failed");
//                   // nav_failed_ = true;
//                   finish(true, 1.0, "Move failed");

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

//     // if (nav_failed_) {
//     //   finish(false, progress_, "Navigation failed");
//     //   return;
//     // }
//     rclcpp::spin_some(nav2_node_);
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     current_x_ = msg->pose.pose.position.x;
//     current_y_ = msg->pose.pose.position.y;
//   }

//   bool nav_failed_;
//   float progress_;
//   bool goal_sent_;
//   double start_x_ = 0.0, start_y_ = 0.0;
//   double current_x_ = 0.0, current_y_ = 0.0;
//   std::string last_action_state_;
//   rclcpp::Node::SharedPtr nav2_node_;
//   rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MoveToDetect>();

//   node->set_parameter(rclcpp::Parameter("action_name", "move_to_detect"));
//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
// // rclcpp::sleep_for(500ms);
//   rclcpp::spin(node->get_node_base_interface());

//   rclcpp::shutdown();

//   return 0;
// }

// NONS SI MUOVE NULLA
// #include <memory>
// #include <algorithm>
// #include <cmath>
// #include "plansys2_executor/ActionExecutorClient.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "nav_msgs/msg/odometry.hpp"

// using namespace std::chrono_literals;

// class MoveToDetect : public plansys2::ActionExecutorClient
// {
// public:
//   MoveToDetect()
//   : plansys2::ActionExecutorClient("move_to_detect_performer", 500ms)
//   {
//     nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
//       this, "navigate_to_pose");

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10, std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1));
//   }

// private:
//   // Reset ad ogni nuova azione del piano
//   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//   on_activate(const rclcpp_lifecycle::State & previous_state) override
//   {
//     goal_sent_ = false;
//     goal_handle_active_ = false;
//     return plansys2::ActionExecutorClient::on_activate(previous_state);
//   }

//   void do_work() override
//   {
//     if (!goal_sent_) {
//       send_nav2_goal();
//       return;
//     }

//     // Qui puoi inviare il feedback di distanza se vuoi
//     send_feedback(0.5, "In movement...");
//   }

//   void send_nav2_goal()
//   {
//     auto args = get_arguments();
//     std::string marker_to = args[1];

//     double gx, gy;
//     if (marker_to == "marker1") { gx = -6.0; gy = -6.0; }
//     else if (marker_to == "marker2") { gx = -6.0; gy = 6.0; }
//     else if (marker_to == "marker3") { gx = 6.0; gy = -6.0; }
//     else if (marker_to == "marker4") { gx = 6.0; gy = 6.0; }
//     else { finish(false, 0.0, "Marker ignoto"); return; }

//     if (!nav2_client_->wait_for_action_server(1s)) {
//       RCLCPP_WARN(get_logger(), "Waiting for Nav2...");
//       return;
//     }

//     auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
//     goal_msg.pose.header.frame_id = "map";
//     goal_msg.pose.pose.position.x = gx;
//     goal_msg.pose.pose.position.y = gy;
//     goal_msg.pose.pose.orientation.w = 1.0;

//     auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

//     send_goal_options.result_callback =
//       [this, marker_to](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//           RCLCPP_INFO(get_logger(), "Arrivato a %s", marker_to.c_str());
//           this->finish(true, 1.0, "Successo");
//         } else {
//           this->finish(false, 0.0, "Fallito");
//         }
//       };

//     nav2_client_->async_send_goal(goal_msg, send_goal_options);
//     goal_sent_ = true;
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     current_x_ = msg->pose.pose.position.x;
//     current_y_ = msg->pose.pose.position.y;
//   }

//   bool goal_sent_ = false;
//   bool goal_handle_active_ = false;
//   double current_x_, current_y_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//   rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
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

// ////////3 codice
#include <memory>
#include <string>
#include <cmath>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;
class MoveToDetect : public plansys2::ActionExecutorClient
{
public:
  MoveToDetect()
  : plansys2::ActionExecutorClient("move_to_detect", 500ms)
  {
    // Subscription odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1));

    // Client Nav2
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");
    
    // inizializza i flag
    nav2_done_ = false;
    nav2_success_ = false;
  }

private:
  // Resetta stato ogni volta che l’azione viene attivata
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override
  {
    RCLCPP_INFO(get_logger(), "[MoveToDetect] Attivata");
    goal_sent_ = false;
    nav2_done_ = false;
    nav2_success_ = false;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 2) {
        finish(false, 0.0, "Argomenti insufficienti");
        return;
    }

    std::string marker_to = args[1];
    double gx, gy;

    if (marker_to == "marker1") { gx=-6; gy=-6; }
    else if (marker_to == "marker2") { gx=-6; gy=6; }
    else if (marker_to == "marker3") { gx=6; gy=-6; }
    else if (marker_to == "marker4") { gx=6; gy=6; }
    else {
        finish(false, 0.0, "Marker ignoto");
        return;
    }

    if (!goal_sent_) {
        if (!nav2_client_->wait_for_action_server(1s)) return;

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = now();
        goal_msg.pose.pose.position.x = gx;
        goal_msg.pose.pose.position.y = gy;
        goal_msg.pose.pose.orientation.w = 1.0;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
        options.result_callback = [this](auto result){
            nav2_done_ = true;
            nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        };

        nav2_client_->async_send_goal(goal_msg, options);
        goal_sent_ = true;
    }

    // finish SOLO qui
    if (nav2_done_) {
        finish(nav2_success_, 1.0, "Completed");
        return;
    }

    double dist = std::hypot(gx - current_x_, gy - current_y_);
    send_feedback(std::max(0.0, 1.0 - dist / 10.0),
                  "In movimento verso " + marker_to);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  // Stato interno
  bool goal_sent_ = false;
  bool nav2_done_ = false;     
  bool nav2_success_ = false;   

  double goal_x_ = 0.0, goal_y_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToDetect>();
  node->set_parameter(rclcpp::Parameter("action_name", "move_to_detect"));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}





// /////////// con waypoints intermedi

// #include <memory>
// #include <string>
// #include <vector>
// #include <cmath>
// #include "plansys2_executor/ActionExecutorClient.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_msgs/action/navigate_through_poses.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "lifecycle_msgs/msg/transition.hpp"

// using namespace std::chrono_literals;

// class MoveToDetect : public plansys2::ActionExecutorClient
// {
// public:
//   MoveToDetect()
//   : plansys2::ActionExecutorClient("move_to_detect", 500ms), goal_sent_(false)
//   {
//     // Subscription odometry
//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//       "/odom", 10,
//       std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1)
//     );

//     // Client Nav2 NavigateThroughPoses
//     nav2_node_ = rclcpp::Node::make_shared("move_to_detect_nav2_client");
//     nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
//       nav2_node_, "navigate_through_poses"
//     );
//   }

// private:
//   void do_work() override
//   {
//     auto args = get_arguments();
//     if (args.size() < 2) {
//       RCLCPP_ERROR(get_logger(), "Argomenti insufficienti");
//       finish(false, 0.0, "Insufficient arguments");
//       return;
//     }

//     std::string robot = args[0];
//     std::string marker_to = args[1];

//     // Genera waypoint: partenza, intermedi e destinazione
//     std::vector<geometry_msgs::msg::PoseStamped> waypoints;

//     // Start
//     geometry_msgs::msg::PoseStamped start_pose;
//     start_pose.header.frame_id = "map";
//     start_pose.pose.position.x = current_x_;
//     start_pose.pose.position.y = current_y_;
//     start_pose.pose.orientation.w = 1.0;
//     waypoints.push_back(start_pose);

//     // Goal finale
//     geometry_msgs::msg::PoseStamped goal_pose;
//     goal_pose.header.frame_id = "map";

//     if (marker_to == "marker1") { goal_pose.pose.position.x = -6.0; goal_pose.pose.position.y = -6.0; }
//     else if (marker_to == "marker2") { goal_pose.pose.position.x = -6.0; goal_pose.pose.position.y = 6.0; }
//     else if (marker_to == "marker3") { goal_pose.pose.position.x = 6.0; goal_pose.pose.position.y = -6.0; }
//     else if (marker_to == "marker4") { goal_pose.pose.position.x = 6.0; goal_pose.pose.position.y = 6.0; }
//     else { finish(false, 0.0, "Marker ignoto"); return; }

//     goal_pose.pose.orientation.w = 1.0;

//     // Waypoint intermedi (2 punti)
//     geometry_msgs::msg::PoseStamped wp1, wp2;
//     wp1.header.frame_id = "map";
//     wp1.pose.position.x = (current_x_ + goal_pose.pose.position.x)/2;
//     wp1.pose.position.y = current_y_;
//     wp1.pose.orientation.w = 1.0;

//     wp2.header.frame_id = "map";
//     wp2.pose.position.x = goal_pose.pose.position.x;
//     wp2.pose.position.y = (current_y_ + goal_pose.pose.position.y)/2;
//     wp2.pose.orientation.w = 1.0;

//     waypoints.push_back(wp1);
//     waypoints.push_back(wp2);
//     waypoints.push_back(goal_pose);

//     // Invio goal solo una volta
//     if (!goal_sent_)
//     {
//       if (!nav_client_->wait_for_action_server(10s)) {
//         RCLCPP_WARN(get_logger(), "navigate_through_poses server non pronto");
//         return;
//       }

//       nav2_msgs::action::NavigateThroughPoses::Goal goal_msg;
//       goal_msg.poses = waypoints;

//       rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions options;

//       // Result callback
//       options.result_callback = [this, marker_to](auto result) {
//         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
//           RCLCPP_INFO(get_logger(), "Goal raggiunto: %s", marker_to.c_str());
//           finish(true, 1.0, "Move completed");
//         } else {
//           RCLCPP_ERROR(get_logger(), "Goal fallito: %s", marker_to.c_str());
//           finish(false, 0.0, "Move failed");
//         }
//       };

//       // Feedback callback
//       options.feedback_callback =
//         [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>> /*gh*/,
//                std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
//       {
//         double dist_remain = std::hypot(
//           feedback->current_pose.pose.position.x - current_x_,
//           feedback->current_pose.pose.position.y - current_y_
//         );
//         send_feedback(std::max(0.0, 1.0 - dist_remain/10.0), "Navigating...");
//       };

//       nav_client_->async_send_goal(goal_msg, options);
//       goal_sent_ = true;
//     }

//     rclcpp::spin_some(nav2_node_);
//   }

//   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     current_x_ = msg->pose.pose.position.x;
//     current_y_ = msg->pose.pose.position.y;
//   }

//   bool goal_sent_;
//   double current_x_ = 0.0, current_y_ = 0.0;
//   rclcpp::Node::SharedPtr nav2_node_;
//   rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_client_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
