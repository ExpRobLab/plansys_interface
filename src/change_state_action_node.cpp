
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
class ChangeState : public plansys2::ActionExecutorClient
{
public:
  ChangeState()
  : plansys2::ActionExecutorClient("change_state", 500ms)
  {
    // Subscription odometry
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ChangeState::odom_callback, this, std::placeholders::_1));

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
    RCLCPP_INFO(get_logger(), "[ChangeState] Attivata");
    goal_sent_ = false;
    nav2_done_ = false;
    nav2_success_ = false;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 7) {
        finish(false, 0.0, "Argomenti insufficienti");
        return;
    }

    std::string marker_to = "home_point";
    double gx = 0;
    double gy = 1;

    

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
  auto node = std::make_shared<ChangeState>();
  node->set_parameter(rclcpp::Parameter("action_name", "change_state"));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}



