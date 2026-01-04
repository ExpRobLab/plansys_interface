#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <vector>
#include <set>
#include <algorithm>
#include <filesystem> // Per gestione path standard

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp> // Per path relativo
#include "bme_gazebo_basics/action/scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

const size_t TARGET_MARKERS_COUNT = 2; 

class MoveToDetect : public plansys2::ActionExecutorClient
{
public:
  struct MarkerData
  {
    long id;
    std::string name;
    std::string frame;
    double x;
    double y;
    double goal_x;
    double goal_y;
  };

  MoveToDetect()
      : plansys2::ActionExecutorClient("move_to_detect", 500ms)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MoveToDetect::odom_callback, this, std::placeholders::_1));

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    scan_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Scan>(
        this, "scan_environment");

    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
        "/aruco_detections", 10, std::bind(&MoveToDetect::detection_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    nav2_done_ = false;
    nav2_success_ = false;
    current_wp_idx_ = 0;
  }

private:
  // Funzione helper per risalire alla radice della workspace
  std::string get_ws_path() {
    try {
        std::string pkg_path = ament_index_cpp::get_package_share_directory("bme_gazebo_basics");
        fs::path p(pkg_path);
        // Risale: share/bme_gazebo_basics -> share -> install -> workspace
        return p.parent_path().parent_path().parent_path().parent_path().string();
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Errore nel trovare il path del pacchetto!");
        return ".";
    }
  }

  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {
    if (markers_detected_this_run_.size() < TARGET_MARKERS_COUNT)
    {
      for (auto &marker : msg->markers)
      {
        bool already_in_local_list = std::any_of(
          markers_detected_this_run_.begin(), 
          markers_detected_this_run_.end(),
          [&](const MarkerData& m) { return m.id == marker.marker_id; }
        );

        if (!already_in_local_list)
        {
          try
          {
            std::string frame_id = "marker_" + std::to_string(marker.marker_id);
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", frame_id, tf2::TimePointZero);

            MarkerData md;
            md.id = marker.marker_id;
            md.x = t.transform.translation.x;
            md.y = t.transform.translation.y;
            md.frame = frame_id;
            md.name = this->marker_to; 
            md.goal_x = this->goal_x_;
            md.goal_y = this->goal_y_;

            markers_detected_this_run_.push_back(md);
            
            RCLCPP_INFO(get_logger(), "Rilevato Marker %ld (%zu/%zu)", 
                        md.id, markers_detected_this_run_.size(), TARGET_MARKERS_COUNT);
            
            if (markers_detected_this_run_.size() >= TARGET_MARKERS_COUNT) break;
          }
          catch (const tf2::TransformException &ex) {}
        }
      }
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override
  {
    RCLCPP_INFO(get_logger(), "[MoveToDetect] Attivata");
    markers_detected_this_run_.clear(); 
    goal_sent_ = false;
    nav2_done_ = false;
    nav2_success_ = false;
    return plansys2::ActionExecutorClient::on_activate(previous_state);
  }

  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 2)
    {
      finish(false, 0.0, "Argomenti insufficienti");
      return;
    }

    this->marker_to = args[1];
    double gx, gy;

    if (this->marker_to == "marker1") { gx = -6.0; gy = -6.0; }
    else if (this->marker_to == "marker2") { gx = -6.0; gy = 6.0; }
    else if (this->marker_to == "marker3") { gx = 6.0; gy = -6.0; }
    else if (this->marker_to == "marker4") { gx = 6.0; gy = 6.0; }
    else { finish(false, 0.0, "Marker ignoto"); return; }

    this->goal_x_ = gx;
    this->goal_y_ = gy;

    if (!goal_sent_)
    {
      if (!nav2_client_->wait_for_action_server(1s)) return;

      nav2_msgs::action::NavigateToPose::Goal goal_msg;
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();
      goal_msg.pose.pose.position.x = gx;
      goal_msg.pose.pose.position.y = gy;
      goal_msg.pose.pose.orientation.w = 1.0;

      auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      options.result_callback = [this](auto result) {
        nav2_done_ = true;
        nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      };

      nav2_client_->async_send_goal(goal_msg, options);
      goal_sent_ = true;
    }

    if (nav2_done_)
    {
      save_in_yaml();
      finish(nav2_success_, 1.0, "Completed");
      return;
    }

    double dist = std::hypot(gx - current_x_, gy - current_y_);
    send_feedback(std::max(0.0, 1.0 - dist / 10.0), "In movimento verso " + this->marker_to);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  void save_in_yaml()
  {
    std::string ws_path = get_ws_path();
    std::string file_path = ws_path + "/points_detected/detected_markers.yaml";

    // Assicuriamoci che la cartella esista
    fs::path dir_path = fs::path(file_path).parent_path();
    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }

    bool file_exists = fs::exists(file_path);

    // Apertura in modalità APPEND
    std::ofstream outfile(file_path, std::ios::app);
    
    if (outfile.is_open())
    {
        if (!file_exists || fs::file_size(file_path) == 0) {
            outfile << "markers:\n";
        }

        for (const auto &m : markers_detected_this_run_)
        {
            outfile << "  - id: " << m.id << "\n";
            outfile << "    name: " << m.name << "\n";
            outfile << "    frame: " << m.frame << "\n";
            outfile << "    x: " << m.x << "\n";
            outfile << "    y: " << m.y << "\n";
            outfile << "    goal_x: " << m.goal_x << "\n";
            outfile << "    goal_y: " << m.goal_y << "\n";
        }
        
        outfile.close();
        RCLCPP_INFO(get_logger(), "Marker salvati con successo in: %s", file_path.c_str());
    } else {
        RCLCPP_ERROR(get_logger(), "Errore nell'apertura del file YAML per il salvataggio!");
    }
  }

  bool goal_sent_ = false;
  bool nav2_done_ = false;
  bool nav2_success_ = false;
  int current_wp_idx_ = 0;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0;
  
  std::vector<MarkerData> markers_detected_this_run_;
  std::string marker_to;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
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