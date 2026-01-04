
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bme_gazebo_basics/action/scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include <rcpputils/filesystem_helper.hpp>

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

    scan_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Scan>(
        this, "scan_environment");

    // Setup Aruco Listener
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
        "/aruco_detections", 10, std::bind(&MoveToDetect::detection_callback, this, std::placeholders::_1));

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // inizializza i flag
    nav2_done_ = false;
    nav2_success_ = false;

    current_wp_idx_ = 0;
  }
  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {

    if (!this->only_one_marker_found_)
    {
      for (auto &marker : msg->markers)
      {
        if (detected_markers_.find(marker.marker_id) == detected_markers_.end())
        {
          try
          {
            std::string frame_id = "marker_" + std::to_string(marker.marker_id);

            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", frame_id, tf2::TimePointZero);

            detected_markers_[marker.marker_id] = {t.transform.translation.x, t.transform.translation.y};

            RCLCPP_INFO(get_logger(), "Mapped Marker %ld", (long)marker.marker_id);
            RCLCPP_INFO(get_logger(), "Numero detectati: %d", this->current_wp_idx_);
            this->only_one_marker_found_ = true;
          }
          catch (const tf2::TransformException &ex)
          {
          }
        }
      }
    }
  }

private:
  // Resetta stato ogni volta che l’azione viene attivata
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override
  {
    RCLCPP_INFO(get_logger(), "[MoveToDetect] Attivata");
    current_wp_idx_++;
    only_one_marker_found_ = false;
    goal_sent_ = false;
    nav2_done_ = false;
    nav2_success_ = false;
    detected_markers_.clear();
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

    if (this->marker_to == "marker1")
    {
      gx = -6;
      gy = -6;
    }
    else if (this->marker_to == "marker2")
    {
      gx = -6;
      gy = 6;
    }
    else if (this->marker_to == "marker3")
    {
      gx = 6;
      gy = -6;
    }
    else if (this->marker_to == "marker4")
    {
      gx = 6;
      gy = 6;
    }
    else
    {
      finish(false, 0.0, "Marker ignoto");
      return;
    }


    this->goal_x_ = gx;
    this->goal_y_ = gy;

    if (!goal_sent_)
    {
      if (!nav2_client_->wait_for_action_server(1s))
        return;

      nav2_msgs::action::NavigateToPose::Goal goal_msg;
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = now();
      goal_msg.pose.pose.position.x = gx;
      goal_msg.pose.pose.position.y = gy;
      goal_msg.pose.pose.orientation.w = 1.0;

      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
      options.result_callback = [this](auto result)
      {
        nav2_done_ = true;

        nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
      };

      nav2_client_->async_send_goal(goal_msg, options);
      goal_sent_ = true;
    }

    // finish SOLO qui

    if (nav2_done_)
    {
      save_in_yaml();
      finish(nav2_success_, 1.0, "Completed");
      return;
    }

    double dist = std::hypot(gx - current_x_, gy - current_y_);
    send_feedback(std::max(0.0, 1.0 - dist / 10.0),
                  "In movimento verso " + this->marker_to);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }
void save_in_yaml()
{
    const char* home_c = std::getenv("HOME");
    if (!home_c)
    {
        RCLCPP_ERROR(get_logger(), "Cannot determine HOME directory");
        return;
    }
    std::string home(home_c);
    std::string dir_path = home + "/Desktop/Experimental/assignment2_ws/points_detected";
    std::string file_path = dir_path + "/detected_markers.yaml";

    rcpputils::fs::create_directories(dir_path);

    // --- Step 1: leggi tutti i marker esistenti ---
    std::map<long, std::pair<double,double>> all_markers;    // id -> xy
    std::map<long,std::string> all_marker_names;             // id -> name
    std::map<long,std::pair<double,double>> all_goal_pos;    // id -> goal xy

    std::ifstream infile(file_path);
    if (infile.is_open())
    {
        std::string line;
        long id = 0;
        double x = 0.0, y = 0.0, gx = 0.0, gy = 0.0;
        std::string name;
        while (std::getline(infile, line))
        {
            line.erase(0, line.find_first_not_of(" \t"));
            if (line.find("id:") == 0) id = std::stol(line.substr(line.find(":") + 1));
            else if (line.find("x:") == 0) x = std::stod(line.substr(line.find(":") + 1));
            else if (line.find("y:") == 0) y = std::stod(line.substr(line.find(":") + 1));
            else if (line.find("name:") == 0) name = line.substr(line.find(":") + 2);
            else if (line.find("goal_x:") == 0) gx = std::stod(line.substr(line.find(":") + 1));
            else if (line.find("goal_y:") == 0) gy = std::stod(line.substr(line.find(":") + 1));

            // Salva solo se abbiamo tutti i dati
            if (id != 0 && !name.empty())
            {
                all_markers[id] = {x, y};
                all_marker_names[id] = name;
                all_goal_pos[id] = {gx, gy};
                id = 0; x = y = gx = gy = 0.0; name.clear();
            }
        }
        infile.close();
    }

    // --- Step 2: aggiungi/aggiorna i nuovi marker ---
    for (const auto &[id, pos] : detected_markers_)
    {
        all_markers[id] = pos;

        // usa marker_to se presente (nuovo marker), altrimenti mantiene il nome vecchio
        std::string name;
        if (!this->marker_to.empty())
            name = this->marker_to;
        else if (all_marker_names.find(id) != all_marker_names.end())
            name = all_marker_names[id];
        else
            name = "marker" + std::to_string(id);

        all_marker_names[id] = name;

        // goal position corrente
        all_goal_pos[id] = {goal_x_, goal_y_};
    }

    // --- Step 3: scrivi YAML ordinato per id ---
    std::ofstream file(file_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Cannot open YAML file: %s", file_path.c_str());
        return;
    }

    file << "markers:\n";
    for (const auto &[id, pos] : all_markers)  // map è già ordinata per chiave
    {
        file << "  - id: " << id << "\n";
        file << "    name: " << all_marker_names[id] << "\n";
        file << "    x: " << pos.first << "\n";
        file << "    y: " << pos.second << "\n";
        file << "    goal_x: " << all_goal_pos[id].first << "\n";
        file << "    goal_y: " << all_goal_pos[id].second << "\n";
    }

    file.close();
    RCLCPP_INFO(get_logger(), "Saved %zu markers in YAML", all_markers.size());
}


  // Stato interno
  bool goal_sent_ = false;
  bool nav2_done_ = false;
  bool nav2_success_ = false;
  int current_wp_idx_ = 0;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0;
  bool only_one_marker_found_ = false;
  std::map<long, std::pair<double, double>> detected_markers_;
  std::string marker_to ;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SharedPtr scan_client_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
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
