
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rcpputils/filesystem_helper.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include <filesystem> // Fondamentale per fs::path
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace std::chrono_literals;
namespace fs = std::filesystem;
class ChangeState : public plansys2::ActionExecutorClient
{
public:
  ChangeState()
      : plansys2::ActionExecutorClient("change_state", 500ms)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ChangeState::odom_callback, this, std::placeholders::_1));

    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    nav2_done_ = false;
    nav2_success_ = false;
  }

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

private:
  // Funzione helper per risalire alla radice della workspace
  std::string get_ws_path() {
    try {
        // Ottiene: .../assignment2_ws/install/bme_gazebo_basics/share/bme_gazebo_basics
        std::string pkg_path = ament_index_cpp::get_package_share_directory("bme_gazebo_basics");
        fs::path p(pkg_path);
        // Risale 4 livelli: share -> bme_gazebo_basics -> install -> workspace
        return p.parent_path().parent_path().parent_path().parent_path().string();
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Impossibile trovare il percorso del pacchetto bme_gazebo_basics!");
        return "."; 
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override
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
    if (args.size() < 7)
    {
      finish(false, 0.0, "Argomenti insufficienti");
      return;
    }

    std::string marker_to = "home_point";
    double gx = 0;
    double gy = 1;

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

    if (nav2_done_)
    {
      cleanup_and_save_yaml();
      finish(nav2_success_, 1.0, "Completed");
      return;
    }

    double dist = std::hypot(gx - current_x_, gy - current_y_);
    send_feedback(std::max(0.0, 1.0 - dist / 10.0),
                  "In movimento verso " + marker_to);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  void cleanup_and_save_yaml()
  {
    // Generazione percorso relativo alla workspace
    std::string file_path = get_ws_path() + "/points_detected/detected_markers.yaml";
    
    // Assicura che la directory esista
    fs::path dir_path = fs::path(file_path).parent_path();
    if (!fs::exists(dir_path)) {
        fs::create_directories(dir_path);
    }

    std::map<std::string, std::vector<MarkerData>> markers_by_name;

    std::ifstream infile(file_path);
    if (infile.is_open())
    {
      std::string line;
      MarkerData m;
      int fields_found = 0;

      while (std::getline(infile, line))
      {
        size_t first = line.find_first_not_of(" \t-");
        if (first == std::string::npos)
          continue;
        std::string clean_line = line.substr(first);

        if (clean_line.find("id:") == 0) { m.id = std::stol(clean_line.substr(3)); fields_found++; }
        else if (clean_line.find("name:") == 0) {
          m.name = clean_line.substr(5);
          m.name.erase(0, m.name.find_first_not_of(" "));
          fields_found++;
        }
        else if (clean_line.find("frame:") == 0) {
          m.frame = clean_line.substr(6);
          m.frame.erase(0, m.frame.find_first_not_of(" "));
          fields_found++;
        }
        else if (clean_line.find("goal_x:") == 0) { m.goal_x = std::stod(clean_line.substr(7)); fields_found++; }
        else if (clean_line.find("goal_y:") == 0) { m.goal_y = std::stod(clean_line.substr(7)); fields_found++; }
        else if (clean_line.find("x:") == 0) { m.x = std::stod(clean_line.substr(2)); fields_found++; }
        else if (clean_line.find("y:") == 0) { m.y = std::stod(clean_line.substr(2)); fields_found++; }

        if (fields_found == 7)
        {
          markers_by_name[m.name].push_back(m);
          fields_found = 0;
        }
      }
      infile.close();
    }

    if (markers_by_name.empty())
      return;

    std::set<long> used_ids;
    std::vector<MarkerData> final_list;

    for (auto &[name, candidates] : markers_by_name)
    {
      std::sort(candidates.begin(), candidates.end(),
                [](const MarkerData &a, const MarkerData &b) { return a.id < b.id; });

      for (const auto &m : candidates)
      {
        if (!used_ids.count(m.id))
        {
          final_list.push_back(m);
          used_ids.insert(m.id);
          break;
        }
      }
    }

    std::ofstream outfile(file_path, std::ios::trunc);
    if (outfile.is_open())
    {
      outfile << "markers:\n";
      for (const auto &m : final_list)
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
      RCLCPP_INFO(get_logger(), "Cleanup completato in: %s", file_path.c_str());
    }
  }

  bool goal_sent_ = false;
  bool nav2_done_ = false;
  bool nav2_success_ = false;
  double current_x_ = 0.0, current_y_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
};

int main(int argc, char **argv)
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
