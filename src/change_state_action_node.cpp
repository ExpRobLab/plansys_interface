
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
  std::string get_ws_path()
  {
    try
    {
      // Ottiene: .../assignment2_ws/install/bme_gazebo_basics/share/bme_gazebo_basics
      std::string pkg_path = ament_index_cpp::get_package_share_directory("bme_gazebo_basics");
      fs::path p(pkg_path);
      // Risale 4 livelli: share -> bme_gazebo_basics -> install -> workspace
      return p.parent_path().parent_path().parent_path().parent_path().string();
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Impossibile trovare il percorso del pacchetto bme_gazebo_basics!");
      return ".";
    }
  }
  // Resetta stato ogni volta che l’azione viene attivata
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
    double gx = 3;
    double gy = 1;

    // invio goal solo la prima volta
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
      options.result_callback = [this, marker_to](auto result)
      {
        this->nav2_done_ = true;
        this->nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);

        std::string reason;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
          reason = "SUCCEEDED";
          break;
        case rclcpp_action::ResultCode::ABORTED:
          reason = "ABORTED";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          reason = "CANCELED";
          break;
        default:
          reason = "UNKNOWN";
          break;
        }

        if (!nav2_success_)
        {
          RCLCPP_ERROR(get_logger(), "Navigation to %s failed: %s",
                       marker_to.c_str(), reason.c_str());
          finish(false, 1.0, "Navigation failed: " + reason);
        }
        else
        {
          this->nav2_done_ = true;
          this->nav2_success_ = true;
          RCLCPP_INFO(get_logger(), "Navigation to %s succeeded", marker_to.c_str());
        }
      };

      nav2_client_->async_send_goal(goal_msg, options);
      goal_sent_ = true;

      this->start_x_ = this->current_x_;
      this->start_y_ = this->current_y_;
    }

    // Calcolo distanza e progresso
    double total_dist = std::hypot(gx - this->start_x_, gy - this->start_y_);
    double rem_dist = std::hypot(gx - current_x_, gy - current_y_);
    progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

    send_feedback(progress_, "Moving to " + marker_to);

    // Threshold per completamento automatico
    const double distance_threshold = 0.1; // metri
    if (nav2_done_)
    {
      RCLCPP_INFO(get_logger(), "Goal raggiunto per distanza, completo l'azione");
      nav2_done_ = true;
      nav2_success_ = true;
      cleanup_and_save_yaml();
      finish(true, 1.0, "Completed");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }
  void cleanup_and_save_yaml()
  {
    std::string file_path = get_ws_path() + "/points_detected/detected_markers.yaml";

    // Assicura che la directory esista
    fs::path dir_path = fs::path(file_path).parent_path();
    if (!fs::exists(dir_path))
    {
      fs::create_directories(dir_path);
    }
    // Mappa per tenere solo il marker con ID minore per ogni NOME (marker1, marker2...)
    std::map<std::string, MarkerData> best_markers_by_name;

    // --- STEP 1: Lettura e Filtraggio per Nome ---
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

        if (clean_line.find("id:") == 0)
        {
          m.id = std::stol(clean_line.substr(3));
          fields_found++;
        }
        else if (clean_line.find("name:") == 0)
        {
          m.name = clean_line.substr(5);
          m.name.erase(0, m.name.find_first_not_of(" "));
          fields_found++;
        }
        else if (clean_line.find("frame:") == 0)
        {
          m.frame = clean_line.substr(6);
          m.frame.erase(0, m.frame.find_first_not_of(" "));
          fields_found++;
        }
        else if (clean_line.find("goal_x:") == 0)
        {
          m.goal_x = std::stod(clean_line.substr(7));
          fields_found++;
        }
        else if (clean_line.find("goal_y:") == 0)
        {
          m.goal_y = std::stod(clean_line.substr(7));
          fields_found++;
        }
        else if (clean_line.find("x:") == 0)
        {
          m.x = std::stod(clean_line.substr(2));
          fields_found++;
        }
        else if (clean_line.find("y:") == 0)
        {
          m.y = std::stod(clean_line.substr(2));
          fields_found++;
        }

        if (fields_found == 7)
        {
          // Se è la prima volta che vediamo questo nome O se questo ID è più piccolo del precedente
          if (best_markers_by_name.find(m.name) == best_markers_by_name.end() || m.id < best_markers_by_name[m.name].id)
          {
            best_markers_by_name[m.name] = m;
          }
          fields_found = 0;
        }
      }
      infile.close();
    }

    if (best_markers_by_name.empty())
      return;

    // --- STEP 2: Trasferimento in vettore e risoluzione conflitti ID ---
    std::vector<MarkerData> final_list;
    for (auto const &[name, data] : best_markers_by_name)
    {
      final_list.push_back(data);
    }

    // Ordiniamo per ID attuale
    std::sort(final_list.begin(), final_list.end(), [](const MarkerData &a, const MarkerData &b)
              { return a.id < b.id; });

    // Risoluzione ID duplicati tra nomi diversi
    for (size_t i = 1; i < final_list.size(); ++i)
    {
      if (final_list[i].id <= final_list[i - 1].id)
      {
        final_list[i].id = final_list[i - 1].id + 1;
      }
    }

    // --- STEP 3: Scrittura finale ---
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
      RCLCPP_INFO(get_logger(), "Cleanup completato: mantenuti %zu marker univoci.", final_list.size());
    }
  }
  // Stato interno
  float progress_;

  bool goal_sent_ = false;
  bool nav2_done_ = false;
  bool nav2_success_ = false;
  double start_x_ = 0.0, start_y_ = 0.0;
  double goal_x_ = 0.0, goal_y_ = 0.0;
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
