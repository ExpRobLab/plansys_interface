#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "bme_gazebo_basics/action/scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include <rcpputils/filesystem_helper.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class MoveToPhotograph : public plansys2::ActionExecutorClient
{
public:


    MoveToPhotograph()
        : plansys2::ActionExecutorClient("move_to_photograph", 500ms)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MoveToPhotograph::odom_callback, this, std::placeholders::_1));

        nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        auto qos = rclcpp::SensorDataQoS();
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            { last_img_ = msg; });

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
            std::string pkg_path = ament_index_cpp::get_package_share_directory("bme_gazebo_basics");
            fs::path p(pkg_path);
            return p.parent_path().parent_path().parent_path().parent_path().string();
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "Impossibile trovare il percorso del pacchetto!");
            return ".";
        }
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
        RCLCPP_INFO(get_logger(), "[MoveToPhotograph] Attivata");
        goal_sent_ = false;
        nav2_done_ = false;
        nav2_success_ = false;
        return plansys2::ActionExecutorClient::on_activate(previous_state);
    }

    void load_detected_markers()
    {
        // RCLCPP_INFO(get_logger(), "[MoveToPhotograph] loadsssssssssssssssing");
        std::string ws_path = get_ws_path();
        // AGGIUNTO IL PUNTO E VIRGOLA QUI SOTTO
        std::string file_path = ws_path + "/points_detected/detected_markers.yaml";
        std::ifstream infile(file_path);

        if (!infile.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "File YAML non trovato: %s", file_path.c_str());
            return;
        }
        // RCLCPP_INFO(get_logger(), "[MoveToPhotograph] dopo seconda load");

        std::string line;
        MarkerData m;
        int fields_found = 0;
        bool marker_loaded = false;

        while (std::getline(infile, line) && !marker_loaded)
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
                this->marker_target = m;
                marker_loaded = true;
            }


        }
        infile.close();
    }

    void remove_first_marker_from_yaml()
    {
        std::string ws_path = get_ws_path();
        // AGGIUNTO IL PUNTO E VIRGOLA QUI SOTTO
        std::string file_path = ws_path + "/points_detected/detected_markers.yaml";
        std::vector<MarkerData> remaining;

        std::ifstream infile(file_path);
        if (infile.is_open())
        {
            std::string line;
            MarkerData m;
            int fields = 0;
            bool skipped = false;
            while (std::getline(infile, line))
            {
                size_t first = line.find_first_not_of(" \t-");
                if (first == std::string::npos)
                    continue;
                std::string clean = line.substr(first);

                if (clean.find("id:") == 0)
                {
                    m.id = std::stol(clean.substr(3));
                    fields++;
                }
                else if (clean.find("name:") == 0)
                {
                    m.name = clean.substr(5);
                    m.name.erase(0, m.name.find_first_not_of(" "));
                    fields++;
                }
                else if (clean.find("frame:") == 0)
                {
                    m.frame = clean.substr(6);
                    m.frame.erase(0, m.frame.find_first_not_of(" "));
                    fields++;
                }
                else if (clean.find("goal_x:") == 0)
                {
                    m.goal_x = std::stod(clean.substr(7));
                    fields++;
                }
                else if (clean.find("goal_y:") == 0)
                {
                    m.goal_y = std::stod(clean.substr(7));
                    fields++;
                }
                else if (clean.find("x:") == 0)
                {
                    m.x = std::stod(clean.substr(2));
                    fields++;
                }
                else if (clean.find("y:") == 0)
                {
                    m.y = std::stod(clean.substr(2));
                    fields++;
                }

                if (fields == 7)
                {
                    if (!skipped)
                        skipped = true;
                    else
                        remaining.push_back(m);
                    fields = 0;
                }
            }
            infile.close();
        }

        std::ofstream outfile(file_path, std::ios::trunc);
        if (outfile.is_open())
        {
            outfile << "markers:\n";
            for (const auto &rm : remaining)
            {
                outfile << "  - id: " << rm.id << "\n    name: " << rm.name << "\n    frame: " << rm.frame << "\n    x: " << rm.x << "\n    y: " << rm.y << "\n    goal_x: " << rm.goal_x << "\n    goal_y: " << rm.goal_y << "\n";
            }
            outfile.close();
        }
    }

    void do_work() override
    {
        load_detected_markers();
        if (marker_target.name.empty())
        {
            

            finish(false, 0.0, "Nessun marker disponibile");
            return;
        }
        // RCLCPP_INFO(get_logger(), "[MoveToPhotograph] dopo la if");

        double gx = marker_target.goal_x;
        double gy = marker_target.goal_y;

        if (!goal_sent_)
        {
            if (!nav2_client_->wait_for_action_server(1s)){
                return;
            }
            nav2_msgs::action::NavigateToPose::Goal goal_msg;
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = now();
            goal_msg.pose.pose.position.x = gx;
            goal_msg.pose.pose.position.y = gy;
            goal_msg.pose.pose.orientation.w = 1.0;

            auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            options.result_callback = [this](auto result)
            {
                nav2_done_ = true;
                nav2_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
            };
            nav2_client_->async_send_goal(goal_msg, options);
            goal_sent_ = true;
        }
        double total_dist = std::hypot(gx - start_x_, gy - start_y_);
        double rem_dist = std::hypot(gx - current_x_, gy - current_y_);
        progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

        send_feedback(progress_, "Moving to " + this->marker_target.name);
        if (nav2_done_ && rem_dist < 0.6)
        {

            if (nav2_success_ && last_img_)
            {

                save_img();
                remove_first_marker_from_yaml();
                finish(true, 1.0, "Completed");
            }
            else if (!nav2_success_)
            {
                finish(false, 0.0, "Nav2 failed");
            }
            return;
        }
    }

    void save_img()
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
            cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2), 50, CV_RGB(0, 255, 0), 3);
            std::string dir_path = get_ws_path() + "/images";
            if (!fs::exists(dir_path))
            {
                fs::create_directories(dir_path);
            }
            std::string file_path = dir_path + "/" + this->marker_target.name + "_" + std::to_string(marker_target.id) + ".png";
            cv::imwrite(file_path, cv_ptr->image);
            RCLCPP_INFO(get_logger(), "Immagine salvata: %s", file_path.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Errore salvataggio: %s", e.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
    }


    bool goal_sent_ = false, nav2_done_ = false, nav2_success_ = false;
    double current_x_ = 0.0, current_y_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0;

    float progress_;

    MarkerData marker_target;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    sensor_msgs::msg::Image::SharedPtr last_img_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPhotograph>();
    node->set_parameter(rclcpp::Parameter("action_name", "move_to_photograph"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}