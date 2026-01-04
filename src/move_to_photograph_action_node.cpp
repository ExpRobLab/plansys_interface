
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
#include "sensor_msgs/msg/image.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include <rcpputils/filesystem_helper.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
class MoveToPhotograph : public plansys2::ActionExecutorClient
{
public:
    MoveToPhotograph()
        : plansys2::ActionExecutorClient("move_to_photograph", 500ms)
    {
        // Subscription odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MoveToPhotograph::odom_callback, this, std::placeholders::_1));

        // Client Nav2
        nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        scan_client_ = rclcpp_action::create_client<bme_gazebo_basics::action::Scan>(
            this, "scan_environment");

        auto qos = rclcpp::SensorDataQoS();
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            { last_img_ = msg; });

        // inizializza i flag
        nav2_done_ = false;
        nav2_success_ = false;

        current_wp_idx_ = 0;
    }

private:
    void load_detected_markers()
    {
        // Path file
        std::string home = std::getenv("HOME");
        std::string file_path = home + "/Desktop/Experimental/assignment2_ws/points_detected/detected_markers.yaml";

        std::ifstream infile(file_path);
        if (!infile.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "Nessun file YAML trovato, parto da zero");
            return; // se il file non esiste, non fare nulla
        }

        std::string line;
        long id = 0;
        double x = 0.0, y = 0.0;
        double goal_x = 0.0, goal_y = 0.0;
        std::string name;
        bool first_marker_saved = false;

        while (std::getline(infile, line) && !first_marker_saved)
        {
            // rimuovi spazi iniziali
            line.erase(0, line.find_first_not_of(" \t"));

            if (line.find("id:") == 0)
            {
                id = std::stol(line.substr(line.find(":") + 1));
            }
            else if (line.find("name:") == 0)
            {
                name = line.substr(line.find(":") + 2); // rimuove ": "
            }
            else if (line.find("x:") == 0)
            {
                x = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("y:") == 0)
            {
                y = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("goal_x:") == 0)
            {
                goal_x = std::stod(line.substr(line.find(":") + 1));
            }
            else if (line.find("goal_y:") == 0)
            {
                goal_y = std::stod(line.substr(line.find(":") + 1));
                // Salva tutto il primo marker completo
                if (!first_marker_saved)
                {
                    this->target_x_ = x;
                    this->target_y_ = y;
                    this->pos_photo_x = goal_x;
                    this->pos_photo_y = goal_y;
                    this->target_id_ = id;
                    this->target_name_ = name;
                    first_marker_saved = true;
                }
            }
        }

        infile.close();

        if (first_marker_saved)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Caricato primo marker dal YAML: id=%ld, name=%s, x=%.2f, y=%.2f, goal_x=%.2f, goal_y=%.2f",
                        static_cast<long>(target_id_), this->target_name_.c_str(), this->target_x_, this->target_y_, this->pos_photo_x, this->pos_photo_y);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Nessun marker trovato nel file YAML");
        }
        }

        // Resetta stato ogni volta che l’azione viene attivata
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "[MoveToPhotograph] Attivata");
            current_wp_idx_++;
            load_detected_markers();
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

            std::string marker_to = this->target_name_;
            double gx, gy;

            if (marker_to == "marker1")
            {
                gx = -6;
                gy = -6;
            }
            else if (marker_to == "marker2")
            {
                gx = -6;
                gy = 6;
            }
            else if (marker_to == "marker3")
            {
                gx = 6;
                gy = -6;
            }
            else if (marker_to == "marker4")
            {
                gx = 6;
                gy = 6;
            }
            else
            {
                finish(false, 0.0, "Marker ignoto");
                return;
            }

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
                if (last_img_)
                {
                    save_img();
                }
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

        void save_img()
        {
            try
            {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
                cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2),
                           50, CV_RGB(0, 255, 0), 3);

                std::string home = std::getenv("HOME");
                
                // Construct path: ~/assignment1_bundle/resources/marker_X.png
                std::string dir_path = home + "/Desktop/Experimental/assignment2_ws";
                dir_path = dir_path + "/images";
                std::string file_path = dir_path + "/marker_" + std::to_string(static_cast<long>(target_id_)) + ".png";

                try
                {
                    rcpputils::fs::create_directories(dir_path);
                    if (cv::imwrite(file_path, cv_ptr->image))
                    {
                        RCLCPP_INFO(get_logger(), "Saved to %s", file_path.c_str());
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to write image to %s", file_path.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), "Filesystem error: %s", e.what());
                }
                
             
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Filesystem error: %s", e.what());
            }
        }
        // Stato interno
        bool goal_sent_ = false;
        bool nav2_done_ = false;
        bool nav2_success_ = false;
        int current_wp_idx_ = 0;
        double goal_x_ = 0.0, goal_y_ = 0.0;
        double current_x_ = 0.0, current_y_ = 0.0;
        bool only_one_marker_found_ = false;

        double target_x_ = 0.0;
        double target_y_ = 1.0;
        double pos_photo_x = 0.0;
        double pos_photo_y = 1.0;
        double target_id_ = -1;
        std::string target_name_ = "home_target";

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
        rclcpp_action::Client<bme_gazebo_basics::action::Scan>::SharedPtr scan_client_;

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
