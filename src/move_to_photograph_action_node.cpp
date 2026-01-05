#include <memory>
#include <string>
#include <cmath>
#include <fstream>
#include <vector>
#include <filesystem>

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
#include "geometry_msgs/msg/twist.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace std::chrono_literals;
namespace fs = std::filesystem;
enum class AlignState
{
    NAVIGATE,
    ALIGN
};
class MoveToPhotograph : public plansys2::ActionExecutorClient
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

    MoveToPhotograph() : plansys2::ActionExecutorClient("move_to_photograph_mode", 500ms)
    {

        // 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MoveToPhotograph::odom_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        auto qos = rclcpp::SensorDataQoS();
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", qos,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            { last_img_ = msg; });

        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("images_detected",10);

        aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
            "/aruco_detections", 10, std::bind(&MoveToPhotograph::detection_callback, this, std::placeholders::_1));

        nav2_done_ = false;
        nav2_success_ = false;
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
    {
        for (const auto &marker : msg->markers)
        {
            if (marker.marker_id != marker_target.id)
                continue; //  SOLO il marker target
              
            try
            {
                std::string marker_frame = "marker_" + std::to_string(marker.marker_id);

                // TRANSFORM LIVE: base_link <- marker
                auto tf = tf_buffer_->lookupTransform(
                    "base_link",
                    marker_frame,
                    tf2::TimePointZero);

                live_marker_x_ = tf.transform.translation.x;
                live_marker_y_ = tf.transform.translation.y;
                if (marker_visible_ == false)
                    marker_visible_ = true;
            }
            catch (const tf2::TransformException &)
            {
                // ERROR
                // marker_visible_ = false;
            }
        }
    }

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
    void load_detected_markers()
    {
        std::string ws_path = get_ws_path();
        std::string file_path = ws_path + "/points_detected/detected_markers.yaml";
        std::ifstream infile(file_path);

        if (!infile.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "File YAML non trovato.");
            return;
        }

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
                        skipped = true; // Salta il primo
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
        auto args = get_arguments();
        if (args.size() < 2)
        {
            finish(false, 0.0, "Argomenti insufficienti");
            return;
        }

        load_detected_markers();

        if (marker_target.name.empty())
        {
            finish(false, 0.0, "Nessun marker disponibile");
            return;
        }

        std::string marker_to = "home_point";
        double gx = marker_target.goal_x;
        double gy = marker_target.goal_y;
        if (this->state_ == AlignState::NAVIGATE)
        { // invio goal solo la prima volta
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
                        RCLCPP_ERROR(get_logger(), "Navigation to %s failed: %s", marker_to.c_str(), reason.c_str());
                        finish(false, 1.0, "Navigation failed: " + reason);
                    }
                    else
                    {
                        this->nav2_done_ = true;
                        this->nav2_success_ = true;
                        this->state_ = AlignState::ALIGN;
                        RCLCPP_INFO(get_logger(), "Navigation to %s succeeded going for align", marker_to.c_str());
                    }
                };
                nav2_client_->async_send_goal(goal_msg, options);
                goal_sent_ = true;
                this->start_x_ = this->current_x_;
                this->start_y_ = this->current_y_;
            } // Calcolo distanza e progresso
            double total_dist = std::hypot(gx - this->start_x_, gy - this->start_y_);
            double rem_dist = std::hypot(gx - current_x_, gy - current_y_);
            progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;
            send_feedback(progress_, "Moving to " + marker_to);
        }

        // Threshold per completamento automatico
        else if (this->state_ == AlignState::ALIGN)
        {
            RCLCPP_INFO(get_logger(), "align state , roating for the alignement");
            if (!marker_visible_)
            {
                send_feedback(progress_, "Waiting for marker...");
            }
            else
            {
               
                double dx = live_marker_x_;
                double dy = live_marker_y_;

                double dist_error = std::hypot(dx, dy);
                double angle_error = std::atan2(dy, dx);

                // const double dist_th = 0.05;
                const double angle_th = 0.02;

                geometry_msgs::msg::Twist cmd;
                // cmd.linear.x = std::clamp(0.4 * dist_error, 0.0, 0.2);
                cmd.angular.z = 0.6 * angle_error;

                cmd_vel_pub_->publish(cmd);
                RCLCPP_INFO(get_logger(), "Navigation photo taken %d", angle_error);

                if (std::abs(angle_error) < angle_th)
                {
                    RCLCPP_INFO(get_logger(), "align state , dopo if");
                    stop_robot();
                    save_img();
                    progress_ = 1.0;
                    send_feedback(progress_, "Aligned to " + marker_to);
                    this->state_ = AlignState::NAVIGATE;
                    remove_first_marker_from_yaml();
                    RCLCPP_INFO(get_logger(), "Navigation photo taken");
                    marker_visible_ = false;
                    finish(true, 1.0, "Completed");
                    return;
                }
            }
        }
    }

    void save_img()
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_img_, sensor_msgs::image_encodings::BGR8);
            cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2), 50, CV_RGB(0, 255, 0), 3);
            // std::string dir_path = std::string(std::getenv("HOME")) + "/Desktop/Experimental/assignment2_ws/images";
            std::string ws_path = get_ws_path();
            std::string file_path = ws_path + "/images";
            if (!fs::exists(file_path))
            {
                fs::create_directories(file_path);
            }

            // std::ifstream infile(file_path);

            // rcpputils::fs::create_directories(dir_path);
            file_path = file_path + "/" + marker_target.name +"_" + std::to_string(marker_target.id) + ".png";
            cv::imwrite(file_path, cv_ptr->image);
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_ptr->toImageMsg();
            img_pub_->publish(*img_msg);

            RCLCPP_INFO(get_logger(), "Saved %s", file_path.c_str());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "Save image error");
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
        this->state_ = AlignState::NAVIGATE;
        goal_sent_ = false;
        nav2_done_ = false;
        marker_target.name = "";
        return plansys2::ActionExecutorClient::on_activate(previous_state);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd); // tutto a zero
    }

    float progress_;
    bool goal_sent_, nav2_done_, nav2_success_;
    double current_x_, current_y_;
    double start_x_ = 0.0, start_y_ = 0.0;
    MarkerData marker_target;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    sensor_msgs::msg::Image::SharedPtr last_img_;
    AlignState state_ = AlignState::NAVIGATE;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double live_marker_x_ = 0.0;
    double live_marker_y_ = 0.0;
    bool marker_visible_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPhotograph>();
    node->set_parameter(rclcpp::Parameter("action_name", "move_to_photograph_mode"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}