#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string odom_topic = "/ego_racecar/odom";
std::string scan_topic = "/scan";
std::string publish_topic = "/drive";
class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("rom_safety_node")
    {
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, std::bind(&Safety::scan_callback, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, std::bind(&Safety::odom_callback, this, _1));

        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(publish_topic, 10);

        timer_ = this->create_wall_timer( 500ms, std::bind(&Safety::timer_callback, this) );

        brake_msg_.header.stamp = this->now();
        brake_msg_.drive.speed = 0.0;
        brake_msg_.drive.steering_angle = 0.0;
    }

private:
    //double speed = 0.0;
    ackermann_msgs::msg::AckermannDriveStamped brake_msg_;
    bool should_brake_ = false;

    double velocity_x_ = 0.0;
    double velocity_y_ = 0.0;

    //sensor_msgs::msg::LaserScan::ConstSharedPtr lidar_info_local_;
    //nav_msgs::msg::Odometry::ConstSharedPtr odom_info_local_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void timer_callback()
    {
      //auto message;
      if(should_brake_ == true)
      {
        ackermann_msgs::msg::AckermannDriveStamped brake_msg;
        brake_msg.header.stamp = this->now();
        ackermann_pub_->publish(brake_msg);
      }
    }
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        auto msg_ = msg;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        double TTC_threshold = 0.4;
        double min_TTC = 100;
        double v_x = velocity_x_;
        double v_y = velocity_y_;
        
        for (unsigned int i = 0; i < scan_msg->ranges.size(); i++) 
        {
            if (!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i])) 
            {
                double distance = scan_msg->ranges[i];
                double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
                double distance_derivative = cos(angle) * v_x + sin(angle) * v_y;
                if (distance_derivative > 0 && distance / distance_derivative < min_TTC) min_TTC = distance / distance_derivative;
            }
        }
        if (min_TTC <= TTC_threshold) {
            should_brake_ = true;
        } else {
            should_brake_ = false;
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        velocity_x_ = odom_msg->twist.twist.linear.x;
        velocity_y_ = odom_msg->twist.twist.linear.y;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}