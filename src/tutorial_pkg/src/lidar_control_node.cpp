#include <chrono>

#include "rclcpp/rclcpp.hpp"
//#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
//#include "std_srvs/srv/empty.hpp"
//#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MyNode : public rclcpp::Node
{
  public:
    MyNode() : Node("lidar_node")
    {
      declare_parameter("timer_period_s", 5);
      auto timer_period_s = std::chrono::seconds(get_parameter("timer_period_s").as_int());
      subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&MyNode::lidar_callback, this, _1));
      publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
      //timer_ = create_wall_timer(timer_period_s, std::bind(&MyNode::timer_callback, this));
      //client_ = create_client<std_srvs::srv::Empty>("/save");
      //server_ = create_service<std_srvs::srv::Trigger>("/image_counter", std::bind(&MyNode::counter_callback, this, _1, _2));

      //subscriber_ = this->create_subscription<std_msgs::msg::String>(
      //  "topic", 10, std::bind(&MyNode::topic_callback, this, _1));
      RCLCPP_INFO(get_logger(), "Lidar node started!");
    }
  private:
    //bool is_front_stuck = 0;
    void lidar0_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
      // deg: 0~39(front), 40~119(right), 120~239(dont care), 240~319(left), 320~359(front)
      // laser_point: 0~79(front), 80~239(left), 240~479(dont care), 480~639(right), 640~719(front)
      float range_total[3]  = {0, 0, 0};
      float range_number[3] = {0, 0, 0};
      float range_avg[3]    = {0, 0, 0};
      float vel_x = 0, ang_z = 0;
      bool is_front_stuck = 0;
      for (int i=0; i<int(laser->ranges.size()); i++) {
        //int j = (718 + i) % 720;
        //float range = laser->ranges[j];
        //if (std::isfinite(range)) {
        //  range_total += range;
        //  range_number++;
        //}
        float range = laser->ranges[i];
        //RCLCPP_INFO(get_logger(), "range[%d]: %f", i, range);
        if (std::isfinite(range)) {
          if ((i < 150) || (i >= 570)) {
            range_total[1]  += range;
            range_number[1] +=     1;
            if (range < 0.3) is_front_stuck = 1;
          }
          else if ((i < 240) && (i >= 150)) {
            range_total[0]  += range;
            range_number[0] +=     1;
          }
          else if ((i < 570) && (i >= 480)) {
            range_total[2]  += range;
            range_number[2] +=     1;
            //RCLCPP_INFO(get_logger(), "range[%d]: %f", i, range);
          }
        }
        else {
          if ((i < 150) || (i >= 570)) {
            range_total[1]  += 0;
            range_number[1] += 1;
          }
          else if ((i < 240) && (i >= 150)) {
            range_total[0]  += 0;
            range_number[0] += 1;
          }
          else if ((i < 570) && (i >= 480)) {
            range_total[2]  += 0;
            range_number[2] += 1;
          }
        }
      }
      //while (1) {}
      //bool is_front_stuck = 0;
      //float max_sense_distance = 0.5;
      for (int i=0; i<3; i++) {
        if (range_number[i] != 0) range_avg[i] = range_total[i] / range_number[i];
        //if (range_avg[i] > max_sense_distance) range_avg[i] = max_sense_distance;
        //else if (range_avg[i] < 0.1) range_avg[i] = 0;
        //float distance_percentage = range_avg[i] / max_sense_distance;
        switch (i) {
          case 0:
            if (range_avg[i] > 0.5) range_avg[i] = 0.5;
            ang_z += (range_avg[i] / 0.5);
            break;
          case 1:
            if (range_avg[i] > 2.0) range_avg[i] = 2.0;
            if (range_avg[i] < 0.4) {
              vel_x = 0;
              is_front_stuck = 1;
            }
            else vel_x += (range_avg[i] / 2.0);
            break;
          case 2:
            if (range_avg[i] > 0.5) range_avg[i] = 0.5;
            ang_z += -(range_avg[i] / 0.5);
            break;
        }
      }

      geometry_msgs::msg::Twist control_msg;
      control_msg.linear.x  = vel_x * 0.2;
      if (is_front_stuck) control_msg.angular.z = 1;
      else control_msg.angular.z = ang_z * 1;
      //control_msg.angular.z = ang_z * 1;
      publisher_->publish(control_msg);
      RCLCPP_INFO(get_logger(), "(stuck, left, center, right, vel_x, ang_z): (%d, %f, %f, %f, %f, %f)", is_front_stuck, range_avg[0], range_avg[1], range_avg[2], vel_x, ang_z);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
    {
      // deg: 0~39(front), 40~119(right), 120~239(dont care), 240~319(left), 320~359(front)
      // laser_point: 0~79(front), 80~239(left), 240~479(dont care), 480~639(right), 640~719(front)
      float range_total[5]  = {0, 0, 0, 0, 0};
      float range_number[5] = {0, 0, 0, 0, 0};
      float range_avg[5]    = {0, 0, 0, 0, 0};
      float vel_x = 0, ang_z = 0;
      //bool is_front_stuck = 0;
      for (int i=0; i<5; ++i) {
        for (int j=0; j<96; ++j) {
          int k = 480 + (96 * i) + j;
          if (k >= 720) k -= 720;
          float range = laser->ranges[k];
          //RCLCPP_INFO(get_logger(), "ranges[%d]: %f", k, range);
          if (std::isfinite(range)) {
            range_total[i]  += range;
            range_number[i] +=     1;
          }
          else {
            range_total[i]  +=     0;
            range_number[i] +=     0;
          }
          //range_number[i] += 1;
        }
      }
      for (int i=0; i<5; i++) {
        if (range_number[i] != 0) range_avg[i] = range_total[i] / range_number[i];
        switch (i) {
          case 0:
            if (range_avg[i] > 0.5) range_avg[i] = 0.5;
            ang_z += -(range_avg[i] / 0.5) * 0.5;
            break;
          case 1:
            if (range_avg[i] > 1.0) range_avg[i] = 1.0;
            vel_x += (range_avg[i] / 1.0) * 0.2;
            ang_z += -(range_avg[i] / 1.0) * 0.5;
            break;
          case 2:
            if (range_avg[i] > 2.0) range_avg[i] = 2.0;
            vel_x += (range_avg[i] / 2.0) * 0.6;
            break;
          case 3:
            if (range_avg[i] > 1.0) range_avg[i] = 1.0;
            vel_x += (range_avg[i] / 1.0) * 0.2;
            ang_z += (range_avg[i] / 1.0) * 0.5;
            break;
          case 4:
            if (range_avg[i] > 0.5) range_avg[i] = 0.5;
            ang_z += (range_avg[i] / 0.5) * 0.5;
            break;
        }
      }
      geometry_msgs::msg::Twist control_msg;
      control_msg.linear.x  = vel_x * 0.2;
      control_msg.angular.z = ang_z * 1.0;
      if (range_avg[2] < 0.4) {
        control_msg.linear.x  = 0;
        control_msg.angular.z = 1;
      }
      //control_msg.linear.x  =  0;
      //control_msg.angular.z = -1;
      publisher_->publish(control_msg);
      //RCLCPP_INFO(get_logger(), "tot(L, DL, C, DR, R): (%f, %f, %f, %f, %f)", range_total[4], range_total[3], range_total[2], range_total[1], range_total[0]);
      RCLCPP_INFO(get_logger(), "(L, DL, C, DR, R, vel_x, ang_z): (%f, %f, %f, %f, %f, %f, %f)", range_avg[4], range_avg[3], range_avg[2], range_avg[1], range_avg[0], control_msg.linear.x, control_msg.angular.z);
    }
/*/
    void timer_callback()
    {
      RCLCPP_INFO(get_logger(), "Timer activate");
      if (!client_->wait_for_service(1s))
      {
        RCLCPP_ERROR(get_logger(), "Failed to connect to the image save service");
        return;
      }
      saved_imgs_++;
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = client_->async_send_request(request);
    }
/*/
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    //rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
    //rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
/*/
    void  topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
/*/
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node =std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

