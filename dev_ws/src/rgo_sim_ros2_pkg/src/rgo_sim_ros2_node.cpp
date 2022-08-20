#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
//using std::placeholders::_2;


class RgoRos2Node : public rclcpp::Node
{
  public:
    RgoRos2Node ()
    : Node("rgo_ros2_node"), count_(0)
    {            
      //create subscribers 
      subscriber_         = this->create_subscription<geometry_msgs::msg::Twist>("robot_sim_velocity", 1, std::bind(&RgoRos2Node::subscribe_message, this, _1));
      odometry_subscriber_   = this->create_subscription<nav_msgs::msg::Odometry>("robot_sim_odometry", 1, std::bind(&RgoRos2Node::subscribe_odometry_message, this, _1));
      
      //create publishers
      publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("rgo_pos",1);
      _pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rgo_pose_with_cov", 1);
	    _absolute_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/absolute_position", 1);
      
      timer_ = this->create_wall_timer(
      500ms, std::bind(&RgoRos2Node::timer_callback, this));
      
      i = 0.0;
    }

  private:
    float i;
    //publisher smart pointers 
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _absolute_pose_publisher;
    
    //subscriber smart pointers 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;    
    rclcpp::TimerBase::SharedPtr timer_;
    
    size_t count_;

    //void 
    
    void subscribe_odometry_message(const nav_msgs::msg::Odometry::SharedPtr message) const
    {                
        float x= message->twist.twist.linear.x;
        float z=message->twist.twist.angular.z;
        RCLCPP_INFO(this->get_logger(), "Odometry Linear Velocity : '%f', Angular Velocity : '%f'", x, z);
    }

    void subscribe_message(const geometry_msgs::msg::Twist::SharedPtr message) const
    {
        //RCLCPP_INFO(this->get_logger(), "Twist Linear Velocity : '%f', Angular Velocity : '%f'", message->linear.x, message->angular.z);
    }
      
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Pose();            
      message.position.x = 4.0 + i; 
      message.position.y = 3.0 + i*2;
      message.position.z = 2.0 + i*3;
      //RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", message.position.x, message.position.y,message.position.z);
      publisher_->publish(message);
      i += 0.1; 
      
      //geometry_msgs::msg::PoseWithCovarianceStamped pose{};
      auto pose = geometry_msgs::msg::PoseWithCovarianceStamped();      
      pose.pose.pose.position.x = 4.0 + i; 
      pose.pose.pose.position.y = 3.0 + i*2;
      pose.pose.pose.position.z = 2.0 + i*3;
      pose.pose.pose.orientation.x = 0.0 + i*1;
      pose.pose.pose.orientation.y = 1.0 + i*1;
      pose.pose.pose.orientation.z = 2.0 + i*1;
      pose.pose.pose.orientation.w = 3.0 + i*1;
      //RCLCPP_INFO(this->get_logger(), "Sending - position x: '%f', y: '%f', z: '%f' ", pose.pose.pose.position.x, pose.pose.pose.orientation.x,pose.pose.pose.position.z);
      
	    _pose_publisher->publish(pose);

    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgoRos2Node>());
  rclcpp::shutdown();
  return 0;
}