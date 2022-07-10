#include <cstdio>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class TurtleSimTeleopKeySubscriber : public rclcpp::Node
{
public:
  TurtleSimTeleopKeySubscriber()
    : Node("teleop_key_subscriber")
  {
    m_subscription = this->create_subscription<geometry_msgs::msg::Twist>(m_topicName,
        10, std::bind(&TurtleSimTeleopKeySubscriber::twistCallback, this, _1));
  }
private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    std::string printoutMsg = "Got command: linear={" 
      +  std::to_string(msg->linear.x) + ", "
      +  std::to_string(msg->linear.y) + ", "
      +  std::to_string(msg->linear.z) + "}, "
      + "angular={"
      + std::to_string(msg->angular.x) + ", "
      + std::to_string(msg->angular.y) + ", "
      + std::to_string(msg->angular.z) + "}";
    RCLCPP_INFO(this->get_logger(), printoutMsg.c_str());
    
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscription;
  std::string m_topicName{"/turtle1/cmd_vel"};
};

int main(int argc, char ** argv)
{
  printf("Hello world spot package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleSimTeleopKeySubscriber>());
  rclcpp::shutdown();
  return 0;
}
