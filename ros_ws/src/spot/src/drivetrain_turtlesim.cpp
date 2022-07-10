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
#if 0
    std::string printoutMsg = "Got command: linear={" 
      +  std::to_string(msg->linear.x) + ", "
      +  std::to_string(msg->linear.y) + ", "
      +  std::to_string(msg->linear.z) + "}, "
      + "angular={"
      + std::to_string(msg->angular.x) + ", "
      + std::to_string(msg->angular.y) + ", "
      + std::to_string(msg->angular.z) + "}";
    RCLCPP_INFO(this->get_logger(), printoutMsg.c_str());
#endif
    // Because of the robot-centric coordinate system it uses, turtle_teleop_key
    // uses the following encoding scheme:
    // forward    := linear.x  ==  2
    // backward   := linear.x  == -2
    // turn left  := angular.z ==  2
    // turn right := angular.z == -2

    // The turtlesim can both turn and locomote at the same time. My robot can
    // only do one or ther other. Using the turtle_teleop_key node only lets one
    // or the other through.

    int forwardMilliseconds = msg->linear.x * 125;
    int turningMilliseconds = msg->angular.z * 75;

    if (forwardMilliseconds > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Driving forward for %d ms", forwardMilliseconds);
    }
    if (forwardMilliseconds < 0)
    {
      RCLCPP_INFO(this->get_logger(), "Driving backward for %d ms", abs(forwardMilliseconds));
    }
    if (turningMilliseconds > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Turning right for %d ms", turningMilliseconds);
    }
    if (turningMilliseconds < 0)
    {
      RCLCPP_INFO(this->get_logger(), "Turning left for %d ms", abs(turningMilliseconds));
    }
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
