#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class ImuTest : public rclcpp::Node
{
public:
  ImuTest()
  : Node("imu_main_test") {
    publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 5);
    timer = this->create_wall_timer(
      10000ms, std::bind(&ImuTest::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = sensor_msgs::msg::Imu();

    for(int i = 0; i < 1000; i++) {
      message.linear_acceleration.x = 0.05;
      message.linear_acceleration.y = 0.05;
      message.linear_acceleration.z = 0.9;
      message.angular_velocity.x = 0.0;
      message.angular_velocity.y = 0.0;
      message.angular_velocity.z = 0.0;
      publisher->publish(message);
    }
    
    for(int i = 0; i < 1000; i++) {
      message.linear_acceleration.x = 0.0;
      message.linear_acceleration.y = -0.5;
      message.linear_acceleration.z = 0.5;
      message.angular_velocity.x = -2.0;
      message.angular_velocity.y = 0.0;
      message.angular_velocity.z = 0.0;
      publisher->publish(message);
    }
    
    for(int i = 0; i<1000; i++) {
      message.linear_acceleration.x = 0.0;
      message.linear_acceleration.y = 0.0;
      message.linear_acceleration.z = 1.0;
      message.angular_velocity.x = 2.0;
      message.angular_velocity.y = 0.0;
      message.angular_velocity.z = 0.0;
      publisher->publish(message);
    }
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ImuTest>());

  rclcpp::shutdown();
  return 0;
}
