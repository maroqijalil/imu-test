#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <imu_madgwick/imu.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto imu_main = std::make_shared<Imu>("imu_main");
  rclcpp::spin(imu_main);

  rclcpp::shutdown();
  return 0;
}