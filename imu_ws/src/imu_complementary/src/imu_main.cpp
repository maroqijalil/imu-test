#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <imu_complementary/imu.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto imu_main = std::make_shared<imu_tools::Imu>("imu_main");
  rclcpp::spin(imu_main);

  rclcpp::shutdown();
  return 0;
}