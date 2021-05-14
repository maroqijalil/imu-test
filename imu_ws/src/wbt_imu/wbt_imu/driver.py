import rclpy
from webots_ros2_core.webots_node import WebotsNode


class MyWebotsDriver(WebotsNode):
  def __init__(self, args):
    super().__init__('my_webots_driver', args=args)


def main(args=None):
  rclpy.init(args=args)
  my_webots_driver = MyWebotsDriver(args=args)
  rclpy.spin(my_webots_driver)
  my_webots_driver.destroy()
  rclpy.shutdown()


if __name__ == '__main__':
  main()