#include "localizer_node.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizerNode>());
  rclcpp::shutdown();
  return 0;
}