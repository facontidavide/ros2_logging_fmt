#include "ros2_logging_fmt/ros2_logging_fmt.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("test_node");

  std::string world = "world";

  RCLCPP_INFO(node.get_logger(), "Hello %s number %d", world.c_str(), 42);
  RCLCPP_ERROR(node.get_logger(), "We have %d errors", 99);
  RCLCPP_WARN(node.get_logger(), "Warning: %f > %f", 30.1, 30.0);
  RCLCPP_DEBUG(node.get_logger(), "DEBUG MESSAGE");


  ros2_logging_fmt::Logger logger(node.get_logger());

  logger.info("Hello {} number {}", world, 42);
  logger.error("We have {} errors", 99);
  logger.warn("Warning: {} > {}", 30.1, 30.0);
  logger.debug("DEBUG MESSAGE");
  logger.info_throttle(*node.get_clock(), 1000ms, "Hello {} number {}", world, 42);
  logger.error_throttle(*node.get_clock(), 1000ms, "We have {} errors", 99);
  logger.warn_throttle(*node.get_clock(), 1000ms, "Warning: {} > {}", 30.1, 30.0);
  logger.debug_throttle(*node.get_clock(),1000ms, "DEBUG MESSAGE");


  ros2_logging_fmt::Logger logger_clk(node.get_logger(), node.get_clock());

  logger_clk.info_throttle(1h, "Hello {} number {}", world, 42);
  logger_clk.error_throttle(1000ms, "We have {} errors", 99);
  logger_clk.warn_throttle(10s, "Warning: {} > {}", 30.1, 30.0);
  logger_clk.debug_throttle(5ms, "DEBUG MESSAGE");


  ros2_logging_fmt::Logger logger_node(node);
  logger_node.info("Hi there!");
  logger_node.info_throttle(std::chrono::seconds{1}, "Hi there!");


  rclcpp_lifecycle::LifecycleNode lf_node("test_lifecycle_node");
  ros2_logging_fmt::Logger logger_lf_node(lf_node);
  logger_lf_node.info("Hi there!");
  logger_lf_node.info_throttle(1000ms, "Hi there!");

  rclcpp::shutdown();
  return 0;
}
