# ros2_logging_fmt

It is the same as [rclcpp logging](https://docs.ros.org/en/foxy/Tutorials/Logging-and-logger-configuration.html)
but much nicer, using [fmt](https://github.com/fmtlib/fmt) instead of **printf-like** formatting

## Example

```c++
#include "ros2_logging_fmt/ros2_logging_fmt.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node node("test_node");

  // the regular syntax
  RCLCPP_INFO(node.get_logger(), "Hello: %s %d", "old world", 42);
  RCLCPP_ERROR(node.get_logger(), "We have %d errors", 99);
  RCLCPP_WARN(node.get_logger(), "Warning: %f > %f", 30.1, 30.0);

  ros2_logging_fmt::Logger logger(node.get_logger());
  // nicer syntax
  logger.info("Hello: {} {}", "new world", 42);
  logger.error("We have {} errors", 99);
  logger.warn("Warning: {} > {}", 30.1, 30.0);

  rclcpp::shutdown();
  return 0;
}
```
