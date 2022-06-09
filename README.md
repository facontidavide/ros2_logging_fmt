# ros2_logging_fmt

It is the same as [rclcpp logging](https://docs.ros.org/en/foxy/Tutorials/Logging-and-logger-configuration.html)
but much nicer, using [fmt](https://github.com/fmtlib/fmt) instead of **printf-like** formatting.

Advantages:

- More type safe that "printf-style" API.
- [Faster execution](https://github.com/fmtlib/fmt#speed-tests).
- Powerful syntax (see [examples](https://fmt.dev/latest/syntax.html#format-examples) ).
- Thread safe.
- It will not allocate any memory (unless for particularly long messages, and only once).

**NOTE: stil under construction. PR and suggestions are welcome.**

## Example

```c++
#include "ros2_logging_fmt/ros2_logging_fmt.hpp"

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

  rclcpp::shutdown();
  return 0;
}
```
