#ifndef ROS2_LOGGING_PLUS_HPP
#define ROS2_LOGGING_PLUS_HPP

#include <rclcpp/rclcpp.hpp>
#define FMT_HEADER_ONLY
#include "ros2_logging_fmt/fmt/core.h"

namespace ros2_logging_fmt
{

class Logger
{
public:
  Logger(rclcpp::Logger rclcpp_logger): rclcpp_logger_(rclcpp_logger)
  {}

  template<typename... Args>
  void info(const char* str, Args... args)
  {
    RCLCPP_INFO(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void warn(const char* str, Args... args)
  {
    RCLCPP_WARN(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void error(const char* str, Args... args)
  {
    RCLCPP_ERROR(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void fatal(const char* str, Args... args)
  {
    RCLCPP_FATAL(rclcpp_logger_, write_buffer(str, args...) );
  }

  rclcpp::Logger rclcpp_logger_;
private:

  template<typename... Args>
  const char* write_buffer(const char* str, Args... args)
  {
    static thread_local std::string buffer;
    buffer.clear();
    fmt::format_to(std::back_inserter(buffer), str, args...);
    return buffer.c_str();
  }

};

}

#endif // ROS2_LOGGING_PLUS_HPP