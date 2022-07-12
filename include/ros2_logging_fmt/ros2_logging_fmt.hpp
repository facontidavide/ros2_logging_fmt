#ifndef ROS2_LOGGING_PLUS_HPP
#define ROS2_LOGGING_PLUS_HPP

#include <rclcpp/rclcpp.hpp>

#define FMT_HEADER_ONLY
#include "fmt/format.h"

namespace ros2_logging_fmt
{

class Logger
{
public:
  Logger(rclcpp::Logger rclcpp_logger): rclcpp_logger_(rclcpp_logger)
  {}

  Logger(rclcpp::Logger rclcpp_logger, rclcpp::Clock::SharedPtr clock): rclcpp_logger_(rclcpp_logger)
  { clk = clock; }

  template<typename... Args>
  void info(const char* str, Args... args)
  {
    RCLCPP_INFO(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void info_throttle(const char* str, rclcpp::Clock::SharedPtr clock, int duration, Args... args)
  {
      RCLCPP_INFO_THROTTLE(rclcpp_logger_, *clock, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void info_throttle(int duration, const char* str, Args... args)
  {
      RCLCPP_INFO_THROTTLE(rclcpp_logger_, *clk, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void warn(const char* str, Args... args)
  {
    RCLCPP_WARN(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void warn_throttle(const char* str, rclcpp::Clock::SharedPtr clock, int duration, Args... args)
  {
      RCLCPP_WARN_THROTTLE(rclcpp_logger_, *clock, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void warn_throttle(int duration, const char* str, Args... args)
  {
      RCLCPP_WARN_THROTTLE(rclcpp_logger_, *clk, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void error(const char* str, Args... args)
  {
    RCLCPP_ERROR(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void error_throttle(const char* str, rclcpp::Clock::SharedPtr clock, int duration, Args... args)
  {
      RCLCPP_ERROR_THROTTLE(rclcpp_logger_, *clock, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void error_throttle(int duration, const char* str, Args... args)
  {
      RCLCPP_ERROR_THROTTLE(rclcpp_logger_, *clk, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void fatal(const char* str, Args... args)
  {
    RCLCPP_FATAL(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void fatal_throttle(const char* str, rclcpp::Clock::SharedPtr clock, int duration, Args... args)
  {
      RCLCPP_FATAL_THROTTLE(rclcpp_logger_, *clock, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void fatal_throttle(int duration, const char* str, Args... args)
  {
      RCLCPP_FATAL_THROTTLE(rclcpp_logger_, *clk, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void debug(const char* str, Args... args)
  {
    RCLCPP_DEBUG(rclcpp_logger_, write_buffer(str, args...) );
  }

  template<typename... Args>
  void debug_throttle(const char* str, rclcpp::Clock::SharedPtr clock, int duration, Args... args)
  {
      RCLCPP_DEBUG_THROTTLE(rclcpp_logger_, *clock, duration, write_buffer(str, args...) );
  }

  template<typename... Args>
  void debug_throttle(int duration, const char* str, Args... args)
  {
      RCLCPP_DEBUG_THROTTLE(rclcpp_logger_, *clk, duration, write_buffer(str, args...) );
  }

  rclcpp::Logger rclcpp_logger_;
private:

  rclcpp::Clock::SharedPtr clk;
  template<typename... Args>
  const char* write_buffer(const char* str, Args... args)
  {
     // static buffer with 500 characters pre-allocated
    static thread_local std::string buffer = [](){
      std::string tmp;
      tmp.reserve(500);
      return tmp;
    }();

    buffer.clear();
    fmt::format_to(std::back_inserter(buffer), str, args...);
    return buffer.c_str();
  }

};

}

#endif // ROS2_LOGGING_PLUS_HPP
