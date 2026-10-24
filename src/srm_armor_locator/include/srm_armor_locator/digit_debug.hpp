#pragma once
#include <chrono>
#include <filesystem>
#include <std_msgs/msg/header.hpp>
#include "srm_armor_locator/locator.hpp"
namespace srm_armor_locator {
// Observation-only writer. Budget counts attempts; a write failure disables it.
class DigitDebugWriter {
public:
  using Clock=std::chrono::steady_clock;
  DigitDebugWriter(bool enabled, const std::string & directory, int limit, double interval);
  bool wants_sample(Clock::time_point now) const;
  bool save(const cv::Mat & gray, const Corners & corners, const cv::Mat & templ,
    const DigitTrace & trace, const std_msgs::msg::Header & header, size_t input_index,
    double score, double threshold, const std::string & template_file, Clock::time_point now);
  const std::filesystem::path & directory() const {return session_;}
  int attempts() const {return attempts_;}
private:
  bool enabled_, failed_{false}; int limit_, attempts_{0}; double interval_;
  Clock::time_point last_{}; std::filesystem::path session_;
};
}
