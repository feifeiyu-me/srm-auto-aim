#pragma once
#include <chrono>
#include <optional>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "srm_armor_locator/locator.hpp"
namespace srm_armor_locator {
// Bounded samples plus streaming statistics. No per-frame console output or file I/O.
class IppeDebug {
public:
  using Clock=std::chrono::steady_clock;
  IppeDebug(double interval_sec, int max_samples, const Parameters & p,
    Clock::time_point now=Clock::now());
  void observe(const std_msgs::msg::Header & header, size_t index,
    const IppeTrace & trace, const PoseResult & result);
  std::optional<diagnostic_msgs::msg::DiagnosticArray> flush(Clock::time_point now);
private:
  struct Metric {
    uint64_t count{0}; double sum{0}, min{0}, max{0};
    void add(double value);
  };
  struct Statistics {
    uint64_t attempts{0}, accepted{0}, ambiguous{0}, zero{0}, one{0}, pairs{0};
    uint64_t angle_only{0}, translation_only{0}, both{0}, trace_errors{0}, omitted{0};
    Metric gap, separation, distance_difference, angle;
    void add(const IppeTrace & trace, const PoseResult & result);
  };
  double interval_; int max_samples_; Parameters p_; Clock::time_point last_;
  Statistics total_,window_; std_msgs::msg::Header header_;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> samples_;
};
}
