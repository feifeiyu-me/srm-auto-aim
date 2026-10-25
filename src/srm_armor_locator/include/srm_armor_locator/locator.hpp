#pragma once
#include <array>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <srm_armor_refiner/refiner.hpp>
namespace srm_armor_locator {
using Corners = srm_armor_refiner::Corners;
struct Calibration {
  cv::Size size;
  cv::Mat k, d;
  static Calibration load(const std::string & path);
};
struct Parameters {
  double width_m{0.135}, height_m{0.058};
  double min_depth_m{0.1}, max_depth_m{20.0}, max_error_px{3.0};
  double ambiguity_gap_px{0.1}, ambiguity_translation_m{0.01}, ambiguity_angle_deg{5.0};
  void validate() const;
};
struct PoseResult {
  bool valid{false}, ambiguous{false};
  std::string reason;
  cv::Vec3d rotation{0,0,0}, translation{0,0,0};
  double error{-1};
};
std::vector<cv::Point3d> object_points(const Parameters & p);
struct DigitTrace {
  cv::Mat perspective, roi, best_template, best_patch;
  cv::Rect best_rect;
  double roi_stddev{0}, best_squeeze{0};
  std::string reason{"invalid_input"};
};
double digit_score(const cv::Mat & gray, const Corners & corners, const cv::Mat & templ,
  DigitTrace * trace = nullptr);
// Observation only: candidates sorted by RMS, after the existing validity filters.
struct IppeCandidate {
  cv::Vec3d translation{0,0,0};
  double error_px{-1}, distance_mm{-1};
};
struct IppeTrace {
  size_t valid_candidates{0};
  bool pair_available{false}, diagnostic_error{false};
  std::array<IppeCandidate,2> candidates;
  cv::Vec3d translation_delta_mm{0,0,0}; // second minus best
  double error_gap_px{-1}, translation_separation_mm{-1};
  double signed_distance_difference_mm{0}, rotation_difference_deg{-1};
  bool error_gap_close{false}, translation_exceeds{false}, rotation_exceeds{false};
};
PoseResult solve(const Calibration & calibration, const Corners & corners,
  const Parameters & p = {}, IppeTrace * trace = nullptr);
// Optional position retention is separate from strict PnP selection/rejection.
struct PositionPolicy {
  bool allow_rotation_ambiguous_position{false};
  double max_translation_difference_m{0.002};
  void validate() const;
};
struct LocationResult {
  PoseResult strict_pose;
  bool position_valid{false}, orientation_valid{false};
  cv::Vec3d position{0,0,0};
  std::string position_reason;
};
LocationResult solve_location(const Calibration & calibration, const Corners & corners,
  const Parameters & p = {}, const PositionPolicy & policy = {}, IppeTrace * trace = nullptr);

}
