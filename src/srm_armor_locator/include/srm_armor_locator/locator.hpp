#pragma once
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
double digit_score(const cv::Mat & gray, const Corners & corners, const cv::Mat & templ);
PoseResult solve(const Calibration & calibration, const Corners & corners,
  const Parameters & p = {});
}
