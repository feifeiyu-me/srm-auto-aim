#pragma once

#include <array>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace srm_armor_refiner
{
// Semantic order, invariant under rotation: LT, LB, RB, RT.
using Corners = std::array<cv::Point2f, 4>;

struct Parameters
{
  double roi_half_width_px{8.0};
  double min_contrast{15.0};
  int min_pixels{8};
  double min_light_length_px{8.0};
  double min_pca_ratio{4.0};
  double max_axis_angle_deg{20.0};
  double max_shift_px{8.0};
  double max_shift_fraction{0.25};
  double min_quad_area{16.0};
  double min_edge_length{2.0};
  double max_edge_ratio{20.0};
  int max_roi_pixels{65536};
  void validate() const;
};

enum class Outcome { Refined, Fallback, Dropped };

struct Result
{
  Outcome outcome{Outcome::Dropped};
  // Only consume corners if outcome is not Dropped.
  Corners corners{};
  std::string reason;
  double max_shift_px{0.0};
};

bool valid_corners(const Corners & corners, cv::Size image_size, const Parameters & p);
Result refine(const cv::Mat & gray, const Corners & initial, const Parameters & p = {});
std::vector<Result> refine_batch(
  const cv::Mat & gray, const std::vector<Corners> & initial, const Parameters & p = {});
}  // namespace srm_armor_refiner
