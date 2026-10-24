#include "srm_armor_refiner/refiner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace srm_armor_refiner
{
void Parameters::validate() const
{
  for (double v : {roi_half_width_px, min_contrast, min_light_length_px, min_pca_ratio,
      max_axis_angle_deg, max_shift_px, max_shift_fraction, min_quad_area,
      min_edge_length, max_edge_ratio})
  {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("Refinement parameters must be finite and positive");
    }
  }
  if (min_pixels < 3 || max_roi_pixels < min_pixels || roi_half_width_px < 1 ||
    min_contrast > 255 || min_pca_ratio <= 1 || max_axis_angle_deg >= 90 ||
    max_shift_fraction > 0.5 || max_edge_ratio < 1)
  {
    throw std::invalid_argument("Refinement parameter outside supported range");
  }
}

bool valid_corners(const Corners & c, cv::Size size, const Parameters & p)
{
  double minimum = std::numeric_limits<double>::max(), maximum = 0, area = 0;
  for (size_t i = 0; i < 4; ++i) {
    const auto & a = c[i];
    const auto & b = c[(i + 1) % 4];
    const auto & d = c[(i + 2) % 4];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || a.x < 0 || a.y < 0 ||
      a.x >= size.width || a.y >= size.height)
    {
      return false;
    }
    const cv::Point2d edge = cv::Point2d(b) - cv::Point2d(a);
    const cv::Point2d next = cv::Point2d(d) - cv::Point2d(b);
    if (edge.cross(next) >= -1e-6) {return false;}
    const double length = cv::norm(edge);
    minimum = std::min(minimum, length);
    maximum = std::max(maximum, length);
    area += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
  }
  return minimum >= p.min_edge_length && maximum / minimum <= p.max_edge_ratio &&
         -area / 2 >= p.min_quad_area;
}

namespace
{
struct Light
{
  bool ok{false};
  cv::Point2f top, bottom;
  std::string reason;
};

Light refine_light(
  const cv::Mat & gray, cv::Point2d top, cv::Point2d bottom, const Parameters & p)
{
  const double length = cv::norm(bottom - top);
  if (length < p.min_light_length_px) {return {false, {}, {}, "light_too_short"};}
  const cv::Point2d initial = (bottom - top) / length;
  const cv::Point2d normal(-initial.y, initial.x);
  const cv::Point2d center = (top + bottom) / 2;
  const double extent = length / 2 + p.max_shift_px + 2;
  std::vector<cv::Point2f> bounds;
  for (double a : {-extent, extent}) {
    for (double b : {-p.roi_half_width_px, p.roi_half_width_px}) {
      bounds.emplace_back(center + a * initial + b * normal);
    }
  }
  for (const auto & point : bounds) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0 || point.y < 0 ||
      point.x >= gray.cols || point.y >= gray.rows)
    {
      return {false, {}, {}, "roi_outside_image"};
    }
  }
  const cv::Rect roi = cv::boundingRect(bounds);
  if ((roi & cv::Rect(0, 0, gray.cols, gray.rows)) != roi) {
    return {false, {}, {}, "roi_outside_image"};
  }
  if (static_cast<int64_t>(roi.width) * roi.height > p.max_roi_pixels) {
    return {false, {}, {}, "roi_too_large"};
  }
  cv::Mat corridor = cv::Mat::zeros(roi.size(), CV_8U);
  std::vector<uchar> intensities;
  for (int y = 0; y < roi.height; ++y) {
    for (int x = 0; x < roi.width; ++x) {
      const cv::Point2d delta = cv::Point2d(x + roi.x, y + roi.y) - center;
      if (std::abs(delta.dot(initial)) <= extent &&
        std::abs(delta.dot(normal)) <= p.roi_half_width_px)
      {
        corridor.at<uchar>(y, x) = 255;
        intensities.push_back(gray.at<uchar>(y + roi.y, x + roi.x));
      }
    }
  }
  if (intensities.size() < static_cast<size_t>(p.min_pixels)) {
    return {false, {}, {}, "insufficient_pixels"};
  }
  auto sorted = intensities;
  std::sort(sorted.begin(), sorted.end());
  if (sorted[static_cast<size_t>(0.98 * (sorted.size() - 1))] -
    sorted[static_cast<size_t>(0.10 * (sorted.size() - 1))] < p.min_contrast)
  {
    return {false, {}, {}, "low_contrast"};
  }
  cv::Mat unused;
  const double threshold = cv::threshold(
    cv::Mat(intensities), unused, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::Mat mask;
  cv::threshold(gray(roi), mask, threshold, 255, cv::THRESH_BINARY);
  cv::bitwise_and(mask, corridor, mask);
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  struct Candidate {double cost; cv::Point2d mean, axis; double low, high;};
  std::vector<Candidate> candidates;
  std::string rejected = "no_bright_component";
  const double cosine = std::cos(p.max_axis_angle_deg * CV_PI / 180);
  std::vector<std::vector<cv::Point2d>> components(count);
  for (int y = 0; y < labels.rows; ++y) {
    for (int x = 0; x < labels.cols; ++x) {
      const int label = labels.at<int>(y, x);
      if (label > 0 && stats.at<int>(label, cv::CC_STAT_AREA) >= p.min_pixels) {
        components[label].emplace_back(x + roi.x, y + roi.y);
      }
    }
  }
  for (int label = 1; label < count; ++label) {
    if (stats.at<int>(label, cv::CC_STAT_AREA) < p.min_pixels) {continue;}
    const auto & pixels = components[label];
    const bool clipped = std::any_of(pixels.begin(), pixels.end(), [&](const auto & pixel) {
      const auto delta = pixel - center;
      return std::abs(delta.dot(initial)) >= extent - 0.75 ||
             std::abs(delta.dot(normal)) >= p.roi_half_width_px - 0.75;
    });
    if (clipped) {rejected = "component_touches_roi"; continue;}
    cv::Mat data(static_cast<int>(pixels.size()), 2, CV_64F);
    for (size_t i = 0; i < pixels.size(); ++i) {
      data.at<double>(static_cast<int>(i), 0) = pixels[i].x;
      data.at<double>(static_cast<int>(i), 1) = pixels[i].y;
    }
    const cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (pca.eigenvalues.at<double>(0) <
      p.min_pca_ratio * std::max(pca.eigenvalues.at<double>(1), 0.1))
    {
      rejected = "insufficient_pca";
      continue;
    }
    const cv::Point2d mean(pca.mean.at<double>(0, 0), pca.mean.at<double>(0, 1));
    cv::Point2d axis(pca.eigenvectors.at<double>(0, 0), pca.eigenvectors.at<double>(0, 1));
    if (axis.dot(initial) < 0) {axis *= -1;}
    if (axis.dot(initial) < cosine) {rejected = "axis_mismatch"; continue;}
    const cv::Point2d delta = mean - center;
    if (std::abs(delta.dot(initial)) > 0.3 * length ||
      std::abs(delta.dot(normal)) > p.roi_half_width_px)
    {
      rejected = "component_off_center";
      continue;
    }
    double low = std::numeric_limits<double>::max(), high = -low;
    for (const auto & pixel : pixels) {
      const double projection = (pixel - mean).dot(axis);
      low = std::min(low, projection);
      high = std::max(high, projection);
    }
    if (high - low < 0.6 * length || high - low > 1.4 * length) {
      rejected = "length_mismatch";
      continue;
    }
    candidates.push_back({cv::norm(delta) / length + std::abs((high - low) / length - 1),
      mean, axis, low, high});
  }
  if (candidates.empty()) {return {false, {}, {}, rejected};}
  std::sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) {return a.cost < b.cost;});
  if (candidates.size() > 1 && candidates[1].cost - candidates[0].cost < 0.05) {
    return {false, {}, {}, "ambiguous_components"};
  }
  const auto & best = candidates.front();
  return {true, cv::Point2f(best.mean + best.low * best.axis),
    cv::Point2f(best.mean + best.high * best.axis), "pca"};
}
}  // namespace

Result refine(const cv::Mat & gray, const Corners & initial, const Parameters & p)
{
  p.validate();
  if (gray.empty() || gray.type() != CV_8UC1) {return {Outcome::Dropped, {}, "invalid_image", 0};}
  if (!valid_corners(initial, gray.size(), p)) {
    return {Outcome::Dropped, {}, "invalid_yolo_geometry", 0};
  }
  auto fallback = [&](const std::string & reason, double shift = 0) {
      return Result{Outcome::Fallback, initial, reason, shift};
    };
  try {
    const auto left = refine_light(gray, initial[0], initial[1], p);
    if (!left.ok) {return fallback("left_" + left.reason);}
    const auto right = refine_light(gray, initial[3], initial[2], p);
    if (!right.ok) {return fallback("right_" + right.reason);}
    const Corners candidate{left.top, left.bottom, right.bottom, right.top};
    double shift = 0;
    for (size_t i = 0; i < 4; ++i) {
      shift = std::max(shift, cv::norm(candidate[i] - initial[i]));
    }
    const double limit = std::min(p.max_shift_px, p.max_shift_fraction * std::min(
        cv::norm(initial[1] - initial[0]), cv::norm(initial[2] - initial[3])));
    if (!std::isfinite(shift) || shift > limit) {return fallback("max_shift_exceeded", shift);}
    if (!valid_corners(candidate, gray.size(), p)) {
      return fallback("invalid_refined_geometry", shift);
    }
    return {Outcome::Refined, candidate, "pca", shift};
  } catch (const cv::Exception &) {
    return fallback("opencv_error");
  }
}

std::vector<Result> refine_batch(
  const cv::Mat & gray, const std::vector<Corners> & initial, const Parameters & p)
{
  std::vector<Result> results;
  results.reserve(initial.size());
  for (const auto & corners : initial) {results.push_back(refine(gray, corners, p));}
  return results;
}
}  // namespace srm_armor_refiner
