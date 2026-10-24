#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include "srm_armor_refiner/refiner.hpp"

using namespace srm_armor_refiner;

namespace
{
Corners truth(float x = 100)
{return {{{x, 100}, {x, 200}, {x + 100, 200}, {x + 100, 100}}};}
cv::Mat bars(const Corners & corners)
{
  cv::Mat gray = cv::Mat::zeros(500, 700, CV_8UC1);
  cv::line(gray, corners[0], corners[1], 220, 5, cv::LINE_8);
  cv::line(gray, corners[3], corners[2], 220, 5, cv::LINE_8);
  return gray;
}
Corners shifted(Corners corners, cv::Point2f offset)
{for (auto & p : corners) {p += offset;} return corners;}
void expect_same(const Corners & a, const Corners & b)
{for (size_t i = 0; i < 4; ++i) {EXPECT_EQ(a[i], b[i]);}}
}  // namespace

TEST(Refiner, BrightBarsProduceBoundedCandidateAndKeepSemantics)
{
  auto original = shifted(truth(), {2, 1});
  const auto result = refine(bars(truth()), original);
  ASSERT_EQ(result.outcome, Outcome::Refined) << result.reason;
  EXPECT_LE(result.max_shift_px, 8);
  EXPECT_NEAR(result.corners[0].x, 100, 0.1);
  EXPECT_NEAR(result.corners[1].x, 100, 0.1);
  EXPECT_NEAR(result.corners[2].x, 200, 0.1);
  EXPECT_NEAR(result.corners[3].x, 200, 0.1);
  EXPECT_LT(result.corners[0].y, result.corners[1].y);
  EXPECT_LT(result.corners[3].y, result.corners[2].y);
  EXPECT_TRUE(valid_corners(result.corners, {700, 500}, {}));
}

TEST(Refiner, RotatedAndInvertedBarsDoNotSortByScreenY)
{
  for (double angle : {35.0, 100.0, 180.0}) {
    Corners rotated = truth();
    const cv::Point2f origin(150, 150), destination(350, 250);
    for (auto & point : rotated) {
      const auto v = point - origin;
      const double a = angle * CV_PI / 180;
      point = destination + cv::Point2f(
        std::cos(a) * v.x - std::sin(a) * v.y,
        std::sin(a) * v.x + std::cos(a) * v.y);
    }
    const auto result = refine(bars(rotated), shifted(rotated, {1, 1}));
    ASSERT_EQ(result.outcome, Outcome::Refined) << angle << ' ' << result.reason;
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_LT(cv::norm(result.corners[i] - rotated[i]), 5);
    }
  }
}

TEST(Refiner, UniformAndLowContrastImagesFallBackExactly)
{
  for (int intensity : {0, 127, 255}) {
    const cv::Mat gray(500, 700, CV_8UC1, cv::Scalar(intensity));
    auto result = refine(gray, truth());
    ASSERT_EQ(result.outcome, Outcome::Fallback);
    EXPECT_EQ(result.reason, "left_low_contrast");
    expect_same(result.corners, truth());
  }
  cv::Mat dim = bars(truth()) / 30;
  EXPECT_EQ(refine(dim, truth()).outcome, Outcome::Fallback);
}

TEST(Refiner, OneMissingLightFallsBackWholeArmorNotJustOneSide)
{
  auto gray = bars(truth());
  gray(cv::Rect(180, 80, 40, 150)).setTo(0);
  const auto original = shifted(truth(), {1, 0});
  const auto result = refine(gray, original);
  ASSERT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "right_low_contrast");
  expect_same(result.corners, original);
}

TEST(Refiner, ShiftGuardReturnsOriginalValidGeometry)
{
  Parameters p;
  p.max_shift_px = 0.25;
  const auto original = shifted(truth(), {2, 0});
  cv::Mat gray = cv::Mat::zeros(500, 700, CV_8UC1);
  cv::rectangle(gray, {98, 100}, {102, 200}, 220, -1);
  cv::rectangle(gray, {198, 100}, {202, 200}, 220, -1);
  const auto result = refine(gray, original, p);
  ASSERT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "max_shift_exceeded");
  EXPECT_GT(result.max_shift_px, p.max_shift_px);
  expect_same(result.corners, original);
}

TEST(Refiner, RoundFeaturesDoNotHaveEnoughPcaInformation)
{
  cv::Mat gray = cv::Mat::zeros(500, 700, CV_8UC1);
  cv::circle(gray, {100, 150}, 6, 255, -1);
  cv::circle(gray, {200, 150}, 6, 255, -1);
  const auto result = refine(gray, truth());
  ASSERT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "left_insufficient_pca");
}

TEST(Refiner, ClippedBrightnessComponentSafelyFallsBack)
{
  auto gray = bars(truth());
  cv::rectangle(gray, {90, 100}, {110, 200}, 220, -1);
  const auto result = refine(gray, truth());
  EXPECT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "left_component_touches_roi");
  expect_same(result.corners, truth());
}

TEST(Refiner, InvalidOriginalPointsAreDroppedInsteadOfUnsafeFallback)
{
  const auto gray = bars(truth());
  std::vector<Corners> invalid;
  auto points = truth(); points[0].x = std::numeric_limits<float>::quiet_NaN();
  invalid.push_back(points);
  points = truth(); points[0].x = -1; invalid.push_back(points);
  points = truth(); points[0].x = 700; invalid.push_back(points);
  points = truth(); points[1] = points[0]; invalid.push_back(points);
  points = truth(); std::swap(points[1], points[3]); invalid.push_back(points);
  points = truth(); std::swap(points[1], points[2]); invalid.push_back(points);
  for (const auto & value : invalid) {
    auto result = refine(gray, value);
    EXPECT_EQ(result.outcome, Outcome::Dropped);
    EXPECT_EQ(result.reason, "invalid_yolo_geometry");
  }
}

TEST(Refiner, AxisAngleGuardRejectsUnexpectedBrightnessDirection)
{
  Corners tilted = truth();
  for (size_t i = 0; i < 4; ++i) {
    const cv::Point2f center(i < 2 ? 100 : 200, 150);
    const auto delta = tilted[i] - center;
    const double angle = 3 * CV_PI / 180;
    tilted[i] = center + cv::Point2f(
      std::cos(angle) * delta.x - std::sin(angle) * delta.y,
      std::sin(angle) * delta.x + std::cos(angle) * delta.y);
  }
  Parameters p; p.max_axis_angle_deg = 1;
  const auto result = refine(bars(tilted), truth(), p);
  EXPECT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "left_axis_mismatch");
}

TEST(Refiner, TwoInitialBarsCannotCollapseOntoOneBrightComponent)
{
  const Corners initial{{{100, 100}, {100, 120}, {102, 120}, {102, 100}}};
  cv::Mat gray = cv::Mat::zeros(500, 700, CV_8UC1);
  cv::line(gray, {101, 100}, {101, 120}, 220, 1);
  const auto result = refine(gray, initial);
  EXPECT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "invalid_refined_geometry");
  expect_same(result.corners, initial);
}

TEST(Refiner, BorderAndRoiLimitsSafelyFallback)
{
  auto near_border = shifted(truth(), {-98, 0});
  auto result = refine(bars(near_border), near_border);
  EXPECT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "left_roi_outside_image");
  Parameters p; p.max_roi_pixels = 10;
  result = refine(bars(truth()), truth(), p);
  EXPECT_EQ(result.outcome, Outcome::Fallback);
  EXPECT_EQ(result.reason, "left_roi_too_large");
}

TEST(Refiner, BatchSeparatelyRefinesFallsBackAndDrops)
{
  const auto gray = bars(truth());
  const auto good = shifted(truth(), {1, 0});
  const auto missing = truth(400);
  auto invalid = truth(); invalid[0].x = -5;
  auto results = refine_batch(gray, {good, missing, invalid});
  ASSERT_EQ(results.size(), 3U);
  EXPECT_EQ(results[0].outcome, Outcome::Refined);
  EXPECT_EQ(results[1].outcome, Outcome::Fallback);
  EXPECT_EQ(results[2].outcome, Outcome::Dropped);
  expect_same(results[1].corners, missing);
  EXPECT_TRUE(refine_batch(gray, {}).empty());
}

TEST(Refiner, InvalidImageAndParametersAreExplicit)
{
  EXPECT_EQ(refine(cv::Mat(), truth()).outcome, Outcome::Dropped);
  EXPECT_EQ(refine(cv::Mat::zeros(500, 700, CV_8UC3), truth()).outcome, Outcome::Dropped);
  Parameters p; p.min_pca_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(p.validate(), std::invalid_argument);
  p = Parameters{}; p.max_axis_angle_deg = 90;
  EXPECT_THROW(p.validate(), std::invalid_argument);
}
