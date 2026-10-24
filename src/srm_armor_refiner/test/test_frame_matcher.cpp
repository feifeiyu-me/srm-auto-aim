#include <gtest/gtest.h>
#include "srm_armor_refiner/frame_matcher.hpp"

using namespace srm_armor_refiner;
namespace
{
FrameMatcher::Time at(int milliseconds)
{return FrameMatcher::Time{} + std::chrono::milliseconds(milliseconds);}
auto image(uint32_t index = 1)
{
  auto image = std::make_shared<FrameMatcher::Image>();
  image->header.stamp.sec = 123; image->header.stamp.nanosec = index;
  image->header.frame_id = "camera";
  image->width = 8; image->height = 8; image->data.resize(192);
  return image;
}
auto detection(const FrameMatcher::Image & image)
{
  auto detection = std::make_shared<FrameMatcher::Detections>();
  detection->header = image.header;
  detection->image_width = image.width; detection->image_height = image.height;
  return detection;
}
}  // namespace

TEST(FrameMatcher, EitherArrivalOrderMatchesExactHistoricalHeader)
{
  for (bool detection_first : {true, false}) {
    FrameMatcher cache(2, 2, 1000, 1);
    auto source = image(); auto result = detection(*source);
    std::optional<FrameMatcher::Pair> pair;
    if (detection_first) {
      EXPECT_FALSE(cache.add_detection(result, at(0)));
      pair = cache.add_image(source, at(100));
    } else {
      EXPECT_FALSE(cache.add_image(source, at(0)));
      pair = cache.add_detection(result, at(100));
    }
    ASSERT_TRUE(pair);
    EXPECT_EQ(pair->image, source);
    EXPECT_EQ(pair->detections, result);
    EXPECT_EQ(cache.image_bytes(), 0U);
  }
}

TEST(FrameMatcher, StampFrameIdAndDimensionsMustAllMatch)
{
  for (int mismatch : {0, 1, 2}) {
    FrameMatcher cache(2, 2, 1000, 1);
    auto source = image(); auto result = detection(*source);
    if (mismatch == 0) {result->header.stamp.nanosec++;}
    if (mismatch == 1) {result->header.frame_id = "other";}
    if (mismatch == 2) {result->image_width++;}
    cache.add_image(source, at(0));
    EXPECT_FALSE(cache.add_detection(result, at(10)));
  }
}

TEST(FrameMatcher, NeverUsesLatestImageInsteadOfExactOldImage)
{
  FrameMatcher cache(3, 3, 1000, 1);
  auto old = image(1), latest = image(2);
  cache.add_image(old, at(0)); cache.add_image(latest, at(50));
  auto pair = cache.add_detection(detection(*old), at(100));
  ASSERT_TRUE(pair); EXPECT_EQ(pair->image, old);
}

TEST(FrameMatcher, IdleExpiryAndCountByteLimitsStayBounded)
{
  FrameMatcher cache(2, 2, 384, 1);
  for (int i = 0; i < 1000; ++i) {
    cache.add_image(image(i), at(0));
    cache.add_detection(detection(*image(i + 10000)), at(0));
    EXPECT_LE(cache.image_count(), 2U); EXPECT_LE(cache.detection_count(), 2U);
    EXPECT_LE(cache.image_bytes(), 384U);
  }
  cache.prune(at(1001));
  EXPECT_EQ(cache.image_count(), 0U); EXPECT_EQ(cache.detection_count(), 0U);
  EXPECT_EQ(cache.image_bytes(), 0U);
  EXPECT_LE(cache.take_events().size(), 100U);
}

TEST(FrameMatcher, ExpiredAndOversizedImagesDoNotMatch)
{
  FrameMatcher cache(2, 2, 300, 0.1);
  auto source = image(); cache.add_image(source, at(0));
  EXPECT_FALSE(cache.add_detection(detection(*source), at(101)));
  auto big = image(2); big->data.resize(301);
  EXPECT_FALSE(cache.add_image(big, at(102)));
}

TEST(FrameMatcher, DuplicateImagesAreAmbiguousAndMatchedKeysCannotReplay)
{
  FrameMatcher cache(2, 2, 1000, 1);
  auto source = image();
  cache.add_image(source, at(0));
  EXPECT_FALSE(cache.add_image(source, at(1)));
  EXPECT_FALSE(cache.add_detection(detection(*source), at(2)));
  source = image(2); cache.add_image(source, at(3));
  EXPECT_TRUE(cache.add_detection(detection(*source), at(4)));
  EXPECT_FALSE(cache.add_detection(detection(*source), at(5)));
}

TEST(FrameMatcher, InvalidStampAndConfigurationAreRejected)
{
  EXPECT_THROW(FrameMatcher(0, 1, 100, 1), std::invalid_argument);
  FrameMatcher cache(2, 2, 1000, 1);
  auto source = image(); source->header.stamp.nanosec = 1000000000;
  EXPECT_FALSE(cache.add_image(source, at(0)));
  EXPECT_FALSE(cache.add_detection(detection(*source), at(1)));
}
