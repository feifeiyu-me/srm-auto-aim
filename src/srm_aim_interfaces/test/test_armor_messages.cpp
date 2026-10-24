#include <gtest/gtest.h>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <srm_aim_interfaces/msg/armor_detection.hpp>
#include <srm_aim_interfaces/msg/armor_detection_array.hpp>

TEST(ArmorMessages, PreservesMultipleArmorsAndSourceImageHeader)
{
  using srm_aim_interfaces::msg::ArmorDetection;
  using srm_aim_interfaces::msg::ArmorDetectionArray;
  ArmorDetectionArray source;
  source.header.stamp.sec = 123;
  source.header.stamp.nanosec = 456789012;
  source.header.frame_id = "camera_optical_frame";
  source.image_width = 1440;
  source.image_height = 1080;

  ArmorDetection red;
  red.color = ArmorDetection::COLOR_RED;
  red.confidence = 0.875F;
  red.lt.x = 100.25F;
  red.lt.y = 200.5F;
  red.lb.x = 101.0F;
  red.lb.y = 300.0F;
  red.rb.x = 400.0F;
  red.rb.y = 301.0F;
  red.rt.x = 399.0F;
  red.rt.y = 201.0F;
  auto blue = red;
  blue.color = ArmorDetection::COLOR_BLUE;
  blue.confidence = 0.75F;
  blue.lt.x = 700.0F;
  source.detections = {red, blue};

  rclcpp::Serialization<ArmorDetectionArray> serializer;
  rclcpp::SerializedMessage wire;
  serializer.serialize_message(&source, &wire);
  ArmorDetectionArray decoded;
  serializer.deserialize_message(&wire, &decoded);

  EXPECT_EQ(decoded, source);
  ASSERT_EQ(decoded.detections.size(), 2U);
  EXPECT_EQ(decoded.detections[0].color, ArmorDetection::COLOR_RED);
  EXPECT_EQ(decoded.detections[1].color, ArmorDetection::COLOR_BLUE);
}

TEST(ArmorMessages, EmptyArrayRetainsFrameIdentity)
{
  using srm_aim_interfaces::msg::ArmorDetectionArray;
  ArmorDetectionArray source;
  source.header.stamp.sec = 124;
  source.header.frame_id = "camera";
  source.image_width = 1440;
  source.image_height = 1080;

  rclcpp::Serialization<ArmorDetectionArray> serializer;
  rclcpp::SerializedMessage wire;
  serializer.serialize_message(&source, &wire);
  ArmorDetectionArray decoded;
  serializer.deserialize_message(&wire, &decoded);

  EXPECT_EQ(decoded, source);
  EXPECT_TRUE(decoded.detections.empty());
}
