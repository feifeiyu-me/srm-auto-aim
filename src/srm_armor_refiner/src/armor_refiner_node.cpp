#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include "srm_armor_refiner/frame_matcher.hpp"
#include "srm_armor_refiner/refiner.hpp"

namespace srm_armor_refiner
{
using Array = srm_aim_interfaces::msg::ArmorDetectionArray;
using Armor = srm_aim_interfaces::msg::ArmorDetection;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

class ArmorRefinerNode : public rclcpp::Node
{
public:
  ArmorRefinerNode() : Node("armor_refiner_node")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    auto real = [&](const std::string & name, double value) {
        return declare_parameter<double>(name, value, descriptor);
      };
    p_.roi_half_width_px = real("roi_half_width_px", p_.roi_half_width_px);
    p_.min_contrast = real("min_contrast", p_.min_contrast);
    p_.min_pixels = declare_parameter<int>("min_pixels", p_.min_pixels, descriptor);
    p_.min_light_length_px = real("min_light_length_px", p_.min_light_length_px);
    p_.min_pca_ratio = real("min_pca_ratio", p_.min_pca_ratio);
    p_.max_axis_angle_deg = real("max_axis_angle_deg", p_.max_axis_angle_deg);
    p_.max_shift_px = real("max_shift_px", p_.max_shift_px);
    p_.max_shift_fraction = real("max_shift_fraction", p_.max_shift_fraction);
    p_.min_quad_area = real("min_quad_area", p_.min_quad_area);
    p_.min_edge_length = real("min_edge_length", p_.min_edge_length);
    p_.max_edge_ratio = real("max_edge_ratio", p_.max_edge_ratio);
    p_.max_roi_pixels = declare_parameter<int>("max_roi_pixels", p_.max_roi_pixels, descriptor);
    p_.validate();
    const int images = declare_parameter<int>("image_cache_size", 20, descriptor);
    const int detections = declare_parameter<int>("detection_cache_size", 20, descriptor);
    const int64_t bytes = declare_parameter<int64_t>("max_cache_bytes", 96 * 1024 * 1024, descriptor);
    const double ttl = real("cache_ttl_sec", 1.0);
    if (images <= 0 || detections <= 0 || bytes <= 0) {
      throw std::invalid_argument("Cache counts and byte limit must be positive");
    }
    matcher_ = std::make_unique<FrameMatcher>(images, detections, bytes, ttl);
    const auto input = declare_parameter<std::string>(
      "detections_topic", "/srm_aim/armor_detections", descriptor);
    const auto output = declare_parameter<std::string>(
      "refined_topic", "/srm_aim/armor_detections_refined", descriptor);
    if (get_node_topics_interface()->resolve_topic_name(input) ==
      get_node_topics_interface()->resolve_topic_name(output))
    {
      throw std::invalid_argument("Refined output must not overwrite the input topic");
    }
    output_ = create_publisher<Array>(output, rclcpp::QoS(1));
    debug_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/srm_aim/armor_refiner/debug", rclcpp::QoS(10));
    images_ = create_subscription<sensor_msgs::msg::Image>(
      "/srm_aim/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
        auto pair = matcher_->add_image(image, FrameMatcher::Clock::now());
        flush_events(); if (pair) {process(*pair);}
      });
    detections_ = create_subscription<Array>(input, rclcpp::QoS(10).best_effort(),
      [this](Array::ConstSharedPtr detection) {
        auto pair = matcher_->add_detection(detection, FrameMatcher::Clock::now());
        flush_events(); if (pair) {process(*pair);}
      });
    cleanup_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      matcher_->prune(FrameMatcher::Clock::now()); flush_events();
    });
    RCLCPP_INFO(get_logger(), "PCA candidate refiner ready; exact-frame matching; no control I/O");
  }

private:
  Parameters p_;
  std::unique_ptr<FrameMatcher> matcher_;
  rclcpp::Publisher<Array>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr debug_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Subscription<Array>::SharedPtr detections_;
  rclcpp::TimerBase::SharedPtr cleanup_;

  static void value(Status & status, const std::string & key, const std::string & text)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key; item.value = text; status.values.push_back(item);
  }
  static std::string coordinates(const Corners & corners)
  {
    std::ostringstream out;
    for (const auto & p : corners) {out << p.x << ',' << p.y << ';';}
    return out.str();
  }
  void frame_event(const std_msgs::msg::Header & header, const std::string & reason)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header = header;
    Status status;
    status.name = "armor_refiner/frame"; status.level = Status::WARN;
    status.message = reason; status.hardware_id = "cpu_pca";
    message.status.push_back(status); debug_->publish(message);
    if (reason != "unmatched_image_expired" && reason != "image_cache_evicted") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Frame dropped: %s", reason.c_str());
    }
  }
  void flush_events()
  {
    for (const auto & event : matcher_->take_events()) {frame_event(event.header, event.reason);}
  }
  void process(const FrameMatcher::Pair & pair)
  {
    const auto started = FrameMatcher::Clock::now();
    cv::Mat gray;
    try {
      const auto & encoding = pair.image->encoding;
      if (encoding != "bgr8" && encoding != "rgb8" && encoding != "mono8" &&
        encoding != "bgra8" && encoding != "rgba8")
      {
        frame_event(pair.detections->header, "unsupported_encoding"); return;
      }
      gray = cv_bridge::toCvCopy(pair.image, "mono8")->image;
      if (gray.cols != static_cast<int>(pair.image->width) ||
        gray.rows != static_cast<int>(pair.image->height))
      {
        frame_event(pair.detections->header, "decoded_size_mismatch"); return;
      }
    } catch (const std::exception & e) {
      frame_event(pair.detections->header, std::string("decode_error: ") + e.what()); return;
    }
    Array output;
    output.header = pair.detections->header;
    output.image_width = pair.detections->image_width;
    output.image_height = pair.detections->image_height;
    diagnostic_msgs::msg::DiagnosticArray debug;
    debug.header = output.header;
    size_t refined = 0, fallback = 0, dropped = 0;
    for (size_t index = 0; index < pair.detections->detections.size(); ++index) {
      const auto & original = pair.detections->detections[index];
      const Corners initial{{{original.lt.x, original.lt.y}, {original.lb.x, original.lb.y},
          {original.rb.x, original.rb.y}, {original.rt.x, original.rt.y}}};
      Result result;
      if (!std::isfinite(original.confidence) || original.confidence < 0 ||
        original.confidence > 1 || original.color > Armor::COLOR_BLUE)
      {
        result = {Outcome::Dropped, {}, "invalid_metadata", 0};
      } else {result = refine(gray, initial, p_);}
      Status status;
      status.name = "armor_refiner/target_" + std::to_string(index);
      status.hardware_id = "cpu_pca"; status.message = result.reason;
      status.level = result.outcome == Outcome::Refined ? Status::OK : Status::WARN;
      value(status, "input_index", std::to_string(index));
      value(status, "original_lt_lb_rb_rt", coordinates(initial));
      value(status, "max_shift_px", std::to_string(result.max_shift_px));
      if (result.outcome == Outcome::Dropped) {
        ++dropped; value(status, "outcome", "dropped"); value(status, "output_index", "-1");
      } else {
        Armor armor = original;
        armor.lt.x = result.corners[0].x; armor.lt.y = result.corners[0].y;
        armor.lb.x = result.corners[1].x; armor.lb.y = result.corners[1].y;
        armor.rb.x = result.corners[2].x; armor.rb.y = result.corners[2].y;
        armor.rt.x = result.corners[3].x; armor.rt.y = result.corners[3].y;
        value(status, "output_index", std::to_string(output.detections.size()));
        value(status, "output_lt_lb_rb_rt", coordinates(result.corners));
        output.detections.push_back(armor);
        if (result.outcome == Outcome::Refined) {++refined; value(status, "outcome", "refined");}
        else {++fallback; value(status, "outcome", "fallback");}
      }
      debug.status.push_back(status);
    }
    Status summary;
    summary.name = "armor_refiner/frame";
    summary.level = fallback || dropped ? Status::WARN : Status::OK;
    summary.message = "matched"; summary.hardware_id = "cpu_pca";
    value(summary, "refined", std::to_string(refined));
    value(summary, "fallback", std::to_string(fallback));
    value(summary, "dropped", std::to_string(dropped));
    value(summary, "processing_ms", std::to_string(
      std::chrono::duration<double, std::milli>(FrameMatcher::Clock::now() - started).count()));
    debug.status.push_back(summary);
    output_->publish(output); debug_->publish(debug);
  }
};
}  // namespace srm_armor_refiner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {rclcpp::spin(std::make_shared<srm_armor_refiner::ArmorRefinerNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("armor_refiner_node"), "Refiner failed: %s", e.what());
    status = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return status;
}
