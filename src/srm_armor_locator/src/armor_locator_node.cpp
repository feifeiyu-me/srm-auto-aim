#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srm_aim_interfaces/msg/armor_location_array.hpp>
#include <srm_armor_refiner/frame_matcher.hpp>
#include "srm_armor_locator/locator.hpp"
namespace srm_armor_locator {
using Input=srm_aim_interfaces::msg::ArmorDetectionArray;
using Output=srm_aim_interfaces::msg::ArmorLocationArray;
using Matcher=srm_armor_refiner::FrameMatcher;
using Status=diagnostic_msgs::msg::DiagnosticStatus;
class Locator : public rclcpp::Node {
public:
  Locator():Node("armor_locator_node") {
    rcl_interfaces::msg::ParameterDescriptor desc; desc.read_only=true;
    calibration_=Calibration::load(declare_parameter<std::string>("calibration_file","",desc));
    optical_=declare_parameter<std::string>("optical_frame_id","camera_optical_frame",desc);
    if (optical_.empty()) throw std::invalid_argument("optical_frame_id must be explicit/nonempty");
    auto real=[&](const std::string & name,double value){return declare_parameter<double>(name,value,desc);};
    p_.width_m=real("width_m",.135); p_.height_m=real("height_m",.058);
    p_.min_depth_m=real("min_depth_m",.1); p_.max_depth_m=real("max_depth_m",20);
    p_.max_error_px=real("max_error_px",3); p_.ambiguity_gap_px=real("ambiguity_gap_px",.1);
    p_.ambiguity_translation_m=real("ambiguity_translation_m",.01);
    p_.ambiguity_angle_deg=real("ambiguity_angle_deg",5); p_.validate();
    digit_enabled_=declare_parameter<bool>("enable_digit_verification",true,desc);
    threshold_=real("digit_threshold",.65);
    if (!std::isfinite(threshold_) || threshold_<=0 || threshold_>1) throw std::invalid_argument("Digit threshold must be in (0,1]; use the explicit bypass switch instead of zero");
    const auto templ=declare_parameter<std::string>("template_file",
      ament_index_cpp::get_package_share_directory("srm_armor_locator")+"/templates/digit_2_template.png",desc);
    if (digit_enabled_) {
      template_=cv::imread(templ,cv::IMREAD_GRAYSCALE);
      cv::Scalar m,s; if (!template_.empty()) cv::meanStdDev(template_,m,s);
      if (template_.empty() || s[0]<2) throw std::invalid_argument("Missing/constant digit template: "+templ);
    }
    const int images=declare_parameter<int>("image_cache_size",20,desc);
    const int detections=declare_parameter<int>("detection_cache_size",20,desc);
    const int64_t bytes=declare_parameter<int64_t>("max_cache_bytes",96*1024*1024,desc);
    if (images<=0 || detections<=0 || bytes<=0) throw std::invalid_argument("Invalid cache limits");
    matcher_=std::make_unique<Matcher>(images,detections,bytes,real("cache_ttl_sec",1));
    auto input=declare_parameter<std::string>("detections_topic","/srm_aim/armor_detections_refined",desc);
    auto output=declare_parameter<std::string>("locations_topic","/srm_aim/armor_locations",desc);
    auto topics=get_node_topics_interface();
    if (topics->resolve_topic_name(input)==topics->resolve_topic_name(output)) throw std::invalid_argument("Output must not overwrite input");
    output_=create_publisher<Output>(output,1);
    debug_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/srm_aim/armor_locator/debug",10);
    images_=create_subscription<sensor_msgs::msg::Image>("/srm_aim/image_raw",rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){auto pair=matcher_->add_image(m,Matcher::Clock::now());flush();if(pair) process(*pair);});
    detections_=create_subscription<Input>(input,rclcpp::QoS(10).best_effort(),
      [this](Input::ConstSharedPtr m){auto pair=matcher_->add_detection(m,Matcher::Clock::now());flush();if(pair) process(*pair);});
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]{matcher_->prune(Matcher::Clock::now());flush();});
    RCLCPP_INFO(get_logger(),"Locator ready; digit verification %s; optical poses only; no control I/O",digit_enabled_?"ENABLED":"DISABLED (digit unverified)");
  }
private:
  Calibration calibration_; Parameters p_; cv::Mat template_; bool digit_enabled_; double threshold_;
  std::string optical_; std::unique_ptr<Matcher> matcher_;
  rclcpp::Publisher<Output>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr debug_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr images_;
  rclcpp::Subscription<Input>::SharedPtr detections_; rclcpp::TimerBase::SharedPtr timer_;
  void diagnostic(const std_msgs::msg::Header & header,const std::string & reason) {
    diagnostic_msgs::msg::DiagnosticArray d;d.header=header;
    Status s;s.name="armor_locator/frame";s.level=Status::WARN;s.message=reason;d.status.push_back(s);debug_->publish(d);
  }
  void flush(){for(const auto & e:matcher_->take_events()) diagnostic(e.header,e.reason);}
  void process(const Matcher::Pair & pair) {
    if (pair.detections->image_width!=static_cast<uint32_t>(calibration_.size.width) ||
      pair.detections->image_height!=static_cast<uint32_t>(calibration_.size.height)) {
      diagnostic(pair.detections->header,"calibration_resolution_mismatch");return;
    }
    cv::Mat gray;
    try {
      const auto & e=pair.image->encoding;
      if(e!="bgr8" && e!="rgb8" && e!="mono8" && e!="bgra8" && e!="rgba8") {
        diagnostic(pair.detections->header,"unsupported_encoding");return;
      }
      gray=cv_bridge::toCvCopy(pair.image,"mono8")->image;
      if(gray.size()!=calibration_.size) {diagnostic(pair.detections->header,"decoded_size_mismatch");return;}
    } catch(const std::exception & e){diagnostic(pair.detections->header,std::string("decode_error: ")+e.what());return;}
    Output output;output.header=pair.detections->header;output.optical_frame_id=optical_;
    output.image_width=pair.detections->image_width;output.image_height=pair.detections->image_height;
    diagnostic_msgs::msg::DiagnosticArray debug;debug.header=output.header;
    for(size_t i=0;i<pair.detections->detections.size();++i) {
      const auto & armor=pair.detections->detections[i];
      srm_aim_interfaces::msg::ArmorLocation location;location.input_index=i;location.detection=armor;
      location.digit_check_enabled=digit_enabled_;location.digit_score=-1;location.reprojection_error_px=-1;
      location.pose.orientation.w=1;
      Corners corners{{{armor.lt.x,armor.lt.y},{armor.lb.x,armor.lb.y},{armor.rb.x,armor.rb.y},{armor.rt.x,armor.rt.y}}};
      if (!std::isfinite(armor.confidence) || armor.confidence<0 || armor.confidence>1 || armor.color>2)
        location.reason="invalid_metadata";
      else if (!srm_armor_refiner::valid_corners(corners,calibration_.size,{})) location.reason="invalid_geometry";
      else {
        if(digit_enabled_) {
          try {location.digit_score=digit_score(gray,corners,template_);}
          catch(const cv::Exception &){location.digit_score=-1;}
          location.digit_verified=location.digit_score>=threshold_;
          if(location.digit_verified) location.digit=2;
        }
        if(digit_enabled_ && !location.digit_verified) location.reason="digit_not_verified";
        else {
          const auto result=solve(calibration_,corners,p_);
          location.pose_valid=result.valid;location.ambiguous=result.ambiguous;
          location.reason=result.reason;location.reprojection_error_px=result.error;
          if(result.valid) {
            location.axial_depth_m=result.translation[2];location.distance_m=cv::norm(result.translation);
            location.pose.position.x=result.translation[0];location.pose.position.y=result.translation[1];location.pose.position.z=result.translation[2];
            const double angle=cv::norm(result.rotation);
            if(angle>1e-12) {
              const auto v=result.rotation/angle*std::sin(angle/2);
              location.pose.orientation.x=v[0];location.pose.orientation.y=v[1];location.pose.orientation.z=v[2];location.pose.orientation.w=std::cos(angle/2);
            }
            if(!digit_enabled_) location.reason="ok_digit_unverified";
          }
        }
      }
      Status s;s.name="armor_locator/target_"+std::to_string(i);
      s.level=location.pose_valid?Status::OK:Status::WARN;s.message=location.reason;
      for(const auto & item:std::vector<std::pair<std::string,std::string>>{
          {"input_index",std::to_string(i)},{"digit_check_enabled",digit_enabled_?"true":"false"},
          {"digit_verified",location.digit_verified?"true":"false"},{"digit_score",std::to_string(location.digit_score)},
          {"pose_valid",location.pose_valid?"true":"false"},{"ambiguous",location.ambiguous?"true":"false"}}) {
        diagnostic_msgs::msg::KeyValue v;v.key=item.first;v.value=item.second;s.values.push_back(v);
      }
      debug.status.push_back(s);output.locations.push_back(location);
    }
    if(output.locations.empty()) {Status s;s.name="armor_locator/frame";s.level=Status::OK;s.message="matched_empty";debug.status.push_back(s);}
    output_->publish(output);debug_->publish(debug);
  }
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);int status=0;
  try{rclcpp::spin(std::make_shared<srm_armor_locator::Locator>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("armor_locator_node"),"Locator refused/failed: %s",e.what());status=1;}
  if(rclcpp::ok()) {rclcpp::shutdown();}
  return status;
}
