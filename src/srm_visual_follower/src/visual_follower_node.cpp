#include "srm_visual_follower/follower.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
namespace srm_visual_follower {
class FollowerNode:public rclcpp::Node {
public:
  FollowerNode():Node("visual_follower_node"){
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    auto real=[&](const char * name,double & v){v=declare_parameter<double>(name,v,d);};
    p_.selection_strategy=declare_parameter<std::string>("selection_strategy",p_.selection_strategy,d);
    p_.sticky_selection=declare_parameter<bool>("sticky_selection",true,d);
    p_.max_tracks=declare_parameter<int>("max_tracks",p_.max_tracks,d);
    real("target_timeout_sec",p_.target_timeout_sec);real("max_state_age_sec",p_.max_state_age_sec);
    real("input_timeout_sec",p_.input_timeout_sec);real("future_tolerance_sec",p_.future_tolerance_sec);
    real("max_prediction_horizon_sec",p_.max_prediction_horizon_sec);
    real("min_depth_m",p_.min_depth_m);real("max_distance_m",p_.max_distance_m);
    p_.extrinsics_valid=declare_parameter<bool>("extrinsics_valid",false,d);
    p_.reference_frame=declare_parameter<std::string>("reference_frame","",d);
    const auto t=declare_parameter<std::vector<double>>("translation_m",{0.,0.,0.},d);
    const auto q=declare_parameter<std::vector<double>>("quaternion_xyzw",{0.,0.,0.,1.},d);
    if(t.size()!=3||q.size()!=4)throw std::invalid_argument("Extrinsics require translation[3], quaternion[4]");
    std::copy(t.begin(),t.end(),p_.translation_m.begin());std::copy(q.begin(),q.end(),p_.quaternion_xyzw.begin());
    p_.validate();core_=std::make_unique<Follower>(p_);
    const auto topic=get_node_topics_interface()->resolve_topic_name("/srm_aim/visual_follower/debug");
    if(topic.rfind("/srm_aim/cmd_",0)==0||topic=="/srm_aim/yaw_error"||topic=="/srm_aim/pitch_error")
      throw std::invalid_argument("Diagnostic output cannot be remapped to a control topic");
    output_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/srm_aim/visual_follower/debug",5);
    input_=create_subscription<Input>("/srm_aim/armor_tracks",rclcpp::QoS(5).reliable(),
      [this](Input::ConstSharedPtr in){auto result=core_->process(*in,get_clock()->now().nanoseconds());
        if(result.input_accepted){receipt_=Clock::now();timeout_sent_=false;}publish(result);});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{
      if(receipt_&&!timeout_sent_&&std::chrono::duration<double>(Clock::now()-*receipt_).count()>p_.input_timeout_sec){
        timeout_sent_=true;publish(core_->invalidate("input_timeout"));
      }else if(auto r=core_->poll(get_clock()->now().nanoseconds()))publish(*r);
    });
    RCLCPP_WARN(get_logger(),"Diagnostics only: camera LOS / optional reference LOS; hardware_command_valid=false; no control publishers");
  }
private:
  using Clock=std::chrono::steady_clock;
  void publish(const Result & r){
    diagnostic_msgs::msg::DiagnosticArray out;out.header=r.header;
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="visual_follower/line_of_sight";
    s.level=r.target_valid?diagnostic_msgs::msg::DiagnosticStatus::OK:diagnostic_msgs::msg::DiagnosticStatus::WARN;
    s.message=r.reason;
    auto text=[&](const std::string & key,const std::string & value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;s.values.push_back(kv);};
    auto number=[&](const std::string & key,double v){std::ostringstream value;value<<std::setprecision(12)<<v;text(key,value.str());};
    auto flag=[&](const std::string & key,bool v){text(key,v?"true":"false");};
    auto stamp=[&](const std::string & key,const builtin_interfaces::msg::Time & t){
      text(key+"_sec",std::to_string(t.sec));text(key+"_nanosec",std::to_string(t.nanosec));};
    auto point=[&](const std::string & key,const Point & p){number(key+"_x_m",p.x);number(key+"_y_m",p.y);number(key+"_z_m",p.z);};
    flag("target_valid",r.target_valid);flag("camera_los_valid",r.target_valid);
    flag("reference_los_valid",r.reference_valid);flag("hardware_command_valid",false);
    flag("extrinsics_valid",p_.extrinsics_valid);text("mode","diagnostics_only");
    text("session_id",r.session_id);text("track_id",std::to_string(r.track_id));
    text("position_source",r.position_source);text("camera_frame","camera_optical_frame");
    text("reference_frame",p_.reference_frame);text("reference_reason",r.reference_reason);
    stamp("source_stamp",r.header.stamp);stamp("state_stamp",r.state_stamp);
    stamp("observation_stamp",r.observation_stamp);stamp("position_stamp",r.position_stamp);
    point("camera_position",r.position);point("reference_position",r.reference_position);
    const double deg=180/std::acos(-1.0);
    number("camera_yaw_rad",r.camera_yaw_rad);number("camera_pitch_rad",r.camera_pitch_rad);
    number("camera_yaw_deg",r.camera_yaw_rad*deg);number("camera_pitch_deg",r.camera_pitch_rad*deg);
    number("reference_yaw_rad",r.reference_yaw_rad);number("reference_pitch_rad",r.reference_pitch_rad);
    number("reference_yaw_deg",r.reference_yaw_rad*deg);number("reference_pitch_deg",r.reference_pitch_rad*deg);
    text("camera_angle_convention","yaw positive right; pitch positive up");
    text("reference_angle_convention","FLU: yaw positive left; pitch positive up; not motor signs");
    out.status.push_back(std::move(s));output_->publish(out);
  }
  Parameters p_;std::unique_ptr<Follower> core_;
  std::optional<Clock::time_point> receipt_;bool timeout_sent_{false};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr output_;
  rclcpp::Subscription<Input>::SharedPtr input_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);int code=0;
  try{rclcpp::spin(std::make_shared<srm_visual_follower::FollowerNode>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("visual_follower_node"),"Follower failed: %s",e.what());code=1;}
  if(rclcpp::ok())rclcpp::shutdown();
  return code;
}
