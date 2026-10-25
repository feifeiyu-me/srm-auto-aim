#include "srm_gimbal_follower/controller.hpp"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
namespace srm_gimbal_follower {
using Diagnostic=diagnostic_msgs::msg::DiagnosticArray;
class FollowerNode:public rclcpp::Node {
public:
  FollowerNode():Node("gimbal_follower_node"){
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    p_.mode=declare_parameter<std::string>("mode",p_.mode,d);
    p_.calibration_verified=declare_parameter<bool>("calibration_verified",false,d);
    p_.allow_unverified_simulation_calibration=declare_parameter<bool>("allow_unverified_simulation_calibration",false,d);
    p_.allow_target_switch=declare_parameter<bool>("allow_target_switch",true,d);
    auto real=[&](const char * name,double & value){value=declare_parameter<double>(name,value,d);};
#define REAL(name) real(#name,p_.name)
    REAL(yaw_feedback_sign);REAL(pitch_feedback_sign);REAL(roll_feedback_sign);
    REAL(yaw_zero_deg);REAL(pitch_zero_deg);REAL(roll_zero_deg);REAL(kp);REAL(deadband_deg);
    REAL(yaw_min_deg);REAL(yaw_max_deg);REAL(pitch_min_deg);REAL(pitch_max_deg);
    REAL(max_step_deg);REAL(max_rate_deg_sec);REAL(max_visual_correction_deg);
    REAL(visual_timeout_sec);REAL(feedback_timeout_sec);REAL(future_tolerance_sec);
    REAL(alignment_gap_sec);REAL(feedback_gap_sec);REAL(history_sec);
    REAL(feedback_max_jump_deg);REAL(feedback_max_rate_deg_sec);
    REAL(target_switch_delay_sec);REAL(target_switch_confirm_sec);
    REAL(max_prediction_sec);REAL(prediction_grace_sec);REAL(max_range_m);REAL(min_depth_m);
    REAL(control_period_sec);REAL(max_tick_gap_sec);
#undef REAL
    p_.history_capacity=declare_parameter<int>("history_capacity",p_.history_capacity,d);
    auto q=declare_parameter<std::vector<double>>("camera_to_gimbal_quaternion_xyzw",{-.5,.5,-.5,.5},d);
    auto t=declare_parameter<std::vector<double>>("camera_to_gimbal_translation_m",{0.,0.,0.},d);
    if(q.size()!=4||t.size()!=3)throw std::invalid_argument("Calibration vector sizes must be 4 and 3");
    std::copy(q.begin(),q.end(),p_.quaternion_xyzw.begin());std::copy(t.begin(),t.end(),p_.translation_m.begin());
    p_.validate();core_=std::make_unique<Controller>(p_);
    for(const auto & name:{"/srm_aim/gimbal_follower/debug","/srm_aim/gimbal_follower/sim_reference"})
      if(get_node_topics_interface()->resolve_topic_name(name).rfind("/srm_aim/gimbal_follower/",0)!=0)
        throw std::invalid_argument("Simulation outputs must stay in /srm_aim/gimbal_follower/");
    debug_=create_publisher<Diagnostic>("/srm_aim/gimbal_follower/debug",5);
    reference_=create_publisher<Diagnostic>("/srm_aim/gimbal_follower/sim_reference",5);
    feedback_=create_subscription<geometry_msgs::msg::Vector3Stamped>("/srm_aim/gimbal_state_stamped",10,
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m){core_->feedback(*m,time());});
    visual_=create_subscription<Diagnostic>("/srm_aim/visual_follower/debug",5,
      [this](Diagnostic::ConstSharedPtr m){core_->visual(*m,time());});
    timer_=create_wall_timer(std::chrono::nanoseconds(int64_t(p_.control_period_sec*1e9)),[this]{publish(core_->tick(time()));});
    RCLCPP_WARN(get_logger(),"SIMULATION ONLY. No hardware command publishers or serial I/O. Calibration verified=%s",p_.calibration_verified?"true":"false");
  }
private:
  Time time(){return {get_clock()->now().nanoseconds(),std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};}
  void publish(const Output & out){
    Diagnostic msg;msg.header.stamp=get_clock()->now();msg.header.frame_id="simulation_feedback_reference_degrees";
    diagnostic_msgs::msg::DiagnosticStatus status;status.name="srm_gimbal_follower/reference";
    status.level=out.simulation_reference_valid?0:1;status.message=out.reason;
    auto text=[&](const std::string & key,const std::string & value){diagnostic_msgs::msg::KeyValue v;v.key=key;v.value=value;status.values.push_back(v);};
    auto number=[&](const std::string & key,double value){std::ostringstream s;s<<std::setprecision(15)<<value;text(key,s.str());};
    auto stamp=[&](const std::string & key,int64_t value){text(key+"_sec",std::to_string(value/1000000000));text(key+"_nanosec",std::to_string(value%1000000000));};
    text("sequence",std::to_string(++sequence_));
    text("mode","simulation");text("real_hardware_enabled","false");text("serial_tx_enabled","false");
    text("simulation_reference_valid",out.simulation_reference_valid?"true":"false");
    text("hold_requested",out.simulation_reference_valid?"false":"true");
    text("calibration_verified",p_.calibration_verified?"true":"false");
    text("calibration_source",p_.calibration_verified?"user_asserted_verified":"unverified_synthetic_simulation_only");
    text("locked_target",core_->locked_target());text("pending_target",core_->pending_target());
    text("reason",out.reason);text("last_input_rejection",core_->last_rejection());
    text("session_id",out.session);text("track_id",out.id);text("position_source",out.source);
    number("yaw_reference_deg",out.yaw_reference_deg);number("pitch_reference_deg",out.pitch_reference_deg);
    number("yaw_target_deg",out.yaw_target_deg);number("pitch_target_deg",out.pitch_target_deg);
    number("yaw_error_deg",out.yaw_error_deg);number("pitch_error_deg",out.pitch_error_deg);number("dt_sec",out.dt_sec);
    number("max_step_deg",p_.max_step_deg);number("max_rate_deg_sec",p_.max_rate_deg_sec);
    text("limited",out.limited?"true":"false");
    stamp("source_stamp",out.source_ns);stamp("state_stamp",out.state_ns);stamp("position_stamp",out.position_ns);stamp("feedback_stamp",out.feedback_ns);
    msg.status.push_back(std::move(status));debug_->publish(msg);reference_->publish(msg);
  }
  Parameters p_;std::unique_ptr<Controller> core_;uint64_t sequence_{0};
  rclcpp::Publisher<Diagnostic>::SharedPtr debug_,reference_;
  rclcpp::Subscription<Diagnostic>::SharedPtr visual_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr feedback_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);int code=0;
  try{rclcpp::spin(std::make_shared<srm_gimbal_follower::FollowerNode>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("gimbal_follower_node"),"Simulation follower failed: %s",e.what());code=1;}
  if(rclcpp::ok()){rclcpp::shutdown();}
  return code;
}
