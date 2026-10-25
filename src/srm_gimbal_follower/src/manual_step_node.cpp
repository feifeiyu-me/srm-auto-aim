#include "srm_gimbal_follower/manual_step.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
namespace srm_gimbal_follower {
class ManualNode:public rclcpp::Node {
public:
  ManualNode():Node("gimbal_manual_step_node"){
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;StepParameters p;
    p.allow_hardware_commands=declare_parameter("allow_hardware_commands",false,d);
    p.absolute_target_protocol_verified=declare_parameter("absolute_target_protocol_verified",false,d);
    p.mechanical_limits_verified=declare_parameter("mechanical_limits_verified",false,d);
    p.yaw_min=declare_parameter("yaw_min_deg",0.,d);p.yaw_max=declare_parameter("yaw_max_deg",0.,d);
    p.pitch_min=declare_parameter("pitch_min_deg",0.,d);p.pitch_max=declare_parameter("pitch_max_deg",0.,d);
    p.rate=declare_parameter("max_rate_deg_sec",2.,d);core_=std::make_unique<ManualStep>(p);
    for(const auto & topic:{"/srm_aim/gimbal_step/cmd_yaw","/srm_aim/gimbal_step/cmd_pitch"})
      if(get_node_topics_interface()->resolve_topic_name(topic)!=topic)throw std::invalid_argument("Private command outputs must not be remapped");
    yaw_=create_publisher<std_msgs::msg::Float32>("/srm_aim/gimbal_step/cmd_yaw",1);
    pitch_=create_publisher<std_msgs::msg::Float32>("/srm_aim/gimbal_step/cmd_pitch",1);
    debug_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/srm_aim/gimbal_step/debug",5);
    connected_=create_subscription<std_msgs::msg::Bool>("/srm_aim/gimbal_step/serial_connected",1,[this](std_msgs::msg::Bool::ConstSharedPtr m){connection_=m->data;connection_at_=now_s();refresh();});
    valid_=create_subscription<std_msgs::msg::Bool>("/srm_aim/gimbal_step/feedback_valid",1,[this](std_msgs::msg::Bool::ConstSharedPtr m){validity_=m->data;validity_at_=now_s();refresh();});
    feedback_=create_subscription<geometry_msgs::msg::Vector3Stamped>("/srm_aim/gimbal_step/feedback",10,[this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m){
      double age=(get_clock()->now().nanoseconds()-(int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec))*1e-9;
      int64_t stamp=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;
      if(m->header.frame_id!="gimbal_feedback_degrees"||m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000||age<-.02||age>.15||
        stamp<=last_feedback_stamp_||!std::isfinite(m->vector.z)){
        fault_=true;core_->stop("invalid_feedback_header");return;}
      last_feedback_stamp_=stamp;fault_=!core_->feedback(m->vector.x,m->vector.y,now_s());
    });
    step_=create_subscription<geometry_msgs::msg::Vector3Stamped>("/srm_aim/gimbal_step/relative_step",1,[this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m){
      refresh();int64_t ns=int64_t(m->header.stamp.sec)*1000000000+m->header.stamp.nanosec;
      double age=(get_clock()->now().nanoseconds()-ns)*1e-9;
      if(m->header.frame_id!="relative_feedback_degrees"||m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000||
        age<-.02||age>.2||ns<=armed_after_ns_||ns<=last_step_stamp_||m->vector.z!=0){core_->stop("invalid_or_replayed_manual_step");return;}
      last_step_stamp_=ns;core_->step(m->vector.x,m->vector.y,now_s());
    });
    arm_=create_service<std_srvs::srv::SetBool>("/srm_aim/gimbal_step/enable",[this](std_srvs::srv::SetBool::Request::SharedPtr request,std_srvs::srv::SetBool::Response::SharedPtr response){
      refresh();if(!request->data){core_->stop();response->success=true;}else {response->success=core_->arm(now_s());if(response->success)armed_after_ns_=get_clock()->now().nanoseconds();}
      response->message=core_->reason+"; stopping TX does NOT ensure motor stop";
    });
    stop_=create_service<std_srvs::srv::Trigger>("/srm_aim/gimbal_step/stop",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){core_->stop();response->success=true;response->message="Adapter sending stopped; serial cache and MCU target may persist. Use physical E-stop for actual stop.";});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{refresh();auto command=core_->tick(now_s());
      if(command){
        float y=static_cast<float>((*command)[0]),p=static_cast<float>((*command)[1]);
        if(!std::isfinite(y)||!std::isfinite(p)||y<core_->low[0]||y>core_->high[0]||p<core_->low[1]||p>core_->high[1])core_->stop("float32_boundary_rejected");
        else {std_msgs::msg::Float32 m;m.data=y;yaw_->publish(m);m.data=p;pitch_->publish(m);}
      }
      diagnostic();});
    RCLCPP_WARN(get_logger(),"Manual test only, default READ ONLY; no vision input; no fire output. Physical stop is NOT guaranteed by stopping messages.");
  }
private:
  static double now_s(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
  void refresh(){double n=now_s();bool graph=count_publishers("/srm_aim/gimbal_step/cmd_yaw")==1&&count_publishers("/srm_aim/gimbal_step/cmd_pitch")==1&&
      count_publishers("/srm_aim/gimbal_step/feedback")==1&&count_publishers("/srm_aim/gimbal_step/serial_connected")==1&&count_publishers("/srm_aim/gimbal_step/feedback_valid")==1;
    core_->status(connection_&&n-connection_at_<.15,validity_&&n-validity_at_<.15&&!fault_&&graph,n);}
  void diagnostic(){diagnostic_msgs::msg::DiagnosticArray m;m.header.stamp=get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="gimbal_manual_step/preflight";s.message=core_->reason;s.level=core_->armed?0:1;
    auto add=[&](std::string k,std::string v){diagnostic_msgs::msg::KeyValue kv;kv.key=k;kv.value=v;s.values.push_back(kv);};
    add("armed",core_->armed?"true":"false");add("ready",core_->ready(now_s())?"true":"false");
    add("allow_hardware_commands",core_->p.allow_hardware_commands?"true":"false");add("mechanical_limits_verified",core_->p.mechanical_limits_verified?"true":"false");
    add("absolute_target_protocol_verified",core_->p.absolute_target_protocol_verified?"true":"false");
    add("serial_connected",connection_?"true":"false");add("feedback_valid",validity_?"true":"false");add("feedback_fault",fault_?"true":"false");
    add("armable",core_->armable(now_s())?"true":"false");
    add("fire_flag","0");add("motor_stopped","unknown; not guaranteed by TX stop");
    add("anchor_valid",core_->anchor?"true":"false");
    if(core_->anchor){add("startup_yaw_deg",std::to_string((*core_->anchor)[0]));add("startup_pitch_deg",std::to_string((*core_->anchor)[1]));}
    for(int i=0;i<2;++i){std::string axis=i?"pitch":"yaw";add(axis+"_feedback_deg",std::to_string(core_->measured[i]));
      add(axis+"_min_deg",std::to_string(core_->low[i]));add(axis+"_max_deg",std::to_string(core_->high[i]));}
    m.status.push_back(s);debug_->publish(m);}
  std::unique_ptr<ManualStep> core_;bool connection_{false},validity_{false},fault_{true};
  double connection_at_{-1e9},validity_at_{-1e9};int64_t last_feedback_stamp_{-1},last_step_stamp_{-1},armed_after_ns_{-1};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_,pitch_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr debug_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connected_,valid_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr feedback_,step_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);int result=0;
  try{rclcpp::spin(std::make_shared<srm_gimbal_follower::ManualNode>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("manual_step"),"Manual adapter failed: %s",e.what());result=1;}
  if(rclcpp::ok()){rclcpp::shutdown();}return result;
}
