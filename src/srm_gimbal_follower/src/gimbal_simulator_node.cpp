#include "srm_gimbal_follower/controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <limits>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
namespace srm_gimbal_follower {
using Diagnostic=diagnostic_msgs::msg::DiagnosticArray;
class Simulator:public rclcpp::Node {
public:
  Simulator():Node("gimbal_follower_simulator"){
    declare_parameter("publish_visual",true);declare_parameter("publish_feedback",true);
    declare_parameter("invalid_visual",false);declare_parameter("invalid_feedback",false);
    declare_parameter("target_id",1);declare_parameter("target_yaw_flu_deg",-10.0);declare_parameter("target_pitch_up_deg",6.0);
    for(const auto & name:{"/srm_aim/gimbal_follower/sim_state_stamped","/srm_aim/gimbal_follower/sim_visual_debug"})
      if(get_node_topics_interface()->resolve_topic_name(name).rfind("/srm_aim/gimbal_follower/",0)!=0)
        throw std::invalid_argument("Simulator publishers must stay in simulation namespace");
    feedback_=create_publisher<geometry_msgs::msg::Vector3Stamped>("/srm_aim/gimbal_follower/sim_state_stamped",10);
    visual_=create_publisher<Diagnostic>("/srm_aim/gimbal_follower/sim_visual_debug",5);
    reference_=create_subscription<Diagnostic>("/srm_aim/gimbal_follower/sim_reference",5,[this](Diagnostic::ConstSharedPtr msg){
      try{
        if(msg->status.size()!=1||msg->status[0].name!="srm_gimbal_follower/reference")throw std::invalid_argument("invalid reference status");
        std::map<std::string,std::string> v;for(const auto & kv:msg->status[0].values)v[kv.key]=kv.value;
        if(v.at("mode")!="simulation"||v.at("real_hardware_enabled")!="false"||v.at("serial_tx_enabled")!="false"||
          v.at("simulation_reference_valid")!="true"){hold();return;}
        double y=std::stod(v.at("yaw_reference_deg")),p=std::stod(v.at("pitch_reference_deg"));
        if(!std::isfinite(y)||!std::isfinite(p)||std::abs(y)>170||std::abs(p)>45){hold();return;}
        target_yaw_=y;target_pitch_=p;last_reference_=std::chrono::steady_clock::now();
      }catch(const std::exception &){hold();}
    });
    last_tick_=std::chrono::steady_clock::now();timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    RCLCPP_INFO(get_logger(),"Synthetic two-axis plant + visual source only; no hardware I/O");
  }
private:
  void hold(){target_yaw_=yaw_;target_pitch_=pitch_;last_reference_.reset();}
  void tick(){
    auto steady=std::chrono::steady_clock::now();double dt=std::chrono::duration<double>(steady-last_tick_).count();last_tick_=steady;
    if(!last_reference_||std::chrono::duration<double>(steady-*last_reference_).count()>.10)hold();
    dt=std::clamp(dt,0.,.05);
    yaw_+=std::clamp((target_yaw_-yaw_)/.15,-60.,60.)*dt;
    pitch_+=std::clamp((target_pitch_-pitch_)/.15,-60.,60.)*dt;
    const builtin_interfaces::msg::Time stamp=get_clock()->now();
    if(get_parameter("publish_feedback").as_bool()){
      geometry_msgs::msg::Vector3Stamped m;m.header.stamp=stamp;m.header.frame_id="gimbal_feedback_degrees";
      m.vector.x=get_parameter("invalid_feedback").as_bool()?std::numeric_limits<double>::quiet_NaN():yaw_;
      m.vector.y=pitch_;feedback_->publish(m);
    }
    if(++counter_%3||!get_parameter("publish_visual").as_bool())return;
    const double pi=std::acos(-1.),a=get_parameter("target_yaw_flu_deg").as_double()*pi/180;
    const double b=get_parameter("target_pitch_up_deg").as_double()*pi/180;
    Point world{3*std::cos(b)*std::cos(a),3*std::cos(b)*std::sin(a),3*std::sin(b)};
    Point body=orientation(world,yaw_,-pitch_,0,true);Point camera{-body.y,-body.z,body.x};
    Diagnostic msg;msg.header.stamp=stamp;msg.header.frame_id="synthetic_optical_image";
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="visual_follower/line_of_sight";
    auto text=[&](const std::string & key,const std::string & value){diagnostic_msgs::msg::KeyValue v;v.key=key;v.value=value;s.values.push_back(v);};
    auto number=[&](const std::string & key,double value){std::ostringstream v;v<<std::setprecision(17)<<value;text(key,v.str());};
    for(const auto & key:{"source_stamp","state_stamp","observation_stamp","position_stamp"}){
      text(std::string(key)+"_sec",std::to_string(stamp.sec));text(std::string(key)+"_nanosec",std::to_string(stamp.nanosec));}
    const bool valid=!get_parameter("invalid_visual").as_bool()&&camera.z>.05&&finite_point(camera);
    text("target_valid",valid?"true":"false");text("camera_los_valid",valid?"true":"false");
    text("hardware_command_valid","false");text("mode","diagnostics_only");
    text("camera_frame","camera_optical_frame");text("camera_angle_convention","yaw positive right; pitch positive up");
    text("session_id","synthetic_gimbal_scene");text("track_id",std::to_string(get_parameter("target_id").as_int()));
    text("position_source","current");number("camera_position_x_m",camera.x);number("camera_position_y_m",camera.y);number("camera_position_z_m",camera.z);
    number("camera_yaw_rad",std::atan2(camera.x,camera.z));number("camera_pitch_rad",std::atan2(-camera.y,std::hypot(camera.x,camera.z)));
    s.message=valid?"synthetic_camera_los_valid":"synthetic_invalid_target";s.level=valid?0:1;
    msg.status.push_back(std::move(s));visual_->publish(msg);
  }
  static bool finite_point(Point p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
  double yaw_{0},pitch_{0},target_yaw_{0},target_pitch_{0};unsigned counter_{0};
  std::chrono::steady_clock::time_point last_tick_;std::optional<std::chrono::steady_clock::time_point> last_reference_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr feedback_;
  rclcpp::Publisher<Diagnostic>::SharedPtr visual_;rclcpp::Subscription<Diagnostic>::SharedPtr reference_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);int code=0;
  try{rclcpp::spin(std::make_shared<srm_gimbal_follower::Simulator>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("gimbal_simulator"),"Simulator failed: %s",e.what());code=1;}
  if(rclcpp::ok()){rclcpp::shutdown();}
  return code;
}
