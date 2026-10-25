#include "srm_gimbal_follower/controller.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
namespace srm_gimbal_follower {
namespace {
constexpr double PI=3.14159265358979323846;
double rad(double d){return d*PI/180;}
double deg(double r){return r*180/PI;}
double wrap(double d){return std::remainder(d,360.);}
double age(int64_t a,int64_t b){return (a-b)*1e-9;}
int64_t stamp(const builtin_interfaces::msg::Time & t){
  if(t.sec<0||t.nanosec>=1000000000)throw std::invalid_argument("invalid stamp");
  return int64_t(t.sec)*1000000000+t.nanosec;
}
bool finite(Point p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
Point camera_to_body(Point v,const Parameters & p){
  const auto & q=p.quaternion_xyzw;double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
  double x=q[0]/n,y=q[1]/n,z=q[2]/n,w=q[3]/n;
  Point a{y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x};
  return {v.x+2*w*a.x+2*(y*a.z-z*a.y)+p.translation_m[0],
    v.y+2*w*a.y+2*(z*a.x-x*a.z)+p.translation_m[1],
    v.z+2*w*a.z+2*(x*a.y-y*a.x)+p.translation_m[2]};
}
}
void Parameters::validate() const {
  if(mode!="simulation")throw std::invalid_argument("Version 1 is simulation-only; hardware/serial mode forbidden");
  for(double v:{kp,max_step_deg,max_rate_deg_sec,max_visual_correction_deg,visual_timeout_sec,feedback_timeout_sec,
    alignment_gap_sec,feedback_gap_sec,history_sec,feedback_max_jump_deg,feedback_max_rate_deg_sec,
    max_prediction_sec,max_range_m,min_depth_m,control_period_sec,max_tick_gap_sec})
    if(!std::isfinite(v)||v<=0||v>1000)throw std::invalid_argument("Invalid positive control parameter");
  for(double v:{deadband_deg,future_tolerance_sec,target_switch_delay_sec,target_switch_confirm_sec,prediction_grace_sec})
    if(!std::isfinite(v)||v<0||v>10)throw std::invalid_argument("Invalid nonnegative control parameter");
  for(double v:{yaw_zero_deg,pitch_zero_deg,roll_zero_deg,yaw_min_deg,yaw_max_deg,pitch_min_deg,pitch_max_deg})
    if(!std::isfinite(v)||std::abs(v)>360)throw std::invalid_argument("Invalid angle calibration/limit");
  for(double v:{yaw_feedback_sign,pitch_feedback_sign,roll_feedback_sign})
    if(v!=1&&v!=-1)throw std::invalid_argument("Feedback sign must be +/-1");
  for(double v:quaternion_xyzw)if(!std::isfinite(v))throw std::invalid_argument("Invalid calibration quaternion");
  const auto & q=quaternion_xyzw;
  double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
  for(double v:translation_m)if(!std::isfinite(v)||std::abs(v)>10)throw std::invalid_argument("Invalid calibration translation");
  if(std::abs(n-1)>1e-6||!std::isfinite(n)||kp>1||history_capacity<2||history_capacity>2000||
    yaw_min_deg>=yaw_max_deg||pitch_min_deg>=pitch_max_deg||min_depth_m>=max_range_m||
    max_prediction_sec>.5||alignment_gap_sec>feedback_gap_sec||history_sec<visual_timeout_sec||
    control_period_sec<.005||control_period_sec>.1||control_period_sec>max_tick_gap_sec)
    throw std::invalid_argument("Invalid calibration/history/control bounds");
}
std::optional<Visual> parse_visual(const diagnostic_msgs::msg::DiagnosticArray & msg,
  const Parameters & p,std::string & reason){
  try {
    if(msg.status.size()>8)throw std::invalid_argument("too many statuses");
    const diagnostic_msgs::msg::DiagnosticStatus * selected=nullptr;
    for(const auto & s:msg.status)if(s.name=="visual_follower/line_of_sight"){
      if(selected){throw std::invalid_argument("duplicate visual status");}
      selected=&s;}
    if(!selected||selected->values.size()>128)throw std::invalid_argument("missing/oversize visual status");
    std::map<std::string,std::string> v;
    for(const auto & kv:selected->values)if(!v.emplace(kv.key,kv.value).second)throw std::invalid_argument("duplicate diagnostic key");
    if(v.at("target_valid")!="true"||v.at("camera_los_valid")!="true"){
      reason="visual_target_invalid";return std::nullopt;}
    if(v.at("mode")!="diagnostics_only"||v.at("hardware_command_valid")!="false"||
      v.at("camera_frame")!="camera_optical_frame"||
      v.at("camera_angle_convention")!="yaw positive right; pitch positive up")
      throw std::invalid_argument("unexpected visual coordinate/mode contract");
    auto number=[&](const std::string & key){size_t used=0;const auto & s=v.at(key);double d=std::stod(s,&used);
      if(used!=s.size()||!std::isfinite(d)){throw std::invalid_argument("nonfinite/invalid number");}
      return d;};
    auto integer=[&](const std::string & key){const auto & s=v.at(key);
      if(s.empty()||s.size()>20||!std::all_of(s.begin(),s.end(),[](unsigned char c){return std::isdigit(c);}))
        throw std::invalid_argument("invalid integer");
      return std::stoull(s);};
    auto time=[&](const std::string & key){auto sec=integer(key+"_sec"),ns=integer(key+"_nanosec");
      if(sec>uint64_t(std::numeric_limits<int32_t>::max())||ns>=1000000000)throw std::invalid_argument("invalid time");
      return int64_t(sec)*1000000000+int64_t(ns);};
    Visual out;out.session=v.at("session_id");out.id=std::to_string(integer("track_id"));out.source=v.at("position_source");
    if(out.session.empty()||out.session.size()>128||(out.source!="current"&&out.source!="predicted"))
      throw std::invalid_argument("invalid target identity/source");
    out.source_ns=time("source_stamp");out.state_ns=time("state_stamp");
    out.observation_ns=time("observation_stamp");out.position_ns=time("position_stamp");
    if(stamp(msg.header.stamp)!=out.source_ns||msg.header.frame_id.empty()||
      out.observation_ns>out.state_ns||out.state_ns>out.source_ns||out.position_ns<out.state_ns||
      age(out.position_ns,out.state_ns)>p.max_prediction_sec||
      (out.source=="current"&&out.position_ns!=out.state_ns))throw std::invalid_argument("inconsistent visual times");
    out.point={number("camera_position_x_m"),number("camera_position_y_m"),number("camera_position_z_m")};
    out.yaw_rad=number("camera_yaw_rad");out.pitch_rad=number("camera_pitch_rad");
    if(!finite(out.point)||out.point.z<p.min_depth_m||std::hypot(out.point.x,out.point.y,out.point.z)>p.max_range_m||
      std::abs(out.yaw_rad-std::atan2(out.point.x,out.point.z))>1e-6||
      std::abs(out.pitch_rad-std::atan2(-out.point.y,std::hypot(out.point.x,out.point.z)))>1e-6)
      throw std::invalid_argument("invalid/inconsistent visual position/angles");
    reason="ok";return out;
  }catch(const std::exception &){reason="malformed_visual_diagnostic";return std::nullopt;}
}
Point orientation(Point v,double yaw,double pitch,double roll,bool inverse){
  const double a=rad(yaw),b=rad(-pitch),c=rad(roll);
  const double ca=std::cos(a),sa=std::sin(a),cb=std::cos(b),sb=std::sin(b),cc=std::cos(c),sc=std::sin(c);
  const double m[3][3]={{ca*cb,ca*sb*sc-sa*cc,ca*sb*cc+sa*sc},
    {sa*cb,sa*sb*sc+ca*cc,sa*sb*cc-ca*sc},{-sb,cb*sc,cb*cc}};
  double in[3]={v.x,v.y,v.z},out[3]={0,0,0};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)out[i]+=(inverse?m[j][i]:m[i][j])*in[j];
  return {out[0],out[1],out[2]};
}
Controller::Controller(Parameters p):p_(p){p_.validate();}
bool Controller::clock(Time t){
  auto reset=[&]{
    feedback_.clear();visual_.reset();reference_.reset();feedback_receipt_.reset();visual_receipt_.reset();
    locked_.clear();pending_.clear();last_visual_source_.reset();last_tick_.reset();
    last_ros_.reset();last_steady_.reset();feedback_fault_=false;rejection_="clock_reset";
  };
  if(t.ros_ns<0||!std::isfinite(t.steady_sec)){reset();return false;}
  if((last_ros_&&t.ros_ns<*last_ros_)||(last_steady_&&t.steady_sec<*last_steady_))reset();
  last_ros_=t.ros_ns;last_steady_=t.steady_sec;return true;
}
bool Controller::feedback(const geometry_msgs::msg::Vector3Stamped & msg,Time t){
  if(!clock(t))return false;
  int64_t ns;
  try{ns=stamp(msg.header.stamp);}catch(const std::exception &){feedback_fault_=true;feedback_.clear();rejection_="invalid_feedback_stamp";return false;}
  double y=msg.vector.x,p=msg.vector.y,r=msg.vector.z;
  if(msg.header.frame_id!="gimbal_feedback_degrees"||!std::isfinite(y)||!std::isfinite(p)||!std::isfinite(r)||
    y<p_.yaw_min_deg||y>p_.yaw_max_deg||p<p_.pitch_min_deg||p>p_.pitch_max_deg||std::abs(r)>180||
    age(t.ros_ns,ns)>p_.feedback_timeout_sec||age(ns,t.ros_ns)>p_.future_tolerance_sec){
    feedback_fault_=true;feedback_.clear();reference_.reset();rejection_="invalid_feedback";return false;}
  if(!feedback_.empty()){
    const auto & old=feedback_.back();if(ns<=old.stamp){rejection_="duplicate_or_out_of_order_feedback";return false;}
    double dt=age(ns,old.stamp);
    if(dt>p_.feedback_gap_sec){feedback_.clear();reference_.reset();}
    else if(std::abs(wrap(y-old.yaw))>std::min(p_.feedback_max_jump_deg,p_.feedback_max_rate_deg_sec*dt)||
      std::abs(p-old.pitch)>std::min(p_.feedback_max_jump_deg,p_.feedback_max_rate_deg_sec*dt)||
      std::abs(wrap(r-old.roll))>std::min(p_.feedback_max_jump_deg,p_.feedback_max_rate_deg_sec*dt)){
      feedback_fault_=true;feedback_.clear();reference_.reset();rejection_="feedback_jump";return false;}
  }
  feedback_.push_back({ns,y,p,r});feedback_receipt_=t.steady_sec;feedback_fault_=false;
  while(feedback_.size()>size_t(p_.history_capacity)||(!feedback_.empty()&&age(ns,feedback_.front().stamp)>p_.history_sec))feedback_.pop_front();
  return true;
}
bool Controller::visual(const diagnostic_msgs::msg::DiagnosticArray & msg,Time t){
  if(!clock(t))return false;
  std::string reason;auto v=parse_visual(msg,p_,reason);
  auto reject=[&](const std::string & why){visual_.reset();reference_.reset();rejection_=why;return false;};
  if(!v)return reject(reason);
  if(age(t.ros_ns,v->state_ns)>p_.visual_timeout_sec||age(v->source_ns,t.ros_ns)>p_.future_tolerance_sec||
    age(t.ros_ns,v->observation_ns)>p_.visual_timeout_sec||
    (v->source=="predicted"&&age(t.ros_ns,v->position_ns)>p_.prediction_grace_sec))return reject("stale_visual");
  const std::string key=v->session+"/"+v->id;
  if(last_visual_source_){
    if(v->source_ns<*last_visual_source_){rejection_="out_of_order_visual";return false;}
    if(v->source_ns==*last_visual_source_ && !(last_visual_kind_=="predicted"&&v->source=="current"&&key==last_visual_key_)){
      rejection_="duplicate_or_regressive_visual";return false;
    }
  }
  last_visual_source_=v->source_ns;last_visual_key_=key;last_visual_kind_=v->source;
  if(locked_.empty()){locked_=key;pending_.clear();}
  else if(key!=locked_){
    if(key!=pending_){pending_=key;pending_since_=t.steady_sec;pending_count_=1;}else pending_count_=std::min(pending_count_+1,2);
    if(!p_.allow_target_switch||t.steady_sec-locked_seen_<p_.target_switch_delay_sec||
      t.steady_sec-pending_since_<p_.target_switch_confirm_sec||pending_count_<2)return reject("target_switch_pending");
    locked_=key;pending_.clear();reference_.reset();
  }else pending_.clear();
  locked_seen_=t.steady_sec;visual_=*v;visual_receipt_=t.steady_sec;return true;
}
std::optional<Controller::Feedback> Controller::aligned(int64_t ns) const {
  for(size_t i=0;i<feedback_.size();++i){const auto & a=feedback_[i];if(a.stamp==ns)return a;
    if(i+1<feedback_.size()){
      const auto & b=feedback_[i+1];if(a.stamp<ns&&ns<b.stamp){
        double gap=age(b.stamp,a.stamp);if(gap>p_.alignment_gap_sec)return std::nullopt;
        double w=double(ns-a.stamp)/double(b.stamp-a.stamp);
        return Feedback{ns,a.yaw+w*wrap(b.yaw-a.yaw),a.pitch+w*(b.pitch-a.pitch),a.roll+w*wrap(b.roll-a.roll)};
      }
    }
  }return std::nullopt;
}
Output Controller::stop(const std::string & reason){
  reference_.reset();Output out;out.reason=reason;
  if(visual_){out.session=visual_->session;out.id=visual_->id;out.source=visual_->source;
    out.source_ns=visual_->source_ns;out.state_ns=visual_->state_ns;out.position_ns=visual_->position_ns;}
  if(!feedback_.empty())out.feedback_ns=feedback_.back().stamp;
  return out;
}
Output Controller::tick(Time t){
  if(!clock(t))return stop("invalid_clock");
  double dt=last_tick_?t.steady_sec-*last_tick_:p_.control_period_sec;last_tick_=t.steady_sec;
  if(dt<=0||dt>p_.max_tick_gap_sec)return stop("control_tick_gap");
  if(!p_.calibration_verified&&!p_.allow_unverified_simulation_calibration)return stop("calibration_not_verified");
  if(feedback_fault_)return stop("feedback_fault");
  if(feedback_.empty()||!feedback_receipt_||t.steady_sec-*feedback_receipt_>p_.feedback_timeout_sec||
    age(t.ros_ns,feedback_.back().stamp)>p_.feedback_timeout_sec)return stop("feedback_timeout");
  if(!visual_||!visual_receipt_)return stop(rejection_.empty()?"visual_unavailable":rejection_);
  const auto & v=*visual_;
  if(t.steady_sec-*visual_receipt_>p_.visual_timeout_sec||age(t.ros_ns,v.observation_ns)>p_.visual_timeout_sec||
    age(t.ros_ns,v.state_ns)>p_.visual_timeout_sec)return stop("target_timeout");
  if(v.source=="predicted"&&age(t.ros_ns,v.position_ns)>p_.prediction_grace_sec)return stop("prediction_expired");
  auto at=aligned(v.state_ns);if(!at)return stop("feedback_not_aligned");
  Point body=camera_to_body(v.point,p_);
  if(!finite(body)||body.x<p_.min_depth_m)return stop("invalid_transformed_target");
  Point world=orientation(body,p_.yaw_zero_deg+p_.yaw_feedback_sign*at->yaw,
    p_.pitch_zero_deg+p_.pitch_feedback_sign*at->pitch,p_.roll_zero_deg+p_.roll_feedback_sign*at->roll);
  double y=(deg(std::atan2(world.y,world.x))-p_.yaw_zero_deg)/p_.yaw_feedback_sign;
  double p=(deg(std::atan2(world.z,std::hypot(world.x,world.y)))-p_.pitch_zero_deg)/p_.pitch_feedback_sign;
  const auto & current=feedback_.back();y=current.yaw+wrap(y-current.yaw);
  if(!std::isfinite(y)||!std::isfinite(p)||std::abs(wrap(y-at->yaw))>p_.max_visual_correction_deg||
    std::abs(p-at->pitch)>p_.max_visual_correction_deg)return stop("excessive_or_invalid_visual_correction");
  Output out;out.dt_sec=dt;out.session=v.session;out.id=v.id;out.source=v.source;
  out.source_ns=v.source_ns;out.state_ns=v.state_ns;out.position_ns=v.position_ns;out.feedback_ns=current.stamp;
  out.yaw_target_deg=std::clamp(y,p_.yaw_min_deg,p_.yaw_max_deg);
  out.pitch_target_deg=std::clamp(p,p_.pitch_min_deg,p_.pitch_max_deg);
  out.limited=out.yaw_target_deg!=y||out.pitch_target_deg!=p;
  out.yaw_error_deg=wrap(out.yaw_target_deg-current.yaw);out.pitch_error_deg=out.pitch_target_deg-current.pitch;
  if(!reference_)reference_=std::array<double,2>{current.yaw,current.pitch};
  std::array<double,2> next;
  for(int axis=0;axis<2;++axis){double measured=axis?current.pitch:current.yaw;
    double error=axis?out.pitch_error_deg:out.yaw_error_deg;
    if(std::abs(error)<=p_.deadband_deg)error=0;
    double proposed=measured+p_.kp*error;
    double increment=std::min(p_.max_step_deg,p_.max_rate_deg_sec*dt);
    double low=std::max({measured-p_.max_step_deg,(*reference_)[axis]-increment,axis?p_.pitch_min_deg:p_.yaw_min_deg});
    double high=std::min({measured+p_.max_step_deg,(*reference_)[axis]+increment,axis?p_.pitch_max_deg:p_.yaw_max_deg});
    if(low>high)return stop("reference_feedback_discontinuity");
    next[axis]=std::clamp(proposed,low,high);out.limited=out.limited||next[axis]!=proposed;
  }
  reference_=next;out.yaw_reference_deg=next[0];out.pitch_reference_deg=next[1];
  out.simulation_reference_valid=true;out.reason="simulation_following";return out;
}
}
