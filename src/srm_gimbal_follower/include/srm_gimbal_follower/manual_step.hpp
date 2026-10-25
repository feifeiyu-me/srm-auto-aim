#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <stdexcept>
namespace srm_gimbal_follower {
// Independent manual adapter. No vision or simulation calibration dependency.
struct StepParameters {
  bool allow_hardware_commands{false},mechanical_limits_verified{false},absolute_target_protocol_verified{false};
  double yaw_min{0},yaw_max{0},pitch_min{0},pitch_max{0},rate{2};
};
class ManualStep {
public:
  explicit ManualStep(StepParameters p={}):p(p){
    if(!std::isfinite(p.rate)||p.rate<=0||p.rate>5)throw std::invalid_argument("rate must be >0 and <=5 deg/s");
    for(double v:{p.yaw_min,p.yaw_max,p.pitch_min,p.pitch_max})
      if(!std::isfinite(v))throw std::invalid_argument("mechanical bounds must be finite");
  }
  StepParameters p;bool armed{false};std::string reason{"read_only"};
  std::optional<std::array<double,2>> anchor;std::array<double,2> measured{},reference{},goal{},low{},high{};
  void stop(const std::string & why="manual_stop"){armed=false;moving=false;reason=why;}
  void status(bool connected,bool valid,double now){
    status_at=now;healthy=connected&&valid;if(!healthy)stop("serial_disconnected_or_invalid");
  }
  bool feedback(double yaw,double pitch,double now){
    if(!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(now)||
      (feedback_at&&now<=*feedback_at)){stop("invalid_feedback");samples.clear();feedback_at.reset();return false;}
    if(feedback_at && now-*feedback_at<.15 &&
      (std::abs(yaw-measured[0])>.5||std::abs(pitch-measured[1])>.5)){
      stop("feedback_jump");samples.clear();feedback_at.reset();return false;}
    measured={yaw,pitch};feedback_at=now;
    samples.push_back({now,yaw,pitch});while(samples.size()>50||now-samples.front()[0]>.4)samples.pop_front();
    if(!anchor && samples.size()>=10 && now-samples.front()[0]>=.20){
      double loY=yaw,hiY=yaw,loP=pitch,hiP=pitch;
      for(auto s:samples){loY=std::min(loY,s[1]);hiY=std::max(hiY,s[1]);loP=std::min(loP,s[2]);hiP=std::max(hiP,s[2]);}
      if(hiY-loY<=.1&&hiP-loP<=.1)anchor=measured;
    }
    if(armed&&!inside(measured)){stop("feedback_outside_test_range");}
    return true;
  }
  bool ready(double now){
    if(!healthy||!status_at||now-*status_at>.15||!feedback_at||now-*feedback_at>.15)return false;
    if(!anchor)return false;
    low={std::max((*anchor)[0]-3,p.yaw_min),std::max((*anchor)[1]-2,p.pitch_min)};
    high={std::min((*anchor)[0]+3,p.yaw_max),std::min((*anchor)[1]+2,p.pitch_max)};
    return p.absolute_target_protocol_verified&&p.mechanical_limits_verified&&p.yaw_min<p.yaw_max&&p.pitch_min<p.pitch_max&&
      low[0]<high[0]&&low[1]<high[1]&&inside(measured);
  }
  bool armable(double now){return !armed&&p.allow_hardware_commands&&ready(now)&&stable(now)&&(!last_sent_at||now-*last_sent_at>=.3);}
  bool arm(double now){
    if(!armable(now)){stop("preflight_not_ready");return false;}
    armed=true;moving=false;reference=goal=measured;armed_at=now;last_tick=now;reason="armed_waiting_manual_step";return true;
  }
  bool step(double dy,double dp,double now){
    if(!armed||moving||!ready(now)||!std::isfinite(dy)||!std::isfinite(dp)||
      std::abs(dy)>1||std::abs(dp)>1||(dy==0&&dp==0)){stop("step_rejected");return false;}
    auto next=measured;next[0]+=dy;next[1]+=dp;
    if(!inside(next)){stop("step_outside_range");return false;}
    reference=measured;goal=next;moving=true;step_at=now;arrived_at.reset();reason="manual_step";return true;
  }
  std::optional<std::array<double,2>> tick(double now){
    if(!armed)return std::nullopt;
    if(!ready(now)){stop("feedback_or_status_timeout");return std::nullopt;}
    if(now-armed_at>5){stop("enable_lease_expired");return std::nullopt;}
    if(!moving){last_tick=now;return std::nullopt;}
    double dt=now-last_tick;last_tick=now;
    if(dt<=0||dt>.05){stop("tick_gap");return std::nullopt;}
    if(now-step_at>2.5){stop("motion_timeout");return std::nullopt;}
    for(int i=0;i<2;++i){
      if(std::abs(reference[i]-measured[i])>.5){stop("excessive_tracking_error");return std::nullopt;}
    }
    double dy=goal[0]-reference[0],dp=goal[1]-reference[1];
    double length=std::hypot(dy,dp),scale=length>0?std::min(1.,p.rate*dt/length):1.;
    reference[0]+=dy*scale;reference[1]+=dp*scale;
    if(!inside(reference)){stop("reference_outside_range");return std::nullopt;}
    if(std::abs(goal[0]-measured[0])<.05&&std::abs(goal[1]-measured[1])<.05){
      if(!arrived_at)arrived_at=now;
      if(now-*arrived_at>.3){stop("step_complete_auto_disarmed");return std::nullopt;}
    }
    last_sent_at=now;return reference;
  }
private:
  bool stable(double now)const{
    if(samples.size()<10||now-samples.front()[0]<.2)return false;
    for(auto sample:samples)if(std::abs(sample[1]-measured[0])>.1||std::abs(sample[2]-measured[1])>.1)return false;
    return true;
  }
  bool inside(std::array<double,2> value)const{return value[0]>=low[0]&&value[0]<=high[0]&&value[1]>=low[1]&&value[1]<=high[1];}
  bool healthy{false},moving{false};double armed_at{0},step_at{0},last_tick{0};
  std::optional<double> feedback_at,status_at,arrived_at,last_sent_at;std::deque<std::array<double,3>> samples;
};
}
