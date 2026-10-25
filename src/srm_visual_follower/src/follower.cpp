#include "srm_visual_follower/follower.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <vector>
namespace srm_visual_follower {
namespace {
std::optional<int64_t> ns(const builtin_interfaces::msg::Time & t){
  if(t.sec<0 || t.nanosec>=1000000000)return std::nullopt;
  return int64_t(t.sec)*1000000000+t.nanosec;
}
double seconds(int64_t a,int64_t b){return (a-b)*1e-9;}
double distance(const Point & p){return std::hypot(p.x,p.y,p.z);}
bool point_valid(const Point & p,const Parameters & limits){
  return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&
    p.z>=limits.min_depth_m&&distance(p)<=limits.max_distance_m;
}
Point transform(const Point & p,const Parameters & params){
  const auto & q=params.quaternion_xyzw;
  const double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
  const double x=q[0]/n,y=q[1]/n,z=q[2]/n,w=q[3]/n;
  const double ax=y*p.z-z*p.y,ay=z*p.x-x*p.z,az=x*p.y-y*p.x;
  Point out;out.x=p.x+2*w*ax+2*(y*az-z*ay)+params.translation_m[0];
  out.y=p.y+2*w*ay+2*(z*ax-x*az)+params.translation_m[1];
  out.z=p.z+2*w*az+2*(x*ay-y*ax)+params.translation_m[2];return out;
}
}
void Parameters::validate() const {
  if(selection_strategy!="closest_to_axis" && selection_strategy!="nearest" && selection_strategy!="lowest_id")
    throw std::invalid_argument("selection_strategy must be closest_to_axis, nearest or lowest_id");
  for(double v:{target_timeout_sec,max_state_age_sec,input_timeout_sec,max_prediction_horizon_sec,min_depth_m,max_distance_m})
    if(!std::isfinite(v)||v<=0||v>1000)throw std::invalid_argument("Invalid finite positive follower limits");
  if(max_tracks<1||max_tracks>256||min_depth_m>=max_distance_m||
    max_prediction_horizon_sec>0.5||!std::isfinite(future_tolerance_sec)||future_tolerance_sec<0||future_tolerance_sec>.5)
    throw std::invalid_argument("Invalid follower bounds");
  for(double v:translation_m)if(!std::isfinite(v)||std::abs(v)>1000)throw std::invalid_argument("Invalid extrinsic translation");
  for(double v:quaternion_xyzw)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite extrinsic quaternion");
  if(extrinsics_valid){
    const auto & q=quaternion_xyzw;
    const double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3]));
    if(reference_frame.empty()||reference_frame=="camera_optical_frame"||!std::isfinite(n)||std::abs(n-1)>1e-6)
      throw std::invalid_argument("Explicit FLU reference frame and unit quaternion required for extrinsics");
  }
}
Follower::Follower(Parameters p):p_(p){p_.validate();}
Result Follower::invalidate(const std::string & reason){
  Result out;out.header=last_.header;out.session_id=session_;out.reason=reason;
  selected_.reset();cached_.reset();last_=out;return out;
}
std::optional<Result> Follower::evaluate(const Track & t,const std_msgs::msg::Header & header,
  const std::string & session,int64_t now) const {
  if(!t.state_valid||t.state>Track::COASTING||!point_valid(t.position,p_))return std::nullopt;
  const auto state=ns(t.state_stamp),seen=ns(t.last_observation_stamp),source=ns(header.stamp);
  if(!state||!seen||!source||*seen>*state||*state>*source ||
    seconds(now,*state)>p_.max_state_age_sec||seconds(*state,now)>p_.future_tolerance_sec||
    seconds(now,*seen)>p_.target_timeout_sec)return std::nullopt;
  Result r;r.header=header;r.session_id=session;r.track_id=t.track_id;
  r.state_stamp=t.state_stamp;r.observation_stamp=t.last_observation_stamp;
  r.position=t.position;r.position_stamp=t.state_stamp;r.position_source="current";
  auto prediction=ns(t.prediction_stamp);
  if(t.prediction_valid&&prediction&&point_valid(t.predicted_position,p_)&&*prediction>=*state&&
    *prediction>=now&&seconds(*prediction,*state)<=p_.max_prediction_horizon_sec&&
    seconds(*prediction,now)<=p_.max_prediction_horizon_sec){
    r.position=t.predicted_position;r.position_stamp=t.prediction_stamp;r.position_source="predicted";
  }
  r.camera_yaw_rad=std::atan2(r.position.x,r.position.z);
  r.camera_pitch_rad=std::atan2(-r.position.y,std::hypot(r.position.x,r.position.z));
  r.target_valid=true;r.reason="camera_los_valid";
  if(p_.extrinsics_valid){
    r.reference_position=transform(r.position,p_);
    const auto & v=r.reference_position;
    if(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&
      v.x>=p_.min_depth_m&&distance(v)<=p_.max_distance_m){
      r.reference_yaw_rad=std::atan2(v.y,v.x);
      r.reference_pitch_rad=std::atan2(v.z,std::hypot(v.x,v.y));
      r.reference_valid=true;r.reference_reason="reference_los_only_not_hardware_command";
    }else {r.reference_position=Point{};r.reference_reason="invalid_reference_position";}
  }
  return r;
}
Result Follower::process(const Input & in,int64_t now){
  if(last_now_&&now<*last_now_){last_stamp_.reset();invalidate("clock_reset");}
  last_now_=now;
  last_.header=in.header;
  const auto stamp=ns(in.header.stamp);
  if(now<0||!stamp||in.tracking_frame_id!="camera_optical_frame"||in.header.frame_id.empty()||
    in.session_id.empty()||!in.image_width||!in.image_height)return invalidate("invalid_input_metadata");
  if(in.tracks.size()>size_t(p_.max_tracks))return invalidate("track_limit");
  if(in.session_id!=session_||in.header.frame_id!=source_frame_||in.image_width!=width_||in.image_height!=height_){
    last_stamp_.reset();invalidate("source_or_session_changed");
  }
  session_=in.session_id;source_frame_=in.header.frame_id;width_=in.image_width;height_=in.image_height;
  if(last_stamp_&&*stamp<=*last_stamp_)return invalidate("duplicate_or_out_of_order");
  if(seconds(now,*stamp)>p_.max_state_age_sec||seconds(*stamp,now)>p_.future_tolerance_sec)
    return invalidate("stale_or_future_source_header");
  std::unordered_set<uint64_t> ids;
  for(const auto & t:in.tracks)if(!ids.insert(t.track_id).second)return invalidate("duplicate_track_id");
  last_stamp_=stamp;
  struct Candidate {const Track * track;Result result;double score;};
  std::vector<Candidate> candidates;
  for(const auto & t:in.tracks){auto r=evaluate(t,in.header,in.session_id,now);if(!r)continue;
    double score=0;
    if(p_.selection_strategy=="nearest")score=distance(r->position);
    else if(p_.selection_strategy=="closest_to_axis")score=std::atan2(std::hypot(r->position.x,r->position.y),r->position.z);
    candidates.push_back({&t,*r,score});
  }
  if(candidates.empty()){auto r=invalidate("no_valid_target");r.input_accepted=true;last_=r;return r;}
  std::sort(candidates.begin(),candidates.end(),[](const auto & a,const auto & b){
    return a.score!=b.score?a.score<b.score:a.track->track_id<b.track->track_id;});
  auto chosen=candidates.begin();
  if(p_.sticky_selection&&selected_){auto old=std::find_if(candidates.begin(),candidates.end(),
    [&](const Candidate & c){return c.track->track_id==*selected_;});if(old!=candidates.end())chosen=old;}
  cached_=*chosen->track;selected_=chosen->track->track_id;
  last_=chosen->result;last_.input_accepted=true;return last_;
}
std::optional<Result> Follower::poll(int64_t now){
  if(last_now_&&now<*last_now_){last_now_=now;last_stamp_.reset();return invalidate("clock_reset");}
  last_now_=now;if(!cached_)return std::nullopt;
  auto r=evaluate(*cached_,last_.header,session_,now);
  if(!r)return invalidate("target_timeout_or_invalid_state");
  if(r->position_source!=last_.position_source){last_=*r;return last_;}
  return std::nullopt;
}
}
