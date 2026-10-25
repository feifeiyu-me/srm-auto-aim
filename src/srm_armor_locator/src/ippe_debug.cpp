#include "srm_armor_locator/ippe_debug.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace srm_armor_locator {
namespace {
using Status=diagnostic_msgs::msg::DiagnosticStatus;
std::string number(double v) {std::ostringstream s;s<<std::setprecision(12)<<v;return s.str();}
void add(Status & s,const std::string & key,const std::string & value) {
  diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;s.values.push_back(kv);
}
void real(Status & s,const std::string & key,double value){add(s,key,number(value));}
void flag(Status & s,const std::string & key,bool value){add(s,key,value?"true":"false");}
}
IppeDebug::IppeDebug(double interval_sec,int max_samples,const Parameters & p,Clock::time_point now)
:interval_(interval_sec),max_samples_(max_samples),p_(p),last_(now) {
  if(!std::isfinite(interval_)||interval_<0.1||interval_>3600||max_samples_<0||max_samples_>64)
    throw std::invalid_argument("IPPE debug interval must be 0.1..3600 s; max samples 0..64");
  samples_.reserve(max_samples_);
}
void IppeDebug::Metric::add(double v){
  if(!std::isfinite(v))return;
  if(!count)min=max=v;else{min=std::min(min,v);max=std::max(max,v);}
  ++count;sum+=v;
}
void IppeDebug::Statistics::add(const IppeTrace & t,const PoseResult & r){
  ++attempts;if(r.valid)++accepted;if(r.ambiguous)++ambiguous;
  if(!t.valid_candidates)++zero;else if(t.valid_candidates==1)++one;
  if(t.diagnostic_error)++trace_errors;
  if(t.pair_available){++pairs;gap.add(t.error_gap_px);separation.add(t.translation_separation_mm);
    distance_difference.add(std::abs(t.signed_distance_difference_mm));angle.add(t.rotation_difference_deg);
    if(r.ambiguous && t.error_gap_close){
      if(t.rotation_exceeds && t.translation_exceeds)++both;
      else if(t.rotation_exceeds)++angle_only;else if(t.translation_exceeds)++translation_only;
    }
  }
}
void IppeDebug::observe(const std_msgs::msg::Header & h,size_t index,const IppeTrace & t,const PoseResult & r){
  header_=h;total_.add(t,r);window_.add(t,r);
  if(!t.pair_available)return;
  if(samples_.size()>=size_t(max_samples_)){++window_.omitted;++total_.omitted;return;}
  Status s;s.name="armor_locator/ippe_pair";s.level=r.ambiguous?Status::WARN:Status::OK;s.message=r.reason;
  add(s,"stamp_sec",std::to_string(h.stamp.sec));add(s,"stamp_nanosec",std::to_string(h.stamp.nanosec));
  add(s,"frame_id",h.frame_id);add(s,"input_index",std::to_string(index));
  add(s,"candidate_order","ascending_reprojection_rms");
  flag(s,"pose_valid",r.valid);flag(s,"ambiguous",r.ambiguous);
  for(size_t i=0;i<2;++i){const auto prefix="candidate_"+std::to_string(i)+"_";
    real(s,prefix+"reprojection_error_px",t.candidates[i].error_px);
    for(int axis=0;axis<3;++axis)real(s,prefix+std::string("t_")+"xyz"[axis]+"_mm",t.candidates[i].translation[axis]*1000);
    real(s,prefix+"distance_mm",t.candidates[i].distance_mm);
  }
  for(int axis=0;axis<3;++axis)real(s,std::string("delta_t_")+"xyz"[axis]+"_mm",t.translation_delta_mm[axis]);
  real(s,"translation_separation_mm",t.translation_separation_mm);
  real(s,"signed_distance_difference_mm",t.signed_distance_difference_mm);
  real(s,"absolute_distance_difference_mm",std::abs(t.signed_distance_difference_mm));
  real(s,"rotation_difference_deg",t.rotation_difference_deg);real(s,"error_gap_px",t.error_gap_px);
  real(s,"ambiguity_gap_px",p_.ambiguity_gap_px);
  real(s,"ambiguity_translation_mm",p_.ambiguity_translation_m*1000);
  real(s,"ambiguity_angle_deg",p_.ambiguity_angle_deg);
  flag(s,"error_gap_close",t.error_gap_close);flag(s,"translation_exceeds",t.translation_exceeds);
  flag(s,"rotation_exceeds",t.rotation_exceeds);
  flag(s,"translation_triggered",r.ambiguous&&t.error_gap_close&&t.translation_exceeds);
  flag(s,"rotation_triggered",r.ambiguous&&t.error_gap_close&&t.rotation_exceeds);
  add(s,"trigger",!r.ambiguous?"none":t.rotation_exceeds?(t.translation_exceeds?"rotation_and_translation":"rotation"):"translation");
  samples_.push_back(std::move(s));
}
std::optional<diagnostic_msgs::msg::DiagnosticArray> IppeDebug::flush(Clock::time_point now){
  if(!window_.attempts || std::chrono::duration<double>(now-last_).count()<interval_)return std::nullopt;
  diagnostic_msgs::msg::DiagnosticArray out;out.header=header_;
  Status s;s.name="armor_locator/ippe_statistics";s.level=window_.ambiguous?Status::WARN:Status::OK;
  s.message="observation_only; counters per PnP attempt/target, not per image";
  for(const auto & entry:std::vector<std::pair<std::string,const Statistics*>>{{"window_",&window_},{"total_",&total_}}){
    const auto & a=*entry.second;const auto & prefix=entry.first;
    for(const auto & counter:std::vector<std::pair<std::string,uint64_t>>{
      {"attempts",a.attempts},{"accepted",a.accepted},{"ambiguous_ippe",a.ambiguous},
      {"zero_valid_candidates",a.zero},{"one_valid_candidate",a.one},{"pairs",a.pairs},
      {"rotation_only",a.angle_only},{"translation_only",a.translation_only},{"both",a.both},
      {"trace_errors",a.trace_errors},{"samples_omitted",a.omitted}})
      add(s,prefix+counter.first,std::to_string(counter.second));
    for(const auto & metric:std::vector<std::pair<std::string,const Metric*>>{
      {"error_gap_px",&a.gap},{"translation_separation_mm",&a.separation},
      {"absolute_distance_difference_mm",&a.distance_difference},{"rotation_difference_deg",&a.angle}}){
      const auto & m=*metric.second;add(s,prefix+metric.first+"_count",std::to_string(m.count));
      if(m.count){real(s,prefix+metric.first+"_min",m.min);real(s,prefix+metric.first+"_mean",m.sum/m.count);real(s,prefix+metric.first+"_max",m.max);}
    }
  }
  real(s,"window_elapsed_sec",std::chrono::duration<double>(now-last_).count());
  add(s,"sample_count",std::to_string(samples_.size()));
  out.status.push_back(std::move(s));for(auto & sample:samples_)out.status.push_back(std::move(sample));
  samples_.clear();window_=Statistics{};last_=now;return out;
}
}
