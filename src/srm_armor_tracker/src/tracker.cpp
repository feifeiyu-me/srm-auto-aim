#include "srm_armor_tracker/tracker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace srm_armor_tracker {
namespace {
constexpr double BAD=1e9;
int64_t ns(const builtin_interfaces::msg::Time & t) {return int64_t(t.sec)*1000000000+t.nanosec;}
builtin_interfaces::msg::Time time_msg(int64_t n) {
  builtin_interfaces::msg::Time t; t.sec=n/1000000000;t.nanosec=n%1000000000;return t;
}
cv::Vec3d position(const srm_aim_interfaces::msg::ArmorLocation & m) {
  return {m.pose.position.x,m.pose.position.y,m.pose.position.z};
}
bool valid(const srm_aim_interfaces::msg::ArmorLocation & m) {
  auto z=position(m);
  return m.pose_valid && !m.ambiguous && cv::checkRange(cv::Mat(z)) && z[2]>0 &&
    m.detection.color<=2 && std::isfinite(m.detection.confidence) &&
    m.detection.confidence>=0 && m.detection.confidence<=1;
}
cv::Matx33d noise(const Parameters & p) {
  return cv::Matx33d(p.measurement_xy*p.measurement_xy,0,0,0,
    p.measurement_xy*p.measurement_xy,0,0,0,p.measurement_z*p.measurement_z);
}
cv::Matx33d innovation(const Matrix & p, const Parameters & params) {
  auto s=noise(params);for(int i=0;i<3;++i)for(int j=0;j<3;++j)s(i,j)+=p(i,j);return s;
}
}
void Parameters::validate() const {
  for(double v:{acceleration_noise,measurement_xy,measurement_z,initial_velocity_std,
    gate_squared,max_distance_m,coast_sec,delete_sec,max_gap_sec,time_reset_sec,
    velocity_min_span_sec,input_timeout_sec})
    if(!std::isfinite(v)||v<1e-9||v>1000) throw std::invalid_argument("Tracker parameters must be finite and positive");
  if(!std::isfinite(prediction_sec)||prediction_sec<0||prediction_sec>0.5 ||
    !std::isfinite(ambiguity_margin)||ambiguity_margin<0 ||
    max_tracks<1||max_tracks>256||max_observations<1||max_observations>256||
    confirmation_hits<2||confirmation_hits>1000||velocity_hits<2||velocity_hits>1000||coast_sec>delete_sec||delete_sec>max_gap_sec)
    throw std::invalid_argument("Invalid tracker limits/lifecycle");
}
std::vector<int> hungarian(const std::vector<std::vector<double>> & a) {
  const int n=a.size();if(!n)return {};
  if(n>512)throw std::invalid_argument("Assignment limit exceeded");
  for(const auto & row:a){if(int(row.size())!=n)throw std::invalid_argument("Assignment must be square");
    for(double c:row)if(!std::isfinite(c)||std::abs(c)>BAD)throw std::invalid_argument("Invalid assignment cost");}
  std::vector<double> u(n+1),v(n+1);std::vector<int> p(n+1),way(n+1);
  for(int i=1;i<=n;++i){p[0]=i;int j0=0;std::vector<double> minv(n+1,BAD*10);std::vector<bool> used(n+1);
    do{used[j0]=true;int i0=p[j0],j1=0;double delta=BAD*10;
      for(int j=1;j<=n;++j)if(!used[j]){double cur=a[i0-1][j-1]-u[i0]-v[j];
        if(cur<minv[j]){minv[j]=cur;way[j]=j0;}if(minv[j]<delta){delta=minv[j];j1=j;}}
      for(int j=0;j<=n;++j)if(used[j]){u[p[j]]+=delta;v[j]-=delta;}else minv[j]-=delta;
      j0=j1;
    }while(p[j0]);
    do{int j1=way[j0];p[j0]=p[j1];j0=j1;}while(j0);
  }
  std::vector<int> out(n,-1);for(int j=1;j<=n;++j)out[p[j]-1]=j-1;return out;
}
bool valid_covariance(const Matrix & p) {
  if(!cv::checkRange(cv::Mat(p)))return false;
  if(cv::norm(cv::Mat(p-p.t()),cv::NORM_INF)>1e-8)return false;
  cv::Mat eigen;return cv::eigen(cv::Mat(p),eigen)&&eigen.at<double>(5)>=-1e-9;
}
Tracker::Tracker(Parameters p):p_(p) {
  p_.validate();prefix_=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}
void Tracker::reset(){tracks_.clear();last_stamp_.reset();++generation_;}
Output Tracker::base(const Input & in,const std::string & reason)const {
  Output o;o.header=in.header;o.image_width=in.image_width;o.image_height=in.image_height;
  o.tracking_frame_id="camera_optical_frame";o.self_motion_compensated=false;
  o.session_id=prefix_+"_"+std::to_string(generation_);o.reason=reason;return o;
}
void Tracker::predict(Track & t,int64_t stamp)const {
  const double dt=(stamp-t.stamp)*1e-9;Matrix f=Matrix::eye(),q=Matrix::zeros();
  const double a=p_.acceleration_noise*p_.acceleration_noise;
  for(int i=0;i<3;++i){f(i,i+3)=dt;q(i,i)=a*dt*dt*dt*dt/4;
    q(i,i+3)=q(i+3,i)=a*dt*dt*dt/2;q(i+3,i+3)=a*dt*dt;}
  t.x=f*t.x;t.p=f*t.p*f.t()+q;t.p=(t.p+t.p.t())*.5;t.stamp=stamp;t.matched=false;
}
bool Tracker::update(Track & t,const srm_aim_interfaces::msg::ArmorLocation & m)const {
  auto s=innovation(t.p,p_);bool ok=false;auto inv=s.inv(cv::DECOMP_CHOLESKY,&ok);if(!ok)return false;
  cv::Matx<double,6,3> k;
  for(int i=0;i<6;++i)for(int j=0;j<3;++j){k(i,j)=0;for(int a=0;a<3;++a)k(i,j)+=t.p(i,a)*inv(a,j);}
  auto z=position(m);cv::Vec3d residual(z[0]-t.x[0],z[1]-t.x[1],z[2]-t.x[2]);t.x+=k*residual;
  Matrix ikh=Matrix::eye();for(int i=0;i<6;++i)for(int j=0;j<3;++j)ikh(i,j)-=k(i,j);
  t.p=ikh*t.p*ikh.t()+k*noise(p_)*k.t();t.p=(t.p+t.p.t())*.5;
  if(!cv::checkRange(cv::Mat(t.x))||!valid_covariance(t.p))return false;
  t.observation=m;if(m.detection.color)t.color=m.detection.color;t.last_seen=t.stamp;
  const int limit=std::max(p_.confirmation_hits,p_.velocity_hits);
  t.hits=std::min(t.hits+1,limit);t.streak=std::min(t.streak+1,limit);t.matched=true;
  if(t.streak>=p_.confirmation_hits)t.confirmed=true;
  t.reason="measurement_update";return true;
}
TrackMsg Tracker::message(const Track & t,uint8_t state,const std::string & reason)const {
  TrackMsg m;m.track_id=t.id;m.state=state;m.reason=reason;m.state_stamp=time_msg(t.stamp);
  m.last_observation_stamp=time_msg(t.last_seen);
  m.state_valid=t.x[2]>0 && state!=TrackMsg::DELETED && (t.stamp-t.last_seen)*1e-9<=p_.coast_sec;
  m.position.x=t.x[0];m.position.y=t.x[1];m.position.z=t.x[2];
  m.velocity.x=t.x[3];m.velocity.y=t.x[4];m.velocity.z=t.x[5];
  m.velocity_valid=m.state_valid && t.hits>=p_.velocity_hits && (t.last_seen-t.first)*1e-9>=p_.velocity_min_span_sec &&
    state!=TrackMsg::DELETED && (t.stamp-t.last_seen)*1e-9<=p_.coast_sec;
  std::copy(std::begin(t.p.val),std::end(t.p.val),m.covariance.begin());
  m.matched_this_frame=t.matched && state!=TrackMsg::DELETED;
  m.input_index=m.matched_this_frame?t.observation.input_index:0;
  m.color=t.color;m.detection_confidence=t.observation.detection.confidence;
  m.digit_check_enabled=t.observation.digit_check_enabled;
  m.digit_verified=m.digit_check_enabled && t.observation.digit_verified && t.observation.digit==2;
  m.digit=m.digit_verified?2:0;
  m.prediction_valid=m.velocity_valid;
  if(m.prediction_valid){m.prediction_stamp=time_msg(t.stamp+int64_t(p_.prediction_sec*1e9));
    m.predicted_position.x=t.x[0]+t.x[3]*p_.prediction_sec;
    m.predicted_position.y=t.x[1]+t.x[4]*p_.prediction_sec;
    m.predicted_position.z=t.x[2]+t.x[5]*p_.prediction_sec;
    m.prediction_valid=std::isfinite(m.predicted_position.x)&&std::isfinite(m.predicted_position.y)&&
      std::isfinite(m.predicted_position.z)&&m.predicted_position.z>0;}
  return m;
}
std::optional<Output> Tracker::interrupt() {
  if(!last_stamp_)return std::nullopt;
  auto o=base(metadata_,"input_timeout");for(const auto & t:tracks_)o.tracks.push_back(message(t,TrackMsg::DELETED,"input_timeout"));
  reset();return o;
}
std::optional<Output> Tracker::process(const Input & in) {
  if(in.header.stamp.sec<0 || in.header.stamp.nanosec>=1000000000)return std::nullopt;
  int64_t stamp=ns(in.header.stamp);std::string reason="ok";
  if(last_stamp_ && stamp<=*last_stamp_){
    if(stamp==*last_stamp_ || (*last_stamp_-stamp)*1e-9<p_.time_reset_sec)return std::nullopt;
    reset();reason="clock_reset";
  }
  if(in.optical_frame_id!="camera_optical_frame" || in.header.frame_id.empty() || !in.image_width || !in.image_height){
    auto o=base(in,"invalid_coordinate_metadata");reset();o.session_id=prefix_+"_"+std::to_string(generation_);return o;
  }
  if(last_stamp_ && (in.header.frame_id!=metadata_.header.frame_id || in.image_width!=metadata_.image_width ||
    in.image_height!=metadata_.image_height)){reset();reason="source_changed";}
  if(last_stamp_ && (stamp-*last_stamp_)*1e-9>p_.max_gap_sec){reset();reason="long_gap_reset";}
  if(in.locations.size()>size_t(p_.max_observations)) {
    // Reject the entire over-limit frame; never silently select by array order.
    reset();auto o=base(in,"observation_limit_reset");return o;
  }
  auto out=base(in,reason);last_stamp_=stamp;metadata_.header=in.header;
  metadata_.image_width=in.image_width;metadata_.image_height=in.image_height;
  std::vector<srm_aim_interfaces::msg::ArmorLocation> obs;
  for(const auto & m:in.locations)if(valid(m))obs.push_back(m);
  for(auto it=tracks_.begin();it!=tracks_.end();){
    if((stamp-it->last_seen)*1e-9>p_.delete_sec){out.tracks.push_back(message(*it,TrackMsg::DELETED,"observation_timeout"));it=tracks_.erase(it);}
    else {predict(*it,stamp);if(!cv::checkRange(cv::Mat(it->x))||!valid_covariance(it->p)){
      // Last valid state is not used again. Report reset via frame reason; do not publish NaNs.
      out.reason="numeric_reset";it=tracks_.erase(it);
    }else ++it;}
  }
  const size_t n=tracks_.size(),m=obs.size(),size=n+m;
  std::vector<std::vector<double>> cost(n,std::vector<double>(m,BAD));
  for(size_t i=0;i<n;++i)for(size_t j=0;j<m;++j){
    auto & t=tracks_[i];auto color=t.color;
    if(color && obs[j].detection.color && color!=obs[j].detection.color)continue;
    auto z=position(obs[j]);cv::Vec3d d(z[0]-t.x[0],z[1]-t.x[1],z[2]-t.x[2]);
    if(cv::norm(d)>p_.max_distance_m)continue;
    bool ok=false;auto inv=innovation(t.p,p_).inv(cv::DECOMP_CHOLESKY,&ok);if(!ok)continue;
    double v=d.dot(inv*d);if(std::isfinite(v)&&v>=0&&v<=p_.gate_squared)cost[i][j]=v;
  }
  std::vector<bool> blocked_t(n),blocked_o(m);
  // Ambiguous row/column: block every competing candidate, including births.
  auto ambiguous=[&](std::vector<std::pair<double,size_t>> candidates){
    std::sort(candidates.begin(),candidates.end());return candidates.size()>1 &&
      candidates[1].first-candidates[0].first<=p_.ambiguity_margin;
  };
  for(size_t i=0;i<n;++i){std::vector<std::pair<double,size_t>> c;
    for(size_t j=0;j<m;++j)if(cost[i][j]<BAD)c.push_back({cost[i][j],j});
    if(ambiguous(c)){blocked_t[i]=true;for(auto x:c)blocked_o[x.second]=true;}}
  for(size_t j=0;j<m;++j){std::vector<std::pair<double,size_t>> c;
    for(size_t i=0;i<n;++i)if(cost[i][j]<BAD)c.push_back({cost[i][j],i});
    if(ambiguous(c)){blocked_o[j]=true;for(auto x:c)blocked_t[x.second]=true;}}
  std::vector<std::vector<double>> a(size,std::vector<double>(size,0));
  for(size_t i=0;i<n;++i){for(size_t j=0;j<m;++j)a[i][j]=blocked_t[i]||blocked_o[j]?BAD:cost[i][j];
    for(size_t j=m;j<size;++j)a[i][j]=p_.gate_squared+1;}
  auto assignment=hungarian(a);std::vector<bool> used(m);
  for(size_t i=0;i<n;++i){int j=assignment[i];auto & t=tracks_[i];
    if(j>=0 && size_t(j)<m && a[i][j]<BAD){used[j]=true;
      if(!update(t,obs[j])){out.reason="numeric_reset";t.reason="numeric_reset";t.hits=0;}}
    else {t.streak=0;t.reason=blocked_t[i]?"association_ambiguous":"no_matching_observation";}
  }
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[](const Track & t){return t.hits==0;}),tracks_.end());
  for(size_t j=0;j<m;++j)if(!used[j]&&!blocked_o[j]){
    if(tracks_.size()>=size_t(p_.max_tracks)){out.reason="track_limit";break;}
    Track t;t.id=next_id_++;auto z=position(obs[j]);for(int k=0;k<3;++k)t.x[k]=z[k];
    t.p=Matrix::zeros();auto r=noise(p_);for(int k=0;k<3;++k){t.p(k,k)=r(k,k);t.p(k+3,k+3)=p_.initial_velocity_std*p_.initial_velocity_std;}
    t.first=t.last_seen=t.stamp=stamp;t.matched=true;t.observation=obs[j];t.color=obs[j].detection.color;tracks_.push_back(t);
  }
  for(const auto & t:tracks_){uint8_t state=t.confirmed?TrackMsg::CONFIRMED:TrackMsg::TENTATIVE;
    if(!t.matched)state=TrackMsg::COASTING;
    auto msg=message(t,state,t.reason);
    if((stamp-t.last_seen)*1e-9>p_.coast_sec){msg.velocity_valid=false;msg.prediction_valid=false;msg.reason="coast_prediction_expired";}
    out.tracks.push_back(msg);}
  return out;
}
}
