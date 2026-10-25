#include "srm_armor_tracker/tracker.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
namespace srm_armor_tracker {
class TrackerNode:public rclcpp::Node {
public:
  TrackerNode():Node("armor_tracker_node") {
    rcl_interfaces::msg::ParameterDescriptor desc;desc.read_only=true;Parameters p;
    auto real=[&](const char * name,double & value){value=declare_parameter<double>(name,value,desc);};
    auto integer=[&](const char * name,int & value){value=declare_parameter<int>(name,value,desc);};
    integer("max_tracks",p.max_tracks);integer("max_observations",p.max_observations);
    integer("confirmation_hits",p.confirmation_hits);integer("velocity_hits",p.velocity_hits);
    real("acceleration_noise",p.acceleration_noise);real("measurement_xy",p.measurement_xy);
    real("measurement_z",p.measurement_z);real("initial_velocity_std",p.initial_velocity_std);
    real("gate_squared",p.gate_squared);real("max_distance_m",p.max_distance_m);
    real("ambiguity_margin",p.ambiguity_margin);real("coast_sec",p.coast_sec);real("delete_sec",p.delete_sec);
    real("max_gap_sec",p.max_gap_sec);real("time_reset_sec",p.time_reset_sec);
    real("velocity_min_span_sec",p.velocity_min_span_sec);real("prediction_sec",p.prediction_sec);
    real("input_timeout_sec",p.input_timeout_sec);p.validate();timeout_=p.input_timeout_sec;
    core_=std::make_unique<Tracker>(p);
    const auto input=declare_parameter<std::string>("locations_topic","/srm_aim/armor_locations",desc);
    const auto output=declare_parameter<std::string>("tracks_topic","/srm_aim/armor_tracks",desc);
    auto topics=get_node_topics_interface();
    if(topics->resolve_topic_name(input)==topics->resolve_topic_name(output))throw std::invalid_argument("Input and output must differ");
    output_=create_publisher<Output>(output,rclcpp::QoS(5).reliable());
    debug_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/srm_aim/armor_tracker/debug",5);
    input_=create_subscription<Input>(input,rclcpp::QoS(5).reliable(),[this](Input::ConstSharedPtr in){
      const auto start=std::chrono::steady_clock::now();
      auto out=core_->process(*in);
      if(out){last_receive_=start;timeout_reported_=false;output_->publish(*out);}
      else ++rejected_;
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      report(in->header,out?out->reason:"rejected_timestamp",ms,out?&*out:nullptr);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      if(last_receive_ && !timeout_reported_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-*last_receive_).count()>timeout_){
        auto out=core_->interrupt();timeout_reported_=true;
        if(out){output_->publish(*out);report(out->header,"input_timeout",0,&*out);}
      }
    });
    RCLCPP_WARN(get_logger(),"FIXED CAMERA ONLY: optical camera-relative position/velocity; self_motion_compensated=false; no control I/O");
  }
private:
  void report(const std_msgs::msg::Header & header,const std::string & reason,double ms,const Output * out){
    diagnostic_msgs::msg::DiagnosticArray d;d.header=header;
    diagnostic_msgs::msg::DiagnosticStatus s;s.name="armor_tracker/frame";s.message=reason;
    s.level=reason=="ok"?0:1;
    auto add=[&](const std::string & key,const std::string & value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;s.values.push_back(kv);};
    add("processing_ms",std::to_string(ms));add("rejected_timestamps",std::to_string(rejected_));
    add("self_motion_compensated","false");add("coordinate_mode","fixed_camera_relative");
    if(out){add("session_id",out->session_id);add("tracks",std::to_string(out->tracks.size()));}
    d.status.push_back(s);
    if(out)for(const auto & t:out->tracks){diagnostic_msgs::msg::DiagnosticStatus ts;
      ts.name="armor_tracker/track_"+std::to_string(t.track_id);ts.message=t.reason;
      ts.level=t.matched_this_frame?0:1;
      for(const auto & kv:std::vector<std::pair<std::string,std::string>>{
        {"state",std::to_string(t.state)},{"velocity_valid",t.velocity_valid?"true":"false"},
        {"digit_verified",t.digit_verified?"true":"false"},
        {"observation_age_sec",std::to_string((int64_t(t.state_stamp.sec)-t.last_observation_stamp.sec)+
          (int64_t(t.state_stamp.nanosec)-t.last_observation_stamp.nanosec)*1e-9)}}){
        diagnostic_msgs::msg::KeyValue value;value.key=kv.first;value.value=kv.second;ts.values.push_back(value);}
      d.status.push_back(ts);}
    debug_->publish(d);
  }
  std::unique_ptr<Tracker> core_;double timeout_;uint64_t rejected_{0};bool timeout_reported_{false};
  std::optional<std::chrono::steady_clock::time_point> last_receive_;
  rclcpp::Subscription<Input>::SharedPtr input_;rclcpp::Publisher<Output>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr debug_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv){rclcpp::init(argc,argv);int code=0;
  try{rclcpp::spin(std::make_shared<srm_armor_tracker::TrackerNode>());}
  catch(const std::exception & e){RCLCPP_ERROR(rclcpp::get_logger("armor_tracker_node"),"Tracker failed: %s",e.what());code=1;}
  if(rclcpp::ok()) {rclcpp::shutdown();}
  return code;
}
