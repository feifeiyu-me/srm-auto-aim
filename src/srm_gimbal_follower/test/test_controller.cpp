#include <gtest/gtest.h>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include "srm_gimbal_follower/controller.hpp"
using namespace srm_gimbal_follower;
namespace {
Time time(double seconds){return {int64_t(std::llround(seconds*1e9)),seconds};}
builtin_interfaces::msg::Time stamp(double seconds){auto n=time(seconds).ros_ns;builtin_interfaces::msg::Time out;
  out.sec=n/1000000000;out.nanosec=n%1000000000;return out;}
Parameters simulation(){Parameters p;p.allow_unverified_simulation_calibration=true;return p;}
geometry_msgs::msg::Vector3Stamped feedback(double t,double yaw=0,double pitch=0){
  geometry_msgs::msg::Vector3Stamped m;m.header.stamp=stamp(t);m.header.frame_id="gimbal_feedback_degrees";
  m.vector.x=yaw;m.vector.y=pitch;return m;
}
diagnostic_msgs::msg::DiagnosticArray visual(double t,double yaw_right=10,double pitch_up=5,const std::string & id="1",bool predicted=false){
  diagnostic_msgs::msg::DiagnosticArray m;m.header.stamp=stamp(t);m.header.frame_id="camera";
  diagnostic_msgs::msg::DiagnosticStatus s;s.name="visual_follower/line_of_sight";
  auto add=[&](std::string k,std::string v){diagnostic_msgs::msg::KeyValue kv;kv.key=k;kv.value=v;s.values.push_back(kv);};
  auto num=[&](std::string k,double v){std::ostringstream stream;stream<<std::setprecision(17)<<v;add(k,stream.str());};
  for(const auto & prefix:{"source_stamp","state_stamp","observation_stamp","position_stamp"}){
    auto st=stamp(t+(predicted&&std::string(prefix)=="position_stamp"?.1:0));
    add(std::string(prefix)+"_sec",std::to_string(st.sec));add(std::string(prefix)+"_nanosec",std::to_string(st.nanosec));}
  const double pi=std::acos(-1.),a=yaw_right*pi/180,b=pitch_up*pi/180;
  Point p{3*std::cos(b)*std::sin(a),-3*std::sin(b),3*std::cos(b)*std::cos(a)};
  add("target_valid","true");add("camera_los_valid","true");add("mode","diagnostics_only");add("hardware_command_valid","false");
  add("camera_frame","camera_optical_frame");add("camera_angle_convention","yaw positive right; pitch positive up");
  add("session_id","test_session");add("track_id",id);add("position_source",predicted?"predicted":"current");
  num("camera_position_x_m",p.x);num("camera_position_y_m",p.y);num("camera_position_z_m",p.z);
  num("camera_yaw_rad",a);num("camera_pitch_rad",b);m.status.push_back(s);return m;
}
void replace(diagnostic_msgs::msg::DiagnosticArray & m,const std::string & key,const std::string & value){
  for(auto & kv:m.status[0].values)if(kv.key==key)kv.value=value;
}
}
TEST(Gimbal, ParsesActualFieldsAndPredictionSource){
  std::string reason;auto m=visual(10,10,5,"18446744073709551615",true);
  auto v=parse_visual(m,simulation(),reason);ASSERT_TRUE(v);EXPECT_EQ(v->id,"18446744073709551615");
  EXPECT_EQ(v->source,"predicted");EXPECT_EQ(v->position_ns,time(10.1).ros_ns);
  replace(m,"camera_yaw_rad","nan");EXPECT_FALSE(parse_visual(m,simulation(),reason));
}
TEST(Gimbal, RejectsInvalidFlagsMalformedDuplicatesAndInconsistentAngles){
  for(auto key:{"target_valid","camera_los_valid","mode","camera_frame","track_id","position_source","camera_pitch_rad"}){
    auto m=visual(10);replace(m,key,"invalid");std::string reason;EXPECT_FALSE(parse_visual(m,simulation(),reason));}
  auto m=visual(10);m.status[0].values.push_back(m.status[0].values[0]);std::string reason;
  EXPECT_FALSE(parse_visual(m,simulation(),reason));
  m=visual(10);replace(m,"camera_yaw_rad","0");EXPECT_FALSE(parse_visual(m,simulation(),reason));
}
TEST(Gimbal, RightAndUpProduceDecreasingFeedbackReferences){
  Controller c(simulation());ASSERT_TRUE(c.feedback(feedback(10),time(10)));ASSERT_TRUE(c.visual(visual(10),time(10)));
  auto out=c.tick(time(10));ASSERT_TRUE(out.simulation_reference_valid)<<out.reason;
  EXPECT_NEAR(out.yaw_target_deg,-10,1e-9);EXPECT_NEAR(out.pitch_target_deg,-5,1e-9);
  EXPECT_LT(out.yaw_reference_deg,0);EXPECT_LT(out.pitch_reference_deg,0);
  EXPECT_LE(std::abs(out.yaw_reference_deg),.6+1e-9);EXPECT_LE(std::abs(out.pitch_reference_deg),.6+1e-9);
}
TEST(Gimbal, HistoricalPoseAlignmentNotLatestFeedbackPlusOldError){
  Controller c(simulation());c.feedback(feedback(10,0),time(10));c.feedback(feedback(10.1,5),time(10.1));
  ASSERT_TRUE(c.visual(visual(10.05,10,0),time(10.1)));auto out=c.tick(time(10.1));
  ASSERT_TRUE(out.simulation_reference_valid)<<out.reason;EXPECT_NEAR(out.yaw_target_deg,-7.5,1e-9);
}
TEST(Gimbal, WaitingForAlignedFeedbackAndNoGapExtrapolation){
  Controller c(simulation());c.feedback(feedback(10),time(10));c.visual(visual(10.02),time(10.02));
  EXPECT_EQ(c.tick(time(10.02)).reason,"feedback_not_aligned");
  c.feedback(feedback(10.04),time(10.04));EXPECT_TRUE(c.tick(time(10.04)).simulation_reference_valid);
  Controller gap(simulation());gap.feedback(feedback(10),time(10));gap.feedback(feedback(10.15),time(10.15));
  gap.visual(visual(10.07),time(10.15));EXPECT_EQ(gap.tick(time(10.15)).reason,"feedback_not_aligned");
}
TEST(Gimbal, CalibrationAndHardwareModeGate){
  Controller c;c.feedback(feedback(10),time(10));c.visual(visual(10),time(10));
  EXPECT_EQ(c.tick(time(10)).reason,"calibration_not_verified");
  auto p=simulation();p.mode="hardware";EXPECT_THROW(Controller bad(p),std::invalid_argument);
  p=simulation();p.quaternion_xyzw={0,0,0,0};EXPECT_THROW(Controller bad(p),std::invalid_argument);
}
TEST(Gimbal, RateStepAndMechanicalLimits){
  auto p=simulation();p.yaw_min_deg=-5;p.pitch_min_deg=-3;Controller c(p);Output previous;
  for(int k=0;k<8;++k){double t=10+k*.02;c.feedback(feedback(t),time(t));c.visual(visual(t),time(t));
    auto out=c.tick(time(t));ASSERT_TRUE(out.simulation_reference_valid)<<out.reason;
    EXPECT_NEAR(out.yaw_target_deg,-5,1e-9);EXPECT_NEAR(out.pitch_target_deg,-3,1e-9);
    EXPECT_LE(std::abs(out.yaw_reference_deg),p.max_step_deg+1e-9);
    if(k)EXPECT_LE(std::abs(out.yaw_reference_deg-previous.yaw_reference_deg),p.max_rate_deg_sec*out.dt_sec+1e-9);
    previous=out;
  }
}
TEST(Gimbal, LossFeedbackAndTargetTimeoutHold){
  Controller c(simulation());c.feedback(feedback(10),time(10));c.visual(visual(10),time(10));ASSERT_TRUE(c.tick(time(10)).simulation_reference_valid);
  c.feedback(feedback(10.16),time(10.16));EXPECT_FALSE(c.tick(time(10.16)).simulation_reference_valid); // tick gap
  c.feedback(feedback(10.32),time(10.32));EXPECT_FALSE(c.tick(time(10.32)).simulation_reference_valid);
  Controller stale(simulation());stale.feedback(feedback(10),time(10));stale.visual(visual(10),time(10));
  EXPECT_EQ(stale.tick(time(10.16)).reason,"feedback_timeout");
  auto invalid=visual(10.02);replace(invalid,"target_valid","false");c.visual(invalid,time(10.34));
  EXPECT_FALSE(c.tick(time(10.34)).simulation_reference_valid);
}
TEST(Gimbal, FeedbackJumpNonfiniteOrderAndBoundedHistory){
  Controller c(simulation());ASSERT_TRUE(c.feedback(feedback(10),time(10)));
  EXPECT_FALSE(c.feedback(feedback(10),time(10)));EXPECT_FALSE(c.feedback(feedback(9.99),time(10)));
  EXPECT_FALSE(c.feedback(feedback(10.02,40),time(10.02)));EXPECT_EQ(c.tick(time(10.02)).reason,"feedback_fault");
  auto f=feedback(10.04);f.vector.y=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(c.feedback(f,time(10.04)));
  auto p=simulation();p.history_capacity=2;Controller bounded(p);
  bounded.feedback(feedback(10),time(10));bounded.feedback(feedback(10.02),time(10.02));bounded.feedback(feedback(10.04),time(10.04));
  bounded.visual(visual(10),time(10.04));EXPECT_EQ(bounded.tick(time(10.04)).reason,"feedback_not_aligned");
}
TEST(Gimbal, TargetSwitchRequiresDelayAndNewConfirmations){
  Controller c(simulation());c.feedback(feedback(10),time(10));c.visual(visual(10,5,0,"1"),time(10));c.tick(time(10));
  EXPECT_FALSE(c.visual(visual(10.1,5,0,"2"),time(10.1)));
  EXPECT_FALSE(c.visual(visual(10.1,5,0,"2"),time(10.2))); // replay cannot confirm pending target
  EXPECT_FALSE(c.visual(visual(10.2,5,0,"2"),time(10.2)));
  EXPECT_TRUE(c.visual(visual(10.31,5,0,"2"),time(10.31)));
  c.feedback(feedback(10.31),time(10.31));auto out=c.tick(time(10.31));EXPECT_FALSE(out.simulation_reference_valid); // long tick gap holds
  c.feedback(feedback(10.33),time(10.33));out=c.tick(time(10.33));EXPECT_TRUE(out.simulation_reference_valid)<<out.reason;
  EXPECT_EQ(out.id,"2");EXPECT_LE(std::abs(out.yaw_reference_deg),.6+1e-9);
}
TEST(Gimbal, PredictionExpiryAndSameFrameCurrentFallback){
  Controller c(simulation());c.feedback(feedback(10),time(10));c.visual(visual(10,5,0,"1",true),time(10));
  EXPECT_TRUE(c.tick(time(10)).simulation_reference_valid);
  c.feedback(feedback(10.14),time(10.14));c.tick(time(10.14));
  c.feedback(feedback(10.16),time(10.16));EXPECT_EQ(c.tick(time(10.16)).reason,"prediction_expired");
  ASSERT_TRUE(c.visual(visual(10,5,0,"1",false),time(10.16)));
  // Exact capture feedback is still available; source fallback does not invent a new pose time.
  c.feedback(feedback(10.18),time(10.18));EXPECT_TRUE(c.tick(time(10.18)).simulation_reference_valid);
}
TEST(Gimbal, OrientationRoundTripAndNoAbsoluteCameraAngleAssumption){
  Point p{2,.5,.3};auto world=orientation(p,20,10,5);auto recovered=orientation(world,20,10,5,true);
  EXPECT_NEAR(recovered.x,p.x,1e-12);EXPECT_NEAR(recovered.y,p.y,1e-12);EXPECT_NEAR(recovered.z,p.z,1e-12);
  Controller c(simulation());c.feedback(feedback(10,20,0),time(10));c.visual(visual(10,10,0),time(10));
  EXPECT_NEAR(c.tick(time(10)).yaw_target_deg,10,1e-9);
}
TEST(Gimbal, ClockResetAndReferenceDiscontinuityHoldSafely){
  Controller c(simulation());c.feedback(feedback(10),time(10));c.visual(visual(10),time(10));c.tick(time(10));
  c.feedback(feedback(10.02,3),time(10.02));
  EXPECT_EQ(c.tick(time(10.02)).reason,"reference_feedback_discontinuity");
  EXPECT_FALSE(c.tick(time(1)).simulation_reference_valid);
  c.feedback(feedback(1),time(1));c.visual(visual(1),time(1));
  EXPECT_TRUE(c.tick(time(1.02)).simulation_reference_valid);
}
