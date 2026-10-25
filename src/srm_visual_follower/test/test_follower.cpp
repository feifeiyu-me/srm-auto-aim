#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "srm_visual_follower/follower.hpp"
using namespace srm_visual_follower;
namespace {
builtin_interfaces::msg::Time stamp(double t){builtin_interfaces::msg::Time s;
  const int64_t n=std::llround(t*1e9);s.sec=n/1000000000;s.nanosec=n%1000000000;return s;}
int64_t now(double t){return std::llround(t*1e9);}
Track track(uint64_t id,double x=0,double y=0,double z=2){Track t;t.track_id=id;t.state_valid=true;
  t.state=Track::CONFIRMED;t.position.x=x;t.position.y=y;t.position.z=z;
  t.state_stamp=t.last_observation_stamp=stamp(10);return t;}
Input frame(std::vector<Track> tracks,double time=10){Input in;in.header.stamp=stamp(time);
  in.header.frame_id="source_camera";in.session_id="session_a";in.tracking_frame_id="camera_optical_frame";
  in.image_width=1440;in.image_height=1080;in.tracks=std::move(tracks);return in;}
}
TEST(Follower, OpticalAngleSignsAndNoExtrinsicsAssumption){
  Follower f;auto r=f.process(frame({track(7,1,-1,1)}),now(10));ASSERT_TRUE(r.target_valid);
  EXPECT_NEAR(r.camera_yaw_rad,std::acos(-1.)/4,1e-12);
  EXPECT_NEAR(r.camera_pitch_rad,std::atan2(1,std::sqrt(2)),1e-12);
  EXPECT_FALSE(r.reference_valid);EXPECT_EQ(r.reference_reason,"extrinsics_unavailable");
  EXPECT_EQ(r.position_source,"current");EXPECT_EQ(r.track_id,7u);
  auto t=track(7,-1,1,1);t.state_stamp=t.last_observation_stamp=stamp(10.1);
  r=f.process(frame({t},10.1),now(10.1));EXPECT_LT(r.camera_yaw_rad,0);EXPECT_LT(r.camera_pitch_rad,0);
}
TEST(Follower, PredictionPreferredThenExpiresToCurrent){
  Follower f;auto t=track(1);t.prediction_valid=true;t.prediction_stamp=stamp(10.1);
  t.predicted_position=t.position;t.predicted_position.x=1;
  auto r=f.process(frame({t}),now(10.01));ASSERT_TRUE(r.target_valid);
  EXPECT_EQ(r.position_source,"predicted");EXPECT_DOUBLE_EQ(r.position.x,1);
  EXPECT_EQ(r.position_stamp,t.prediction_stamp);EXPECT_FALSE(f.poll(now(10.05)));
  auto changed=f.poll(now(10.11));ASSERT_TRUE(changed);EXPECT_TRUE(changed->target_valid);
  EXPECT_EQ(changed->position_source,"current");EXPECT_DOUBLE_EQ(changed->position.x,0);
  EXPECT_EQ(changed->position_stamp,t.state_stamp);
  auto expired=f.poll(now(10.31));ASSERT_TRUE(expired);EXPECT_FALSE(expired->target_valid);
  EXPECT_DOUBLE_EQ(expired->camera_yaw_rad,0);EXPECT_FALSE(f.poll(now(10.4)));
}
TEST(Follower, InvalidPredictionFallsBackWithoutTrustingBadCoordinates){
  for(int mode=0;mode<5;++mode){Follower f;auto t=track(1);t.prediction_valid=true;
    t.prediction_stamp=stamp(10.1);t.predicted_position=t.position;
    if(mode==0)t.predicted_position.x=std::numeric_limits<double>::quiet_NaN();
    if(mode==1)t.predicted_position.z=-1;
    if(mode==2)t.prediction_stamp=stamp(9.9);
    if(mode==3)t.prediction_stamp=stamp(11);
    if(mode==4)t.prediction_stamp.nanosec=1000000000;
    auto r=f.process(frame({t}),now(10));EXPECT_TRUE(r.target_valid);EXPECT_EQ(r.position_source,"current");
  }
}
TEST(Follower, StickyDeterministicSelectionAndSessionChange){
  Follower f;auto a=track(20,.1),b=track(10,1);auto in=frame({b,a});
  EXPECT_EQ(f.process(in,now(10)).track_id,20u);
  b.position.x=0;a.state_stamp=b.state_stamp=stamp(10.1);
  in=frame({b,a},10.1);EXPECT_EQ(f.process(in,now(10.1)).track_id,20u);
  a.state_valid=false;in=frame({a,b},10.2);b.state_stamp=stamp(10.2);in.tracks[1]=b;
  EXPECT_EQ(f.process(in,now(10.2)).track_id,10u);
  a=track(20,0);b=track(10,1);a.state_stamp=b.state_stamp=stamp(10.21);
  in=frame({a,b},10.21);in.session_id="new_session";
  EXPECT_EQ(f.process(in,now(10.21)).track_id,20u);
}
TEST(Follower, StrategiesAndTieBreakDoNotDependOnArrayOrder){
  Parameters p;p.sticky_selection=false;
  auto in=frame({track(9,0,0,3),track(2,0,0,2)});
  p.selection_strategy="nearest";EXPECT_EQ(Follower(p).process(in,now(10)).track_id,2u);
  p.selection_strategy="lowest_id";EXPECT_EQ(Follower(p).process(in,now(10)).track_id,2u);
  p.selection_strategy="closest_to_axis";EXPECT_EQ(Follower(p).process(in,now(10)).track_id,2u);
  std::reverse(in.tracks.begin(),in.tracks.end());EXPECT_EQ(Follower(p).process(in,now(10)).track_id,2u);
}
TEST(Follower, StateFlagsAndNumericProtection){
  for(int mode=0;mode<6;++mode){Follower f;auto t=track(1);
    if(mode==0)t.state_valid=false;
    if(mode==1)t.state=Track::DELETED;
    if(mode==2)t.position.z=0;
    if(mode==3)t.position.x=std::numeric_limits<double>::infinity();
    if(mode==4)t.position.x=1e300;
    if(mode==5)t.position.z=-1;
    auto r=f.process(frame({t}),now(10));EXPECT_FALSE(r.target_valid);EXPECT_EQ(r.reason,"no_valid_target");
  }
}
TEST(Follower, SourceStateAndObservationFreshness){
  auto t=track(1);EXPECT_FALSE(Follower().process(frame({t}),now(10.31)).target_valid);
  EXPECT_FALSE(Follower().process(frame({t}),now(9.9)).target_valid);
  auto in=frame({t},10.4);EXPECT_FALSE(Follower().process(in,now(10.4)).target_valid);
  t.state_stamp=stamp(10.4);in=frame({t},10.4);EXPECT_FALSE(Follower().process(in,now(10.4)).target_valid);
  t.last_observation_stamp=stamp(10.5);in=frame({t},10.4);EXPECT_FALSE(Follower().process(in,now(10.4)).target_valid);
}
TEST(Follower, DuplicateOrderInvalidMetadataAndClockRewind){
  Follower f;auto in=frame({track(1)});ASSERT_TRUE(f.process(in,now(10)).target_valid);
  EXPECT_EQ(f.process(in,now(10.01)).reason,"duplicate_or_out_of_order");
  in.header.stamp=stamp(9.99);EXPECT_FALSE(f.process(in,now(10.02)).target_valid);
  in=frame({track(1)},10.1);in.tracking_frame_id="world";
  EXPECT_EQ(f.process(in,now(10.1)).reason,"invalid_input_metadata");
  EXPECT_TRUE(f.process(frame({track(1)}),now(10)).target_valid); // ROS clock rewound, resets chronology.
  auto reset=f.poll(now(1));ASSERT_TRUE(reset);EXPECT_EQ(reset->reason,"clock_reset");
}
TEST(Follower, BoundedInputAndDuplicateIds){
  Parameters p;p.max_tracks=1;
  EXPECT_EQ(Follower(p).process(frame({track(1),track(2)}),now(10)).reason,"track_limit");
  EXPECT_EQ(Follower().process(frame({track(1),track(1)}),now(10)).reason,"duplicate_track_id");
  Follower f;auto r=f.process(frame({}),now(10));EXPECT_FALSE(r.target_valid);EXPECT_TRUE(r.input_accepted);
  EXPECT_FALSE(f.poll(now(11)));
}
TEST(Follower, ExplicitExtrinsicRotationAndTranslation){
  Parameters p;p.extrinsics_valid=true;p.reference_frame="fixed_reference_flu";
  p.quaternion_xyzw={-.5,.5,-.5,.5};p.translation_m={.1,.2,.3};
  auto r=Follower(p).process(frame({track(1,1,-1,2)}),now(10));ASSERT_TRUE(r.reference_valid);
  EXPECT_NEAR(r.reference_position.x,2.1,1e-12);EXPECT_NEAR(r.reference_position.y,-.8,1e-12);
  EXPECT_NEAR(r.reference_position.z,1.3,1e-12);
  EXPECT_NEAR(r.reference_yaw_rad,std::atan2(-.8,2.1),1e-12);
  EXPECT_NEAR(r.reference_pitch_rad,std::atan2(1.3,std::hypot(2.1,.8)),1e-12);
  EXPECT_EQ(r.reference_reason,"reference_los_only_not_hardware_command");
}
TEST(Follower, BadReferenceDoesNotInvalidateCameraLos){
  Parameters p;p.extrinsics_valid=true;p.reference_frame="reference_flu";
  auto r=Follower(p).process(frame({track(1)}),now(10));
  EXPECT_TRUE(r.target_valid);EXPECT_FALSE(r.reference_valid);EXPECT_EQ(r.reference_reason,"invalid_reference_position");
}
TEST(Follower, InvalidParametersRejected){
  Parameters p;p.selection_strategy="unknown";EXPECT_THROW(Follower f(p),std::invalid_argument);
  p=Parameters{};p.extrinsics_valid=true;EXPECT_THROW(Follower f(p),std::invalid_argument);
  p.reference_frame="reference_flu";p.quaternion_xyzw={0,0,0,0};EXPECT_THROW(Follower f(p),std::invalid_argument);
  p=Parameters{};p.target_timeout_sec=0;EXPECT_THROW(Follower f(p),std::invalid_argument);
}
