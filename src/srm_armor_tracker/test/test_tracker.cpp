#include <gtest/gtest.h>
#include <limits>
#include <algorithm>
#include <cmath>
#include "srm_armor_tracker/tracker.hpp"
using namespace srm_armor_tracker;
namespace {
Input frame(double t,std::vector<double> xs={},std::vector<uint8_t> colors={}){
  Input in;in.header.stamp.sec=static_cast<int>(t);in.header.stamp.nanosec=std::llround((t-in.header.stamp.sec)*1e9);
  in.header.frame_id="camera";in.optical_frame_id="camera_optical_frame";in.image_width=1440;in.image_height=1080;
  for(size_t i=0;i<xs.size();++i){srm_aim_interfaces::msg::ArmorLocation m;m.input_index=i;
    m.pose_valid=true;m.orientation_valid=true;m.position_valid=true;
    m.pose.position.x=xs[i];m.pose.position.z=3;m.position=m.pose.position;
    m.detection.color=colors.empty()?2:colors[i];m.detection.confidence=.85;
    m.digit_score=-1;in.locations.push_back(m);}return in;
}
const TrackMsg & byid(const Output & o,uint64_t id){
  auto it=std::find_if(o.tracks.begin(),o.tracks.end(),[&](const TrackMsg & m){return m.track_id==id;});
  if(it==o.tracks.end())throw std::runtime_error("missing track");return *it;
}
void healthy(const Output & o){for(auto & t:o.tracks){Matrix p;std::copy(t.covariance.begin(),t.covariance.end(),p.val);
  EXPECT_TRUE(valid_covariance(p));EXPECT_TRUE(std::isfinite(t.position.x));EXPECT_TRUE(std::isfinite(t.velocity.x));}}
}
TEST(Tracker, StationaryAndUnverified){
  Tracker tracker;Output o;
  for(int k=0;k<30;++k){o=*tracker.process(frame(10+k*.0625,{.2}));healthy(o);ASSERT_EQ(o.tracks.size(),1u);}
  const auto & t=o.tracks[0];EXPECT_EQ(t.state,TrackMsg::CONFIRMED);EXPECT_TRUE(t.velocity_valid);
  EXPECT_NEAR(t.velocity.x,0,1e-9);EXPECT_NEAR(t.position.x,.2,1e-9);
  EXPECT_FALSE(o.self_motion_compensated);EXPECT_EQ(o.tracking_frame_id,"camera_optical_frame");
  EXPECT_FALSE(t.digit_check_enabled);EXPECT_FALSE(t.digit_verified);EXPECT_EQ(t.digit,0);
}
TEST(Tracker, ConstantVelocityVariableDtPrediction){
  Tracker tracker;Output o;double t=10;
  for(int k=0;k<60;++k){t+=k%2?.07:.055;o=*tracker.process(frame(t,{.7*(t-10)}));healthy(o);}
  const auto & m=o.tracks[0];EXPECT_NEAR(m.velocity.x,.7,.02);EXPECT_NEAR(m.position.x,.7*(t-10),.01);
  EXPECT_TRUE(m.prediction_valid);EXPECT_NEAR(m.predicted_position.x,.7*(t-10+.1),.015);
  auto delta=(m.prediction_stamp.sec-m.state_stamp.sec)+(int64_t(m.prediction_stamp.nanosec)-m.state_stamp.nanosec)*1e-9;
  EXPECT_NEAR(delta,.1,1e-9);
}
TEST(Tracker, VelocityInitialization){
  Tracker tracker;auto first=*tracker.process(frame(10,{0}));EXPECT_FALSE(first.tracks[0].velocity_valid);
  EXPECT_FALSE(first.tracks[0].prediction_valid);
  auto second=*tracker.process(frame(10.06,{.03}));EXPECT_FALSE(second.tracks[0].velocity_valid);
  auto third=*tracker.process(frame(10.12,{.06}));EXPECT_TRUE(third.tracks[0].velocity_valid);
}
TEST(Tracker, MultiTargetOrderAndColor){
  Tracker tracker;auto a=*tracker.process(frame(10,{-.4,.4},{1,2}));auto red=a.tracks[0].track_id,blue=a.tracks[1].track_id;
  for(int k=1;k<15;++k){double x=-.4+k*.015;auto o=*tracker.process(frame(10+k*.0625,{-x,x},{2,1}));
    ASSERT_EQ(o.tracks.size(),2u);EXPECT_EQ(byid(o,red).input_index,1u);EXPECT_EQ(byid(o,blue).input_index,0u);healthy(o);}
}
TEST(Tracker, DifferentColorSamePosition){
  Tracker tracker;auto a=*tracker.process(frame(10,{-.02,.02},{1,2}));
  auto o=*tracker.process(frame(10.0625,{0,0},{2,1}));ASSERT_EQ(o.tracks.size(),2u);
  EXPECT_TRUE(byid(o,a.tracks[0].track_id).matched_this_frame);EXPECT_EQ(byid(o,a.tracks[0].track_id).color,1);
}
TEST(Tracker, SameColorCrossingCoastsConservatively){
  Tracker tracker;Output o;uint64_t left=0,right=0;bool ambiguous=false;
  for(int k=0;k<=20;++k){double x=-.3+k*.03;
    o=*tracker.process(frame(10+k*.0625,{x,-x}));
    if(k==0){left=o.tracks[0].track_id;right=o.tracks[1].track_id;}
    ASSERT_EQ(o.tracks.size(),2u);
    for(auto & m:o.tracks)if(m.reason=="association_ambiguous")ambiguous=true;
    healthy(o);
  }
  EXPECT_TRUE(ambiguous);EXPECT_GT(byid(o,left).position.x,0);EXPECT_LT(byid(o,right).position.x,0);
}
TEST(Tracker, ExactAmbiguityDoesNotSpawnOrSwap){
  Tracker tracker;auto a=*tracker.process(frame(10,{-.1,.1}));
  auto o=*tracker.process(frame(10.0625,{0,0}));ASSERT_EQ(o.tracks.size(),2u);
  for(auto & m:o.tracks){EXPECT_FALSE(m.matched_this_frame);EXPECT_EQ(m.reason,"association_ambiguous");}
  EXPECT_EQ(o.tracks[0].track_id,a.tracks[0].track_id);
}
TEST(Tracker, ShortLossReacquireAndLongLoss){
  Tracker tracker;Output o;for(int k=0;k<4;++k)o=*tracker.process(frame(10+k*.0625,{.1}));auto id=o.tracks[0].track_id;
  o=*tracker.process(frame(10.25));EXPECT_EQ(o.tracks[0].state,TrackMsg::COASTING);
  EXPECT_TRUE(o.tracks[0].prediction_valid);EXPECT_FALSE(o.tracks[0].matched_this_frame);
  o=*tracker.process(frame(10.3125,{.1}));EXPECT_EQ(o.tracks[0].track_id,id);EXPECT_TRUE(o.tracks[0].matched_this_frame);
  o=*tracker.process(frame(10.55));EXPECT_FALSE(o.tracks[0].prediction_valid);
  o=*tracker.process(frame(10.65));ASSERT_EQ(o.tracks.size(),1u);EXPECT_EQ(o.tracks[0].state,TrackMsg::DELETED);
  o=*tracker.process(frame(10.70));EXPECT_TRUE(o.tracks.empty());
  o=*tracker.process(frame(10.75,{.1}));EXPECT_NE(o.tracks[0].track_id,id);
}
TEST(Tracker, InvalidPnPEmptyAndJump){
  Tracker tracker;auto first=*tracker.process(frame(10,{0}));auto id=first.tracks[0].track_id;
  auto in=frame(10.0625,{0,0,0,0});in.locations[0].position_valid=false;in.locations[1].position_valid=false;
  in.locations[2].position.x=std::numeric_limits<double>::quiet_NaN();in.locations[3].position.z=-1;
  auto o=*tracker.process(in);ASSERT_EQ(o.tracks.size(),1u);EXPECT_FALSE(o.tracks[0].matched_this_frame);
  o=*tracker.process(frame(10.125,{10}));EXPECT_EQ(o.tracks.size(),2u);EXPECT_NEAR(byid(o,id).position.x,0,1e-9);
}
TEST(Tracker, DuplicateOutOfOrderAndClockReset){
  Tracker tracker;auto a=*tracker.process(frame(10,{0}));auto id=a.tracks[0].track_id;
  EXPECT_FALSE(tracker.process(frame(10,{5})));EXPECT_FALSE(tracker.process(frame(9.99,{5})));
  auto b=*tracker.process(frame(10.0625,{0}));EXPECT_EQ(b.tracks[0].track_id,id);
  auto c=*tracker.process(frame(1,{0}));EXPECT_EQ(c.reason,"clock_reset");EXPECT_NE(c.session_id,a.session_id);
  EXPECT_NE(c.tracks[0].track_id,id);EXPECT_FALSE(c.tracks[0].velocity_valid);
}
TEST(Tracker, CoordinateSourceSizeAndGapReset){
  Tracker tracker;auto a=*tracker.process(frame(10,{0}));
  auto in=frame(10.0625,{0});in.optical_frame_id="world";auto b=*tracker.process(in);
  EXPECT_EQ(b.reason,"invalid_coordinate_metadata");EXPECT_TRUE(b.tracks.empty());EXPECT_NE(b.session_id,a.session_id);
  a=*tracker.process(frame(10.125,{0}));in=frame(10.1875,{0});in.header.frame_id="camera2";
  b=*tracker.process(in);EXPECT_EQ(b.reason,"source_changed");EXPECT_NE(b.session_id,a.session_id);
  in=frame(10.25,{0});in.header.frame_id="camera2";in.image_width=640;
  b=*tracker.process(in);EXPECT_EQ(b.reason,"source_changed");
  in.header.stamp.sec=12;b=*tracker.process(in);EXPECT_EQ(b.reason,"long_gap_reset");
}
TEST(Tracker, InterruptOnceAndNewSession){
  Tracker tracker;auto a=*tracker.process(frame(10,{0}));auto o=*tracker.interrupt();
  EXPECT_EQ(o.reason,"input_timeout");EXPECT_EQ(o.tracks[0].state,TrackMsg::DELETED);
  EXPECT_FALSE(o.tracks[0].velocity_valid);EXPECT_FALSE(tracker.interrupt());
  auto b=*tracker.process(frame(10.0625,{0}));EXPECT_NE(a.session_id,b.session_id);
}
TEST(Tracker, BoundedTracksAndObservations){
  Parameters p;p.max_tracks=2;p.max_observations=3;Tracker tracker(p);
  auto a=*tracker.process(frame(10,{-5,0,5}));EXPECT_EQ(a.tracks.size(),2u);EXPECT_EQ(a.reason,"track_limit");
  auto b=*tracker.process(frame(10.0625,{-5,0,5,10}));EXPECT_TRUE(b.tracks.empty());EXPECT_EQ(b.reason,"observation_limit_reset");
}
TEST(Tracker, CovarianceValidationAndParameters){
  EXPECT_TRUE(valid_covariance(Matrix::eye()));auto a=Matrix::eye();a(0,1)=1;EXPECT_FALSE(valid_covariance(a));
  a=Matrix::eye();a(0,0)=-1;EXPECT_FALSE(valid_covariance(a));
  a(0,0)=std::numeric_limits<double>::infinity();EXPECT_FALSE(valid_covariance(a));
  Parameters p;p.measurement_xy=0;EXPECT_THROW(Tracker t(p),std::invalid_argument);
  p=Parameters{};p.prediction_sec=1;EXPECT_THROW(Tracker t(p),std::invalid_argument);
}
TEST(Tracker, HungarianGlobalNotGreedy){
  auto result=hungarian({{1,2,99},{1.1,99,99},{99,1,0}});
  ASSERT_EQ(result.size(),3u);EXPECT_EQ(result[0],1);EXPECT_EQ(result[1],0);EXPECT_EQ(result[2],2);
  EXPECT_THROW(hungarian({{1,2}}),std::invalid_argument);
}

TEST(Tracker, UnknownColorDoesNotEraseIdentity){
  Tracker tracker;auto a=*tracker.process(frame(10,{0},{1}));auto id=a.tracks[0].track_id;
  auto b=*tracker.process(frame(10.0625,{0},{0}));EXPECT_EQ(b.tracks[0].color,1);
  auto c=*tracker.process(frame(10.125,{0},{2}));EXPECT_FALSE(byid(c,id).matched_this_frame);
}
TEST(Tracker, InvalidTimestampAndMetadata){
  Tracker tracker;auto in=frame(10,{0});in.header.stamp.nanosec=1000000000u;EXPECT_FALSE(tracker.process(in));
  in=frame(10,{0});in.header.stamp.sec=-1;EXPECT_FALSE(tracker.process(in));
  in=frame(10,{0});in.image_width=0;auto o=*tracker.process(in);
  EXPECT_TRUE(o.tracks.empty());EXPECT_EQ(o.reason,"invalid_coordinate_metadata");
}

TEST(Tracker, NoisyStationaryPositionIsSmoothed){
  Tracker tracker;double squared=0;int count=0;
  for(int k=0;k<100;++k){double x=k%2?.03:-.03;auto o=*tracker.process(frame(10+k*.0625,{x}));
    ASSERT_EQ(o.tracks.size(),1u);healthy(o);
    if(k>20){squared+=o.tracks[0].position.x*o.tracks[0].position.x;++count;}}
  EXPECT_LT(std::sqrt(squared/count),.03);
}

TEST(Tracker, OrdinaryPositionUsesIndependentCoordinates){
  Tracker tracker;auto in=frame(10,{.25});
  in.locations[0].pose.position.x=100;in.locations[0].pose.position.z=100;
  auto out=*tracker.process(in);ASSERT_EQ(out.tracks.size(),1u);
  EXPECT_DOUBLE_EQ(out.tracks[0].position.x,.25);EXPECT_DOUBLE_EQ(out.tracks[0].position.z,3);
}
TEST(Tracker, PositionOnlyAndMixedObservationsPreserveKalmanResults){
  Tracker full,mixed;Output baseline,result;
  for(int k=0;k<30;++k){auto in=frame(10+k*.0625,{k*.02});baseline=*full.process(in);
    if(k%2){auto & m=in.locations[0];m.pose_valid=false;m.orientation_valid=false;
      m.ambiguous=true;m.reason="ambiguous_ippe";m.pose.position=geometry_msgs::msg::Point{};}
    result=*mixed.process(in);ASSERT_EQ(result.tracks.size(),1u);
    EXPECT_TRUE(result.tracks[0].matched_this_frame);
    EXPECT_EQ(result.tracks[0].track_id,baseline.tracks[0].track_id);
    EXPECT_EQ(result.tracks[0].position,baseline.tracks[0].position);
    EXPECT_EQ(result.tracks[0].velocity,baseline.tracks[0].velocity);
    EXPECT_EQ(result.tracks[0].covariance,baseline.tracks[0].covariance);
    EXPECT_EQ(result.tracks[0].state,baseline.tracks[0].state);healthy(result);
  }
  EXPECT_TRUE(result.tracks[0].velocity_valid);EXPECT_TRUE(result.tracks[0].prediction_valid);
}
TEST(Tracker, InvalidPositionNeverFallsBackToLegacyPose){
  Tracker tracker;auto in=frame(10,{.25});in.locations[0].position_valid=false;
  // A valid legacy Pose must not override an explicit invalid independent position.
  EXPECT_TRUE(tracker.process(in)->tracks.empty());
  in=frame(10.0625,{.25});in.locations[0].position_valid=false;in.locations[0].position=geometry_msgs::msg::Point{};
  in.locations[0].pose_valid=false;in.locations[0].orientation_valid=false;in.locations[0].pose.position=geometry_msgs::msg::Point{};
  EXPECT_TRUE(tracker.process(in)->tracks.empty());
}
TEST(Tracker, InvalidIndependentCoordinatesNeverFallBack){
  Tracker tracker;
  for(int k=0;k<3;++k){auto in=frame(10+k*.0625,{.25});
    if(k==0)in.locations[0].position.x=std::numeric_limits<double>::infinity();
    if(k==1)in.locations[0].position.z=0;
    if(k==2)in.locations[0].position.z=-1;
    EXPECT_TRUE(tracker.process(in)->tracks.empty());
  }
}
