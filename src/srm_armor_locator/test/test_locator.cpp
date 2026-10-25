#include <fstream>
#include <chrono>
#include <filesystem>
#include <limits>
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "srm_armor_locator/locator.hpp"
using namespace srm_armor_locator;
Calibration camera(){return {{1440,1080},(cv::Mat_<double>(3,3)<<1800,0,720,0,1800,540,0,0,1),cv::Mat::zeros(5,1,CV_64F)};}
Corners project(const Calibration & c,cv::Vec3d r,cv::Vec3d t){
  std::vector<cv::Point2d> pixels;cv::projectPoints(object_points({}),r,t,c.k,c.d,pixels);
  Corners out;for(size_t i=0;i<4;++i)out[i]=pixels[i];return out;
}
TEST(PnP, SyntheticObliqueProjectionRecoversPoseAndDistinctDistances){
  const auto c=camera();const cv::Vec3d t(.3,.1,2),r(.2,.4,.1);
  const auto result=solve(c,project(c,r,t));
  ASSERT_TRUE(result.valid)<<result.reason;EXPECT_LT(cv::norm(result.translation-t),1e-4);
  EXPECT_NEAR(result.translation[2],2,1e-4);EXPECT_GT(cv::norm(result.translation),result.translation[2]);
  EXPECT_LT(result.error,.001);
}
TEST(PnP, DistortionAndRotationPreserveCornerSemantics){
  auto c=camera();c.d=(cv::Mat_<double>(5,1)<<-.08,.15,-.001,.001,.03);
  for(double angle:{0.,1.,3.}){
    const auto result=solve(c,project(c,{.3,.5,angle},{.2,.1,1.5}));
    ASSERT_TRUE(result.valid)<<result.reason;EXPECT_NEAR(result.translation[2],1.5,1e-4);
  }
}
TEST(PnP, InvalidGeometryAndDepthAreRejected){
  auto c=camera();auto points=project(c,{.2,.4,0},{0,0,2});
  std::swap(points[1],points[3]);EXPECT_FALSE(solve(c,points).valid);
  points[0].x=std::numeric_limits<float>::quiet_NaN();EXPECT_FALSE(solve(c,points).valid);
  Parameters p;p.max_depth_m=1;EXPECT_FALSE(solve(c,project(c,{.2,.4,0},{0,0,2}),p).valid);
}
TEST(PnP, NearlyTiedDistinctIppeSolutionsAreNotSilentlySelected){
  auto c=camera();Parameters p;p.ambiguity_gap_px=100;
  const auto result=solve(c,project(c,{.1,.2,0},{0,0,3}),p);
  EXPECT_FALSE(result.valid);EXPECT_TRUE(result.ambiguous);EXPECT_EQ(result.reason,"ambiguous_ippe");
}
TEST(PnP, ReprojectionErrorGuardAndParameterValidation){
  auto c=camera();auto points=project(c,{.2,.4,0},{0,0,2});points[0].x+=10;
  Parameters p;p.max_error_px=.001;EXPECT_FALSE(solve(c,points,p).valid);
  p.width_m=0;EXPECT_THROW(p.validate(),std::invalid_argument);
}
TEST(Digit, InstalledSourceTemplatePositiveAndNegativeSamples){
  const auto templ=cv::imread(TEMPLATE_PATH,cv::IMREAD_GRAYSCALE);ASSERT_FALSE(templ.empty());
  cv::Mat patch=cv::Mat::zeros(96,96,CV_8U),scaled;
  // Exact scale used by the multi-scale search: height 56 and squeeze .6.
  const int width=cvRound(56.*templ.cols/templ.rows*.6);
  cv::resize(templ,scaled,{width,56});scaled.copyTo(patch(cv::Rect((96-width)/2,20,width,56)));
  cv::Mat gray=cv::Mat::zeros(160,160,CV_8U);patch.copyTo(gray(cv::Rect(20,20,96,96)));
  Corners points{{{20,20},{20,115},{115,115},{115,20}}};
  EXPECT_GT(digit_score(gray,points,templ),.95);
  EXPECT_LT(digit_score(cv::Mat::zeros(160,160,CV_8U),points,templ),.65);
  cv::RNG rng(123);rng.fill(gray,cv::RNG::UNIFORM,0,255);
  EXPECT_LT(digit_score(gray,points,templ),.65);
  EXPECT_LT(digit_score(gray,points,cv::Mat()),0);
}
TEST(Calibration, ExplicitFileValidation){
  EXPECT_THROW(Calibration::load(""),std::exception);
  EXPECT_THROW(Calibration::load("/does/not/exist.yaml"),std::exception);
  const auto unique=std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path=std::filesystem::temp_directory_path()/
    ("srm_locator_calibration_"+std::to_string(unique)+".yaml");
  const std::string valid="image_width: 1440\nimage_height: 1080\ndistortion_model: plumb_bob\ncamera_matrix:\n  rows: 3\n  cols: 3\n  data: [1800, 0, 720, 0, 1800, 540, 0, 0, 1]\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [0, 0, 0, 0, 0]\n";
  {std::ofstream f(path);f<<valid;}
  EXPECT_EQ(Calibration::load(path.string()).size,cv::Size(1440,1080));
  auto bad=valid;bad.replace(bad.find("1800"),4,"-100");
  {std::ofstream f(path);f<<bad;}
  EXPECT_THROW(Calibration::load(path.string()),std::exception);
  {std::ofstream f(path);f<<"image_width: 0\n";}
  EXPECT_THROW(Calibration::load(path.string()),std::exception);
  std::filesystem::remove(path);
}

TEST(Digit, PerspectiveNormalizationUsesLtLbRbRt){
  const auto templ=cv::imread(TEMPLATE_PATH,cv::IMREAD_GRAYSCALE);
  cv::Mat patch=cv::Mat::zeros(96,96,CV_8U),scaled;
  const int width=cvRound(56.*templ.cols/templ.rows*.6);
  cv::resize(templ,scaled,{width,56});scaled.copyTo(patch(cv::Rect((96-width)/2,20,width,56)));
  const std::vector<cv::Point2f> canonical{{0,0},{0,95},{95,95},{95,0}};
  const Corners corners{{{30,40},{20,170},{165,180},{185,30}}};
  cv::Mat projected;
  cv::warpPerspective(patch,projected,cv::getPerspectiveTransform(canonical,
    std::vector<cv::Point2f>(corners.begin(),corners.end())),{220,220});
  EXPECT_GT(digit_score(projected,corners,templ),.65);
}

#include "srm_armor_locator/digit_debug.hpp"
#include <yaml-cpp/yaml.h>
TEST(DigitDebug, TracePreservesScoresAndExactMatchingBuffers){
  const auto templ=cv::imread(TEMPLATE_PATH,cv::IMREAD_GRAYSCALE);
  cv::Mat patch=cv::Mat::zeros(96,96,CV_8U),scaled;
  const int width=cvRound(56.*templ.cols/templ.rows*.6);
  cv::resize(templ,scaled,{width,56});scaled.copyTo(patch(cv::Rect((96-width)/2,20,width,56)));
  cv::Mat gray=cv::Mat::zeros(160,160,CV_8U);patch.copyTo(gray(cv::Rect(20,20,96,96)));
  Corners points{{{20,20},{20,115},{115,115},{115,20}}};
  DigitTrace trace;const auto score=digit_score(gray,points,templ,&trace);
  EXPECT_DOUBLE_EQ(score,digit_score(gray,points,templ));
  EXPECT_EQ(trace.perspective.size(),cv::Size(96,96));
  EXPECT_EQ(trace.roi.size(),cv::Size(68,96));
  EXPECT_EQ(cv::norm(trace.roi,trace.perspective(cv::Rect(14,0,68,96)),cv::NORM_INF),0);
  EXPECT_EQ(cv::norm(trace.best_patch,trace.roi(trace.best_rect),cv::NORM_INF),0);
  cv::Mat scores;cv::matchTemplate(trace.best_patch,trace.best_template,scores,cv::TM_CCOEFF_NORMED);
  EXPECT_NEAR(scores.at<float>(0,0),score,1e-5);
}
TEST(DigitDebug, DisabledBudgetIntervalMetadataAndUniqueSessions){
  const auto base=std::filesystem::temp_directory_path()/
    ("digit_debug_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  DigitDebugWriter disabled(false,base.string(),2,0);
  EXPECT_FALSE(std::filesystem::exists(base));
  EXPECT_FALSE(disabled.wants_sample(DigitDebugWriter::Clock::now()));
  DigitDebugWriter writer(true,base.string(),2,1);
  const auto now=DigitDebugWriter::Clock::now();
  const auto templ=cv::imread(TEMPLATE_PATH,cv::IMREAD_GRAYSCALE);
  cv::Mat gray=cv::Mat::zeros(160,160,CV_8U);
  Corners points{{{20,20},{20,115},{115,115},{115,20}}};DigitTrace trace;
  const auto score=digit_score(gray,points,templ,&trace);
  EXPECT_EQ(trace.reason,"low_roi_contrast");
  std_msgs::msg::Header h;h.stamp.sec=123;h.stamp.nanosec=456;h.frame_id="camera/frame";
  EXPECT_TRUE(writer.save(gray,points,templ,trace,h,3,score,.65,TEMPLATE_PATH,now));
  EXPECT_FALSE(writer.save(gray,points,templ,trace,h,3,score,.65,TEMPLATE_PATH,now));
  EXPECT_TRUE(writer.save(gray,points,templ,trace,h,3,score,.65,TEMPLATE_PATH,now+std::chrono::seconds(2)));
  EXPECT_FALSE(writer.wants_sample(now+std::chrono::seconds(5)));
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(writer.directory()),std::filesystem::directory_iterator{}),10);
  const auto meta=YAML::LoadFile((writer.directory()/"1_123_456_target_3_metadata.yaml").string());
  EXPECT_EQ(meta["frame_id"].as<std::string>(),h.frame_id);
  EXPECT_EQ(meta["stamp_nanosec"].as<int>(),456);EXPECT_EQ(meta["input_index"].as<int>(),3);
  EXPECT_DOUBLE_EQ(meta["threshold"].as<double>(),.65);EXPECT_FALSE(meta["verified"].as<bool>());
  EXPECT_EQ(cv::imread((writer.directory()/"1_123_456_target_3_roi.png").string(),0).size(),cv::Size(68,96));
  DigitDebugWriter another(true,base.string(),2,0);EXPECT_NE(writer.directory(),another.directory());
  std::filesystem::remove_all(base);
}
TEST(DigitDebug, WriteFailureStopsFurtherAttempts){
  const auto base=std::filesystem::temp_directory_path()/
    ("digit_debug_failure_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  DigitDebugWriter writer(true,base.string(),2,0);
  std::filesystem::remove_all(writer.directory());
  const auto now=DigitDebugWriter::Clock::now();
  std_msgs::msg::Header header;
  EXPECT_THROW(writer.save(cv::Mat::zeros(10,10,CV_8U),{},cv::Mat(),{},header,0,0,.65,"",now),std::exception);
  EXPECT_EQ(writer.attempts(),1);EXPECT_FALSE(writer.wants_sample(now));
  std::filesystem::remove_all(base);
}

#include "srm_armor_locator/ippe_debug.hpp"
#include <map>
TEST(IppeDebug, TraceDoesNotChangeSelectionAndReportsGeometry){
  const auto c=camera();
  for(const auto & pose:std::vector<std::pair<cv::Vec3d,cv::Vec3d>>{
    {{.1,.2,0},{0,0,4.7}},{{.3,.5,.1},{.3,.1,2}},{{0,0,0},{0,0,4.7}}}) {
    const auto points=project(c,pose.first,pose.second);
    const auto plain=solve(c,points);IppeTrace trace;
    const auto observed=solve(c,points,{},&trace);
    EXPECT_EQ(observed.valid,plain.valid);EXPECT_EQ(observed.ambiguous,plain.ambiguous);
    EXPECT_EQ(observed.reason,plain.reason);EXPECT_DOUBLE_EQ(observed.error,plain.error);
    EXPECT_EQ(cv::norm(observed.translation-plain.translation),0);
    EXPECT_EQ(cv::norm(observed.rotation-plain.rotation),0);
    ASSERT_TRUE(trace.pair_available);EXPECT_EQ(trace.valid_candidates,2u);
    EXPECT_LE(trace.candidates[0].error_px,trace.candidates[1].error_px);
    for(const auto & candidate:trace.candidates)
      EXPECT_NEAR(candidate.distance_mm,cv::norm(candidate.translation)*1000,1e-9);
    EXPECT_NEAR(trace.translation_separation_mm,cv::norm(trace.translation_delta_mm),1e-9);
    EXPECT_NEAR(trace.signed_distance_difference_mm,trace.candidates[1].distance_mm-trace.candidates[0].distance_mm,1e-9);
    EXPECT_EQ(observed.ambiguous,trace.error_gap_close&&(trace.rotation_exceeds||trace.translation_exceeds));
    EXPECT_GE(trace.rotation_difference_deg,0);EXPECT_LE(trace.rotation_difference_deg,180);
  }
}
TEST(IppeDebug, MissingPairResetsTraceAndRemainsUnavailable){
  auto c=camera();IppeTrace trace;trace.pair_available=true;
  Corners invalid{};const auto result=solve(c,invalid,{},&trace);
  EXPECT_FALSE(result.valid);EXPECT_FALSE(trace.pair_available);EXPECT_EQ(trace.valid_candidates,0u);
  Parameters p;p.max_depth_m=1;
  solve(c,project(c,{.2,.4,0},{0,0,4.7}),p,&trace);
  EXPECT_FALSE(trace.pair_available);EXPECT_EQ(trace.valid_candidates,0u);
  // Tight error guard leaves only the best synthetic solution.
  p=Parameters{};p.max_error_px=.001;
  solve(c,project(c,{.3,.5,0},{.1,0,2}),p,&trace);
  EXPECT_EQ(trace.valid_candidates,1u);EXPECT_FALSE(trace.pair_available);
}
namespace {
std::map<std::string,std::string> values(const diagnostic_msgs::msg::DiagnosticStatus & s){
  std::map<std::string,std::string> out;for(const auto & v:s.values)out[v.key]=v.value;return out;
}
}
TEST(IppeDebug, BoundedSamplesRateAndStatistics){
  const auto start=IppeDebug::Clock::now();Parameters p;IppeDebug debug(1,2,p,start);
  IppeTrace trace;trace.valid_candidates=2;trace.pair_available=true;
  trace.error_gap_px=.05;trace.translation_separation_mm=2;
  trace.signed_distance_difference_mm=-1;trace.rotation_difference_deg=20;
  trace.error_gap_close=true;trace.rotation_exceeds=true;
  trace.candidates[0].translation={.1,.2,4.7};trace.candidates[0].distance_mm=4705;
  PoseResult result;result.ambiguous=true;result.reason="ambiguous_ippe";
  std_msgs::msg::Header header;header.stamp.sec=123;header.stamp.nanosec=456;header.frame_id="source_camera";
  for(size_t i=0;i<5;++i)debug.observe(header,i,trace,result);
  EXPECT_FALSE(debug.flush(start+std::chrono::milliseconds(999)));
  auto report=debug.flush(start+std::chrono::seconds(1));ASSERT_TRUE(report);
  EXPECT_EQ(report->header,header);ASSERT_EQ(report->status.size(),3u);
  auto stats=values(report->status[0]);EXPECT_EQ(stats["window_attempts"],"5");
  EXPECT_EQ(stats["total_ambiguous_ippe"],"5");EXPECT_EQ(stats["window_rotation_only"],"5");
  EXPECT_EQ(stats["window_samples_omitted"],"3");
  EXPECT_DOUBLE_EQ(std::stod(stats["window_absolute_distance_difference_mm_mean"]),1);
  auto sample=values(report->status[1]);EXPECT_EQ(sample["frame_id"],header.frame_id);
  EXPECT_EQ(sample["stamp_nanosec"],"456");EXPECT_EQ(sample["input_index"],"0");
  EXPECT_EQ(sample["trigger"],"rotation");EXPECT_EQ(sample["translation_triggered"],"false");
  EXPECT_DOUBLE_EQ(std::stod(sample["candidate_0_t_z_mm"]),4700);
  EXPECT_FALSE(debug.flush(start+std::chrono::seconds(2))); // no idle spam
  trace.translation_exceeds=true;debug.observe(header,0,trace,result);
  auto next=debug.flush(start+std::chrono::seconds(2));ASSERT_TRUE(next);
  stats=values(next->status[0]);EXPECT_EQ(stats["window_attempts"],"1");
  EXPECT_EQ(stats["total_attempts"],"6");EXPECT_EQ(stats["window_both"],"1");
}
TEST(IppeDebug, TranslationOnlyNonTriggeredCountsAndStatsOnlyMode){
  const auto start=IppeDebug::Clock::now();IppeDebug debug(.1,0,{},start);
  IppeTrace trace;trace.valid_candidates=2;trace.pair_available=true;trace.error_gap_close=true;
  trace.translation_exceeds=true;PoseResult r;r.ambiguous=true;
  std_msgs::msg::Header h;debug.observe(h,0,trace,r);
  trace.error_gap_close=false;r.ambiguous=false;r.valid=true;debug.observe(h,1,trace,r);
  trace=IppeTrace{};trace.valid_candidates=1;debug.observe(h,2,trace,r);
  auto report=debug.flush(start+std::chrono::seconds(1));ASSERT_TRUE(report);ASSERT_EQ(report->status.size(),1u);
  const auto stats=values(report->status[0]);EXPECT_EQ(stats.at("window_translation_only"),"1");
  EXPECT_EQ(stats.at("window_accepted"),"2");EXPECT_EQ(stats.at("window_one_valid_candidate"),"1");
  EXPECT_EQ(stats.at("window_samples_omitted"),"2");
  EXPECT_THROW(IppeDebug invalid(0,2,{}),std::invalid_argument);
  EXPECT_THROW(IppeDebug invalid(1,65,{}),std::invalid_argument);
}

TEST(PositionPolicy, DefaultStrictResultsUnchanged){
  auto c=camera();
  for(auto r:std::vector<cv::Vec3d>{{.1,.2,0},{.3,.5,.1}}){
    auto corners=project(c,r,{0,0,4.7});auto strict=solve(c,corners);
    auto out=solve_location(c,corners);
    EXPECT_EQ(out.strict_pose.valid,strict.valid);EXPECT_EQ(out.strict_pose.ambiguous,strict.ambiguous);
    EXPECT_EQ(out.strict_pose.reason,strict.reason);EXPECT_DOUBLE_EQ(out.strict_pose.error,strict.error);
    EXPECT_EQ(cv::norm(out.strict_pose.translation-strict.translation),0);
    EXPECT_EQ(cv::norm(out.strict_pose.rotation-strict.rotation),0);
    EXPECT_EQ(out.position_valid,strict.valid);EXPECT_EQ(out.orientation_valid,strict.valid);
  }
}
TEST(PositionPolicy, RotationOnlyKeepsPositionWithoutAcceptingOrientation){
  auto c=camera();auto corners=project(c,{.1,.2,0},{0,0,4.7});
  PositionPolicy policy;policy.allow_rotation_ambiguous_position=true;IppeTrace trace;
  auto out=solve_location(c,corners,{},policy,&trace);
  ASSERT_TRUE(out.strict_pose.ambiguous);ASSERT_TRUE(trace.pair_available);
  EXPECT_TRUE(trace.rotation_exceeds);EXPECT_FALSE(trace.translation_exceeds);
  EXPECT_TRUE(out.position_valid);EXPECT_FALSE(out.orientation_valid);EXPECT_FALSE(out.strict_pose.valid);
  EXPECT_EQ(out.strict_pose.reason,"ambiguous_ippe");
  EXPECT_EQ(out.position_reason,"position_retained_rotation_ambiguity");
  EXPECT_EQ(cv::norm(out.position-trace.candidates[0].translation),0);
  EXPECT_EQ(cv::norm(out.strict_pose.translation),0);
  auto implicit=solve_location(c,corners,{},policy); // Must work even with IPPE debug disabled.
  EXPECT_TRUE(implicit.position_valid);EXPECT_EQ(cv::norm(implicit.position-out.position),0);
}
TEST(PositionPolicy, OwnTranslationLimitAndOriginalTranslationAmbiguityStillReject){
  auto c=camera();auto corners=project(c,{.1,.2,0},{.1,0,4.7});IppeTrace trace;
  solve(c,corners,{},&trace);ASSERT_TRUE(trace.pair_available);
  const double separation=cv::norm(trace.candidates[0].translation-trace.candidates[1].translation);
  ASSERT_GT(separation,0);
  PositionPolicy policy;policy.allow_rotation_ambiguous_position=true;
  policy.max_translation_difference_m=separation; // Inclusive upper boundary.
  auto out=solve_location(c,corners,{},policy);EXPECT_TRUE(out.position_valid);
  policy.max_translation_difference_m=separation*.5;
  out=solve_location(c,corners,{},policy);EXPECT_FALSE(out.position_valid);
  EXPECT_EQ(out.position_reason,"position_translation_limit_exceeded");
  policy.max_translation_difference_m=1;Parameters p;p.ambiguity_translation_m=separation*.5;
  out=solve_location(c,corners,p,policy);EXPECT_FALSE(out.position_valid);EXPECT_FALSE(out.orientation_valid);
  EXPECT_TRUE(out.strict_pose.ambiguous);EXPECT_EQ(out.position_reason,"translation_ambiguity_rejected");
  p.ambiguity_angle_deg=180; // Translation-only ambiguity is also rejected.
  out=solve_location(c,corners,p,policy);EXPECT_FALSE(out.position_valid);
  EXPECT_TRUE(out.strict_pose.ambiguous);EXPECT_EQ(out.position_reason,"translation_ambiguity_rejected");
}
TEST(PositionPolicy, OtherValidityGuardsAndPolicyValidation){
  auto c=camera();PositionPolicy policy;policy.allow_rotation_ambiguous_position=true;
  auto full=solve_location(c,project(c,{.3,.5,.1},{.3,.1,2}),{},policy);
  EXPECT_TRUE(full.strict_pose.valid);EXPECT_TRUE(full.position_valid);EXPECT_TRUE(full.orientation_valid);
  auto points=project(c,{.1,.2,0},{0,0,4.7});Parameters p;p.max_depth_m=1;
  auto out=solve_location(c,points,p,policy);EXPECT_FALSE(out.position_valid);EXPECT_FALSE(out.orientation_valid);
  auto distorted=points;distorted[0].x+=10;p=Parameters{};p.max_error_px=.001;
  out=solve_location(c,distorted,p,policy);EXPECT_FALSE(out.position_valid);
  points[0].x=-1;out=solve_location(c,points,{},policy);EXPECT_FALSE(out.position_valid);
  policy.max_translation_difference_m=0;EXPECT_THROW(policy.validate(),std::invalid_argument);
  policy.max_translation_difference_m=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(policy.validate(),std::invalid_argument);
}
