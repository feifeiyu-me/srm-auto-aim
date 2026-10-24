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
