#include "srm_armor_locator/locator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace srm_armor_locator {
Calibration Calibration::load(const std::string & path) {
  if (path.empty()) throw std::invalid_argument("calibration_file is required; no default intrinsics");
  const auto y = YAML::LoadFile(path);
  Calibration c;
  c.size = {y["image_width"].as<int>(), y["image_height"].as<int>()};
  const auto model = y["distortion_model"].as<std::string>();
  auto k = y["camera_matrix"]["data"].as<std::vector<double>>();
  auto d = y["distortion_coefficients"]["data"].as<std::vector<double>>();
  if (c.size.width <= 0 || c.size.height <= 0 || k.size() != 9 ||
    y["camera_matrix"]["rows"].as<int>() != 3 ||
    y["camera_matrix"]["cols"].as<int>() != 3 ||
    y["distortion_coefficients"]["rows"].as<int>() != 1 ||
    y["distortion_coefficients"]["cols"].as<size_t>() != d.size() ||
    !((model == "plumb_bob" && (d.size() == 4 || d.size() == 5)) ||
    (model == "rational_polynomial" && d.size() == 8)))
    throw std::invalid_argument("Invalid calibration dimensions/distortion model");
  for (double v : k) if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite intrinsics");
  for (double v : d) if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite distortion");
  if (k[0] <= 0 || k[4] <= 0 || std::abs(k[1]) > 1e-9 || std::abs(k[3]) > 1e-9 ||
    std::abs(k[6]) > 1e-9 || std::abs(k[7]) > 1e-9 || std::abs(k[8]-1) > 1e-9 ||
    k[2] < 0 || k[2] >= c.size.width || k[5] < 0 || k[5] >= c.size.height)
    throw std::invalid_argument("Unsupported/invalid camera matrix");
  c.k = cv::Mat(k).reshape(1,3).clone(); c.d = cv::Mat(d).clone();
  return c;
}
void Parameters::validate() const {
  for (double v : {width_m,height_m,min_depth_m,max_depth_m,max_error_px,
    ambiguity_gap_px,ambiguity_translation_m,ambiguity_angle_deg})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("PnP parameters must be positive/finite");
  if (max_depth_m <= min_depth_m || ambiguity_angle_deg > 180)
    throw std::invalid_argument("Invalid depth or ambiguity range");
}
std::vector<cv::Point3d> object_points(const Parameters & p) {
  return {{-p.width_m/2,-p.height_m/2,0},{-p.width_m/2,p.height_m/2,0},
    {p.width_m/2,p.height_m/2,0},{p.width_m/2,-p.height_m/2,0}};
}
double digit_score(const cv::Mat & gray, const Corners & c, const cv::Mat & templ) {
  if (gray.empty() || gray.type()!=CV_8UC1 || templ.empty() || templ.type()!=CV_8UC1 ||
    !srm_armor_refiner::valid_corners(c,gray.size(),{})) return -1;
  const std::vector<cv::Point2f> src(c.begin(),c.end());
  const std::vector<cv::Point2f> dst{{0,0},{0,95},{95,95},{95,0}};
  cv::Mat normalized;
  cv::warpPerspective(gray,normalized,cv::getPerspectiveTransform(src,dst),{96,96});
  const cv::Mat roi = normalized(cv::Rect(14,0,68,96));
  cv::Scalar mean, deviation; cv::meanStdDev(roi,mean,deviation);
  if (deviation[0] < 2) return 0;
  double best=0;
  for (int h : {44,56,68,80,90}) for (double squeeze : {0.6,0.8,1.0}) {
    const int w = cvRound(h * static_cast<double>(templ.cols)/templ.rows * squeeze);
    if (w<8 || w>roi.cols) continue;
    cv::Mat candidate, scores; cv::resize(templ,candidate,{w,h});
    cv::meanStdDev(candidate,mean,deviation); if (deviation[0]<2) continue;
    cv::matchTemplate(roi,candidate,scores,cv::TM_CCOEFF_NORMED);
    for (int y=0;y<scores.rows;++y) for (int x=0;x<scores.cols;++x) {
      const float score=scores.at<float>(y,x);
      if (std::abs(x+w/2.0-roi.cols/2.0)<=9 && std::abs(y+h/2.0-48)<=18 &&
        std::isfinite(score)) best=std::max(best,static_cast<double>(score));
    }
  }
  return best;
}
PoseResult solve(const Calibration & c, const Corners & corners, const Parameters & p) {
  p.validate();
  if (!srm_armor_refiner::valid_corners(corners,c.size,{})) return {false,false,"invalid_geometry"};
  std::vector<cv::Mat> rotations, translations;
  const auto model=object_points(p);
  std::vector<PoseResult> candidates;
  try {
    cv::solvePnPGeneric(model,std::vector<cv::Point2f>(corners.begin(),corners.end()),
      c.k,c.d,rotations,translations,false,cv::SOLVEPNP_IPPE);
    for (size_t i=0;i<rotations.size();++i) {
      cv::Vec3d r=rotations[i], t=translations[i];
      if (!cv::checkRange(rotations[i]) || !cv::checkRange(translations[i]) ||
        t[2]<p.min_depth_m || t[2]>p.max_depth_m) continue;
      cv::Matx33d rotation; cv::Rodrigues(r,rotation);
      bool positive=true;
      for (const auto & point:model) if ((rotation*cv::Vec3d(point.x,point.y,0)+t)[2]<=0) positive=false;
      if (!positive) continue;
      std::vector<cv::Point2d> projected; cv::projectPoints(model,r,t,c.k,c.d,projected);
      double squared=0;
      for (size_t j=0;j<4;++j) {const auto delta=projected[j]-cv::Point2d(corners[j]); squared+=delta.dot(delta);}
      double error=std::sqrt(squared/4);
      if (std::isfinite(error) && error<=p.max_error_px) candidates.push_back({true,false,"ok",r,t,error});
    }
    if (candidates.empty()) return {false,false,"no_valid_pnp_solution"};
    std::sort(candidates.begin(),candidates.end(),[](const auto & a,const auto & b){return a.error<b.error;});
    if (candidates.size()>1 && candidates[1].error-candidates[0].error<=p.ambiguity_gap_px) {
      cv::Matx33d a,b; cv::Rodrigues(candidates[0].rotation,a); cv::Rodrigues(candidates[1].rotation,b);
      cv::Vec3d delta; cv::Rodrigues(a*b.t(),delta);
      if (cv::norm(delta)*180/CV_PI>p.ambiguity_angle_deg ||
        cv::norm(candidates[0].translation-candidates[1].translation)>p.ambiguity_translation_m)
        return {false,true,"ambiguous_ippe",{}, {},candidates[0].error};
    }
    return candidates.front();
  } catch (const cv::Exception &) {return {false,false,"pnp_opencv_error"};}
}
}
