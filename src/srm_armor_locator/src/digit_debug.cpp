#include "srm_armor_locator/digit_debug.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
namespace srm_armor_locator {
DigitDebugWriter::DigitDebugWriter(bool enabled,const std::string & directory,int limit,double interval)
:enabled_(enabled),limit_(limit),interval_(interval) {
  if(!enabled_) return;
  if(directory.empty() || limit<1 || limit>1000 || !std::isfinite(interval) || interval<0)
    throw std::invalid_argument("Digit debug requires output_dir, max_samples in [1,1000], interval >= 0");
  const auto base=std::filesystem::absolute(directory);
  std::filesystem::create_directories(base);
  for(int i=0;i<100;++i) {
    auto path=base/("session_"+std::to_string(Clock::now().time_since_epoch().count())+"_"+std::to_string(i));
    if(std::filesystem::create_directory(path)) {session_=path;return;}
  }
  throw std::runtime_error("Cannot create unique digit-debug session");
}
bool DigitDebugWriter::wants_sample(Clock::time_point now) const {
  return enabled_ && !failed_ && attempts_<limit_ &&
    (attempts_==0 || std::chrono::duration<double>(now-last_).count()>=interval_);
}
bool DigitDebugWriter::save(const cv::Mat & gray,const Corners & corners,const cv::Mat & templ,
  const DigitTrace & trace,const std_msgs::msg::Header & header,size_t index,
  double score,double threshold,const std::string & template_file,Clock::time_point now) {
  if(!wants_sample(now)) return false;
  ++attempts_; last_=now;
  try {
    const auto prefix=session_/(std::to_string(attempts_)+"_"+std::to_string(header.stamp.sec)+
      "_"+std::to_string(header.stamp.nanosec)+"_target_"+std::to_string(index));
    std::ostringstream label;
    label<<"stamp="<<header.stamp.sec<<'.'<<std::setw(9)<<std::setfill('0')<<header.stamp.nanosec<<" frame="<<header.frame_id
         <<" target="<<index<<" score="<<score<<" threshold="<<threshold;
    auto write=[&](const std::string & suffix,const cv::Mat & image) {
      if(image.empty() || !cv::imwrite(prefix.string()+suffix,image))
        throw std::runtime_error("Digit debug PNG write failed");
    };
    cv::Mat source;cv::cvtColor(gray,source,cv::COLOR_GRAY2BGR);
    const char * names[]={"LT","LB","RB","RT"};
    for(size_t i=0;i<4;++i) {
      cv::circle(source,corners[i],3,{0,255,0},-1);
      cv::putText(source,names[i],corners[i],cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,0},1);
      cv::line(source,corners[i],corners[(i+1)%4],{0,255,0},1);
    }
    cv::putText(source,label.str(),{8,20},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
    write("_source.png",source);
    // These are exact, unannotated buffers used by matching, not display resizes.
    write("_perspective.png",trace.perspective);write("_roi.png",trace.roi);
    cv::Mat comparison(250,800,CV_8UC3,cv::Scalar(25,25,25));
    cv::putText(comparison,label.str(),{8,18},cv::FONT_HERSHEY_SIMPLEX,.4,{255,255,255},1);
    auto panel=[&](const cv::Mat & raw,int x,const std::string & name) {
      cv::putText(comparison,name,{x,45},cv::FONT_HERSHEY_SIMPLEX,.45,{255,255,255},1);
      if(raw.empty()) {cv::putText(comparison,"N/A",{x,80},cv::FONT_HERSHEY_SIMPLEX,.5,{0,165,255},1);return;}
      cv::Mat display,color;cv::resize(raw,display,{140,180},0,0,cv::INTER_NEAREST);
      cv::cvtColor(display,color,cv::COLOR_GRAY2BGR);color.copyTo(comparison(cv::Rect(x,55,140,180)));
    };
    cv::Mat marked=trace.roi.clone();
    if(!trace.best_template.empty()) cv::rectangle(marked,trace.best_rect,255,1);
    panel(marked,8,"ROI + best box");panel(templ,170,"Original template");
    panel(trace.best_patch,335,"Actual patch");panel(trace.best_template,500,"Resized template");
    cv::putText(comparison,trace.reason,{650,90},cv::FONT_HERSHEY_SIMPLEX,.35,{0,165,255},1);
    write("_comparison.png",comparison);
    YAML::Emitter meta;
    meta<<YAML::BeginMap<<YAML::Key<<"stamp_sec"<<YAML::Value<<header.stamp.sec
      <<YAML::Key<<"stamp_nanosec"<<YAML::Value<<header.stamp.nanosec
      <<YAML::Key<<"frame_id"<<YAML::Value<<header.frame_id
      <<YAML::Key<<"input_index"<<YAML::Value<<index
      <<YAML::Key<<"score"<<YAML::Value<<score<<YAML::Key<<"threshold"<<YAML::Value<<threshold
      <<YAML::Key<<"verified"<<YAML::Value<<(score>=threshold)
      <<YAML::Key<<"trace_reason"<<YAML::Value<<trace.reason
      <<YAML::Key<<"roi_stddev"<<YAML::Value<<trace.roi_stddev
      <<YAML::Key<<"template_file"<<YAML::Value<<template_file
      <<YAML::Key<<"template_width"<<YAML::Value<<templ.cols
      <<YAML::Key<<"template_height"<<YAML::Value<<templ.rows
      <<YAML::Key<<"best_rect_xywh"<<YAML::Value<<YAML::Flow<<YAML::BeginSeq
      <<trace.best_rect.x<<trace.best_rect.y<<trace.best_rect.width<<trace.best_rect.height<<YAML::EndSeq
      <<YAML::Key<<"best_squeeze"<<YAML::Value<<trace.best_squeeze
      <<YAML::Key<<"source_width"<<YAML::Value<<gray.cols
      <<YAML::Key<<"source_height"<<YAML::Value<<gray.rows
      <<YAML::Key<<"corners_lt_lb_rb_rt"<<YAML::Value<<YAML::BeginSeq;
    for(const auto & p:corners) meta<<YAML::Flow<<YAML::BeginSeq<<p.x<<p.y<<YAML::EndSeq;
    meta<<YAML::EndSeq<<YAML::EndMap;
    std::ofstream file(prefix.string()+"_metadata.yaml");file<<meta.c_str();file.close();
    if(!file) throw std::runtime_error("Digit debug metadata write failed");
    return true;
  } catch(...) {failed_=true;throw;}
}
}
