#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <srm_aim_interfaces/msg/armor_detection_array.hpp>

namespace srm_armor_refiner
{
class FrameMatcher
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Image = sensor_msgs::msg::Image;
  using Detections = srm_aim_interfaces::msg::ArmorDetectionArray;
  using Key = std::tuple<int32_t, uint32_t, std::string>;
  struct Pair {Image::ConstSharedPtr image; Detections::ConstSharedPtr detections;};
  struct Event {std_msgs::msg::Header header; std::string reason;};

  FrameMatcher(size_t image_limit, size_t detection_limit, size_t byte_limit, double ttl)
  : image_limit_(image_limit), detection_limit_(detection_limit), byte_limit_(byte_limit), ttl_(ttl)
  {
    if (!image_limit || !detection_limit || !byte_limit || !std::isfinite(ttl) || ttl <= 0) {
      throw std::invalid_argument("Cache limits/TTL must be finite and positive");
    }
  }

  std::optional<Pair> add_image(Image::ConstSharedPtr image, Time now)
  {
    prune(now);
    const auto key = identity(image->header);
    if (!valid_stamp(image->header) || image->width == 0 || image->height == 0 ||
      image->data.empty() || image->data.size() > byte_limit_)
    {
      event(image->header, "invalid_or_oversized_image"); return {};
    }
    if (blocked_.count(key)) {event(image->header, "duplicate_header"); return {};}
    if (images_.count(key)) {
      erase_image(images_.find(key));
      detections_.erase(key);
      block(key, now);
      event(image->header, "ambiguous_duplicate_image"); return {};
    }
    bytes_ += image->data.size();
    images_.emplace(key, Entry<Image>{image, now, sequence_++});
    while (images_.size() > image_limit_ || bytes_ > byte_limit_) {
      auto oldest = oldest_entry(images_);
      event(oldest->second.message->header, "image_cache_evicted");
      erase_image(oldest);
    }
    return match(key, now);
  }

  std::optional<Pair> add_detection(Detections::ConstSharedPtr detection, Time now)
  {
    prune(now);
    const auto key = identity(detection->header);
    if (!valid_stamp(detection->header)) {
      event(detection->header, "invalid_stamp"); return {};
    }
    if (blocked_.count(key)) {event(detection->header, "duplicate_header"); return {};}
    detections_.erase(key);
    detections_.emplace(key, Entry<Detections>{detection, now, sequence_++});
    while (detections_.size() > detection_limit_) {
      auto oldest = oldest_entry(detections_);
      event(oldest->second.message->header, "detection_cache_evicted");
      detections_.erase(oldest);
    }
    return match(key, now);
  }

  void prune(Time now)
  {
    for (auto it = images_.begin(); it != images_.end(); ) {
      if (age(now, it->second.arrived) >= ttl_) {
        event(it->second.message->header, "unmatched_image_expired");
        auto expired = it++; erase_image(expired);
      } else {++it;}
    }
    for (auto it = detections_.begin(); it != detections_.end(); ) {
      if (age(now, it->second.arrived) >= ttl_) {
        event(it->second.message->header, "unmatched_detection_expired");
        it = detections_.erase(it);
      } else {++it;}
    }
    for (auto it = blocked_.begin(); it != blocked_.end(); ) {
      if (age(now, it->second) >= ttl_) {it = blocked_.erase(it);} else {++it;}
    }
  }

  std::vector<Event> take_events()
  {
    std::vector<Event> events(events_.begin(), events_.end());
    events_.clear(); return events;
  }
  size_t image_count() const {return images_.size();}
  size_t detection_count() const {return detections_.size();}
  size_t image_bytes() const {return bytes_;}

private:
  template<typename M> struct Entry
  {typename M::ConstSharedPtr message; Time arrived; uint64_t sequence;};
  using ImageMap = std::map<Key, Entry<Image>>;
  size_t image_limit_, detection_limit_, byte_limit_, bytes_{0};
  double ttl_;
  uint64_t sequence_{0};
  ImageMap images_;
  std::map<Key, Entry<Detections>> detections_;
  std::map<Key, Time> blocked_;
  std::deque<Event> events_;

  static Key identity(const std_msgs::msg::Header & h)
  {return {h.stamp.sec, h.stamp.nanosec, h.frame_id};}
  static bool valid_stamp(const std_msgs::msg::Header & h) {return h.stamp.nanosec < 1000000000;}
  static double age(Time now, Time arrived)
  {return std::chrono::duration<double>(now - arrived).count();}
  template<typename Map> static typename Map::iterator oldest_entry(Map & map)
  {
    return std::min_element(map.begin(), map.end(), [](const auto & a, const auto & b) {
      return a.second.sequence < b.second.sequence;
    });
  }
  void erase_image(ImageMap::iterator it)
  {bytes_ -= it->second.message->data.size(); images_.erase(it);}
  void event(const std_msgs::msg::Header & h, const std::string & reason)
  {
    if (events_.size() == 100) {events_.pop_front();}
    events_.push_back({h, reason});
  }
  void block(const Key & key, Time now)
  {
    blocked_[key] = now;
    while (blocked_.size() > image_limit_ + detection_limit_) {
      auto oldest = std::min_element(blocked_.begin(), blocked_.end(),
        [](const auto & a, const auto & b) {return a.second < b.second;});
      blocked_.erase(oldest);
    }
  }
  std::optional<Pair> match(const Key & key, Time now)
  {
    auto image = images_.find(key);
    auto detection = detections_.find(key);
    if (image == images_.end() || detection == detections_.end()) {return {};}
    Pair pair{image->second.message, detection->second.message};
    erase_image(image); detections_.erase(detection); block(key, now);
    if (pair.image->width != pair.detections->image_width ||
      pair.image->height != pair.detections->image_height)
    {
      event(pair.detections->header, "image_size_mismatch"); return {};
    }
    return pair;
  }
};
}  // namespace srm_armor_refiner
