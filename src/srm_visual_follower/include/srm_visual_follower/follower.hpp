#pragma once
#include <array>
#include <optional>
#include <string>
#include <srm_aim_interfaces/msg/armor_track_array.hpp>
namespace srm_visual_follower {
using Input=srm_aim_interfaces::msg::ArmorTrackArray;
using Track=srm_aim_interfaces::msg::ArmorTrack;
using Point=geometry_msgs::msg::Point;
struct Parameters {
  std::string selection_strategy{"closest_to_axis"};
  bool sticky_selection{true};
  int max_tracks{64};
  double target_timeout_sec{0.3}, max_state_age_sec{0.3}, input_timeout_sec{0.3};
  double future_tolerance_sec{0.02}, max_prediction_horizon_sec{0.5};
  double min_depth_m{0.05}, max_distance_m{50};
  bool extrinsics_valid{false};
  std::string reference_frame;
  std::array<double,3> translation_m{{0,0,0}};
  std::array<double,4> quaternion_xyzw{{0,0,0,1}};
  void validate() const;
};
struct Result {
  std_msgs::msg::Header header;
  std::string session_id, reason{"waiting_for_tracks"}, position_source{"none"};
  std::string reference_reason{"extrinsics_unavailable"};
  bool input_accepted{false}, target_valid{false}, reference_valid{false};
  uint64_t track_id{0};
  Point position, reference_position;
  builtin_interfaces::msg::Time state_stamp, observation_stamp, position_stamp;
  double camera_yaw_rad{0}, camera_pitch_rad{0}, reference_yaw_rad{0}, reference_pitch_rad{0};
};
class Follower {
public:
  explicit Follower(Parameters p={});
  Result process(const Input & input,int64_t now_ns);
  // Emits only source changes (prediction expires) or invalidation, never stale angles.
  std::optional<Result> poll(int64_t now_ns);
  Result invalidate(const std::string & reason);
private:
  Parameters p_;std::optional<uint64_t> selected_;
  std::optional<Track> cached_;Result last_;
  std::optional<int64_t> last_stamp_,last_now_;
  std::string session_,source_frame_;uint32_t width_{0},height_{0};
  std::optional<Result> evaluate(const Track & track,const std_msgs::msg::Header & header,
    const std::string & session,int64_t now_ns) const;
};
}
