#pragma once
#include <array>
#include <deque>
#include <optional>
#include <string>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
namespace srm_gimbal_follower {
struct Time {int64_t ros_ns;double steady_sec;};
struct Point {double x{0},y{0},z{0};};
struct Parameters {
  std::string mode{"simulation"};
  bool calibration_verified{false},allow_unverified_simulation_calibration{false};
  // Synthetic optical->FLU rotation. Not asserted to describe any real mount.
  std::array<double,4> quaternion_xyzw{{-.5,.5,-.5,.5}};
  std::array<double,3> translation_m{{0,0,0}};
  double yaw_feedback_sign{1},pitch_feedback_sign{-1},roll_feedback_sign{1};
  double yaw_zero_deg{0},pitch_zero_deg{0},roll_zero_deg{0};
  double kp{0.6},deadband_deg{0.1};
  double yaw_min_deg{-170},yaw_max_deg{170},pitch_min_deg{-45},pitch_max_deg{45};
  double max_step_deg{1.5},max_rate_deg_sec{30},max_visual_correction_deg{30};
  double visual_timeout_sec{0.30},feedback_timeout_sec{0.15},future_tolerance_sec{0.02};
  double alignment_gap_sec{0.10},feedback_gap_sec{0.20},history_sec{1.0};
  int history_capacity{200};
  double feedback_max_jump_deg{20},feedback_max_rate_deg_sec{200};
  double target_switch_delay_sec{0.30},target_switch_confirm_sec{0.15};
  bool allow_target_switch{true};
  double max_prediction_sec{0.5},prediction_grace_sec{0.03};
  double max_range_m{50},min_depth_m{0.05},control_period_sec{0.02},max_tick_gap_sec{0.10};
  void validate() const;
};
struct Visual {
  std::string session,id,source;
  int64_t source_ns{0},state_ns{0},observation_ns{0},position_ns{0};
  Point point;double yaw_rad{0},pitch_rad{0};
};
// Strictly parses the actual visual_follower diagnostic contract, not arbitrary messages.
std::optional<Visual> parse_visual(const diagnostic_msgs::msg::DiagnosticArray & message,
  const Parameters & p,std::string & reason);
struct Output {
  bool simulation_reference_valid{false},limited{false};
  std::string reason{"waiting_for_inputs"},session,id,source;
  double yaw_reference_deg{0},pitch_reference_deg{0},yaw_target_deg{0},pitch_target_deg{0};
  double yaw_error_deg{0},pitch_error_deg{0},dt_sec{0};
  int64_t source_ns{0},state_ns{0},position_ns{0},feedback_ns{0};
};
// FLU body->fixed reference Rz(yaw)*Ry(-pitch_up)*Rx(roll), angles degrees.
Point orientation(Point point,double yaw_deg,double pitch_up_deg,double roll_deg,bool inverse=false);
class Controller {
public:
  explicit Controller(Parameters p={});
  bool feedback(const geometry_msgs::msg::Vector3Stamped & message,Time now);
  bool visual(const diagnostic_msgs::msg::DiagnosticArray & message,Time now);
  Output tick(Time now);
  const std::string & locked_target() const{return locked_;}
  const std::string & pending_target() const{return pending_;}
  const std::string & last_rejection() const{return rejection_;}
private:
  struct Feedback {int64_t stamp;double yaw,pitch,roll;};
  Parameters p_;std::deque<Feedback> feedback_;
  std::optional<Visual> visual_;std::optional<double> feedback_receipt_,visual_receipt_;
  std::optional<double> last_tick_,last_steady_;std::optional<int64_t> last_ros_;
  std::optional<std::array<double,2>> reference_;
  std::string locked_,pending_,rejection_;double locked_seen_{0},pending_since_{0};int pending_count_{0};
  bool feedback_fault_{false};
  std::optional<int64_t> last_visual_source_;std::string last_visual_key_,last_visual_kind_;
  bool clock(Time now);
  std::optional<Feedback> aligned(int64_t stamp) const;
  Output stop(const std::string & reason);
};
}
