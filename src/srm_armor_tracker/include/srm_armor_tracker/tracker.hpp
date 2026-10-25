#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <srm_aim_interfaces/msg/armor_location_array.hpp>
#include <srm_aim_interfaces/msg/armor_track_array.hpp>
namespace srm_armor_tracker {
using Input = srm_aim_interfaces::msg::ArmorLocationArray;
using Output = srm_aim_interfaces::msg::ArmorTrackArray;
using TrackMsg = srm_aim_interfaces::msg::ArmorTrack;
using Matrix = cv::Matx<double,6,6>;
using Vector = cv::Vec<double,6>;
struct Parameters {
  int max_tracks{32}, max_observations{64}, confirmation_hits{3}, velocity_hits{3};
  double acceleration_noise{4}, measurement_xy{0.03}, measurement_z{0.10};
  double initial_velocity_std{3}, gate_squared{11.345}, max_distance_m{1};
  double ambiguity_margin{0.5}, coast_sec{0.20}, delete_sec{0.30};
  double max_gap_sec{0.5}, time_reset_sec{1}, velocity_min_span_sec{0.08};
  double prediction_sec{0.10}, input_timeout_sec{0.5};
  void validate() const;
};
// Square minimum-cost assignment; dummy rows/columns are supplied by caller.
std::vector<int> hungarian(const std::vector<std::vector<double>> & costs);
bool valid_covariance(const Matrix & p);
class Tracker {
public:
  explicit Tracker(Parameters p = {});
  // nullopt means rejected duplicate/out-of-order input; state remains untouched.
  std::optional<Output> process(const Input & input);
  // Called once by the node on monotonic receipt timeout. No extrapolation to wall time.
  std::optional<Output> interrupt();
private:
  struct Track {
    uint64_t id{0}; uint8_t color{0}; Vector x{}; Matrix p{};
    int hits{1}, streak{1}; bool confirmed{false}, matched{false};
    int64_t first{0}, last_seen{0}, stamp{0};
    srm_aim_interfaces::msg::ArmorLocation observation;
    std::string reason{"new_track"};
  };
  Parameters p_; std::vector<Track> tracks_; uint64_t next_id_{1}, generation_{0};
  std::string prefix_; std::optional<int64_t> last_stamp_; Input metadata_;
  void predict(Track & t, int64_t stamp) const;
  bool update(Track & t, const srm_aim_interfaces::msg::ArmorLocation & m) const;
  TrackMsg message(const Track & t, uint8_t state, const std::string & reason) const;
  Output base(const Input & input, const std::string & reason) const;
  void reset();
};
}
