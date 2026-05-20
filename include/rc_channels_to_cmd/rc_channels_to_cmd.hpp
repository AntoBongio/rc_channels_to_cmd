/*
 * rc_channels_to_cmd.hpp
 *
 * ROS 2 node that translates raw CRSF channel data
 * (crsf_receiver_msg/msg/CRSFChannels16) into:
 *
 *   - geometry_msgs/Twist  published on ~/cmd_vel
 *   - set_locomotion_mode  service call (loc_modes_srvs/srv/LocModes)
 *   - set_suspension_mode  service call (loc_modes_srvs/srv/SuspModes)
 *
 * Mirrors the behaviour of joy_translator but reads CRSF channels
 * instead of sensor_msgs/Joy.
 *
 * ── Channel assignment (1-based, all overridable via YAML) ────────────────
 *
 *   CH1 – Roll             (right stick ↔, Mode II)
 *   CH2 – Pitch            (right stick ↕, Mode II)
 *   CH3 – Forward/back     (left  stick ↕, Mode II)
 *   CH4 – Yaw              (left  stick ↔, Mode II)
 *   CH5 – Arm / Disarm     (2-pos switch)
 *   CH6 – Locomotion mode  (6-pos switch)
 *
 * ── CH6 → locomotion mode mapping ────────────────────────────────────────
 *
 *   Position 1 → IDLE
 *   Position 2 → CRABBING
 *   Position 3 → TURNINPLACE
 *   Position 4 → ACKERMANN_FRONT
 *   Position 5 → DOUBLE_ACKERMANN
 *   Position 6 → DOUBLE_ACKERMANN_SIDEWAYS
 *
 * ── Safety contract ───────────────────────────────────────────────────────
 *
 *   CH5 ≤ arm_threshold  →  zero Twist published immediately.
 *                           Locomotion mode service is NOT touched on disarm;
 *                           CH6 remains the sole owner of the mode state.
 *   No movement command is ever sent while disarmed.
 *
 * ── CRSF normalisation ────────────────────────────────────────────────────
 *
 *   Raw range : 172 (min) … 991 (centre) … 1810 (max)
 *   Normalised: −1.0       …   0.0       …  +1.0
 *   Two-slope mapping so the slight range asymmetry does not bias centre.
 *   Per-channel deadband (default 0.05) zeroes stick noise near centre;
 *   output is rescaled so it still reaches ±1 at full deflection.
 *
 * ── Twist dispatch by locomotion mode ────────────────────────────────────
 *
 *   Mirrors joy_translator's handle_lcm_vector dispatch table.
 *   The Twist is computed differently per mode to match the kinematics
 *   expected by each locomotion controller:
 *
 *   IDLE                      → zero Twist
 *   CRABBING                  → handle_translation()
 *   TURNINPLACE               → handle_spot_turning()
 *   ACKERMANN_FRONT /
 *   DOUBLE_ACKERMANN          → handle_ackerman()
 *   DOUBLE_ACKERMANN_SIDEWAYS → handle_ackerman_alt()
 */

#ifndef RC_CHANNELS_TO_CMD__RC_CHANNELS_TO_CMD_HPP_
#define RC_CHANNELS_TO_CMD__RC_CHANNELS_TO_CMD_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <crsf_receiver_msg/msg/crsf_channels16.hpp>

#include <emrs_locomotion_control/locomotion_modes.hpp>
#include <loc_modes_srvs/srv/loc_modes.hpp>
#include <loc_modes_srvs/loc_modes_constants.hpp>

#include <loc_modes_srvs/srv/susp_modes.hpp>
#include <loc_modes_srvs/susp_modes_constants.hpp>
#include <emrs_suspension_control/suspension_modes.hpp>

namespace rc_channels_to_cmd
{

using CRSFChannels16    = crsf_receiver_msg::msg::CRSFChannels16;
using LocomotionModeMsg = loc_modes_srvs::msg::LocMode;
using LocomotionModeSrv = loc_modes_srvs::srv::LocModes;
using SuspensionModeMsg = loc_modes_srvs::msg::SuspMode;
using SuspensionModeSrv = loc_modes_srvs::srv::SuspModes;

/* ── CRSF hardware constants ─────────────────────────────────────────────── */
static constexpr double  CRSF_MIN    = 172.0;
static constexpr double  CRSF_MAX    = 1810.0;
static constexpr double  CRSF_CENTRE = 991.0;
static constexpr int32_t ARM_DEFAULT_THRESHOLD = 1500;

/* ── Number of positions on the mode-select switch ───────────────────────── */
static constexpr int NUM_MODE_POSITIONS = 6;

class RcChannelsToCmd : public rclcpp::Node
{
public:
  explicit RcChannelsToCmd(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~RcChannelsToCmd() = default;

private:
  /* ── Parameters ─────────────────────────────────────────────────────────── */

  // Channel indices stored 0-based internally (declared 1-based in YAML)
  int ch_roll_;
  int ch_pitch_;
  int ch_fwd_;
  int ch_yaw_;
  int ch_arm_;
  int ch_mode_;

  int32_t arm_threshold_;

  double deadband_roll_;
  double deadband_pitch_;
  double deadband_fwd_;
  double deadband_yaw_;

  double scale_roll_;
  double scale_pitch_;
  double scale_fwd_;
  double scale_yaw_;

  // Lower edge of each CH6 band (raw CRSF). Upper edge = next entry − 1.
  std::array<int32_t, NUM_MODE_POSITIONS> mode_band_lower_;

  // Rover geometry — used by handle_spot_turning() (all in mm, T_MIN in rad).
  // Match the defaults from joy_translator.
  double X_;
  double Y_;
  double B_;
  double C_;
  double T_MIN_;

  // Per-mode throttle / roll scale factors (mirror joy_translator parameters).
  double translation_throttle_scale_;
  double translation_roll_scale_;
  double spot_turning_throttle_scale_;
  double ackerman_throttle_scale_;
  double ackerman_roll_scale_;
  double ackerman_alt_throttle_scale_;
  double ackerman_alt_roll_scale_;

  /* ── Runtime state ───────────────────────────────────────────────────────── */
  bool armed_{false};
  bool arm_seen_low_{false};  // must see CH5 low before first arm is allowed

  uint8_t current_locomotion_mode_{LocomotionModeMsg::IDLE};
  uint8_t requested_locomotion_mode_{LocomotionModeMsg::IDLE};
  uint8_t current_suspension_mode_{SuspensionModeMsg::IDLE};
  uint8_t requested_suspension_mode_{SuspensionModeMsg::IDLE};

  /* ── ROS interfaces ──────────────────────────────────────────────────────── */
  rclcpp::Subscription<CRSFChannels16>::SharedPtr crsf_sub_;

  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> twist_pub_;

  rclcpp::Client<LocomotionModeSrv>::SharedPtr loco_mode_client_;
  rclcpp::Client<SuspensionModeSrv>::SharedPtr susp_mode_client_;

  /* ── Static helpers ──────────────────────────────────────────────────────── */

  // Copy named fields ch1…ch16 into a uniform array for indexed access.
  static std::array<int32_t, 16> to_array(const CRSFChannels16 & msg);

  // Two-slope normalisation: raw CRSF → [−1.0, +1.0].
  static double normalise(int32_t raw);

  // Deadband with smooth rescaling: no jump at the deadband edge.
  static double apply_deadband(double value, double deadband);

  // Cubic throttle curve: same as joy_translator::apply_throttle_curve().
  static double apply_throttle_curve(double t);

  /* ── Setup helpers ───────────────────────────────────────────────────────── */

  // Divide [CRSF_MIN, CRSF_MAX] into NUM_MODE_POSITIONS equal bands.
  void divide_mode_bands();

  // Map a raw CH6 value to a 0-based position index [0, NUM_MODE_POSITIONS−1].
  int classify_mode_position(int32_t raw) const;

  // Map a 0-based position index to a LocomotionModeMsg uint8 constant.
  static uint8_t position_to_loco_mode(int position);

  /* ── Runtime helpers ─────────────────────────────────────────────────────── */

  // Publish a zero Twist (called immediately on disarm and while IDLE).
  void publish_zero_twist();

  // Send locomotion mode service request if mode has changed.
  // Twist dispatch uses current_locomotion_mode_ (confirmed by service callback).
  void send_loco_mode(uint8_t mode);

  // Send suspension mode service request if mode has changed.
  void send_susp_mode(uint8_t mode);

  /* ── Per-mode Twist handlers (mirror joy_translator handle_* methods) ────── */

  // CRABBING: lateral translation with atan2 steering.
  void handle_translation(double fwd, double roll, double pitch, double yaw);

  // TURNINPLACE: spot turn using rover geometry (X_, Y_, B_, C_, T_MIN_).
  void handle_spot_turning(double fwd, double roll, double pitch, double yaw);

  // DOUBLE_ACKERMANN / ACKERMANN_FRONT: standard Ackermann scales.
  void handle_ackerman(double fwd, double roll, double pitch, double yaw);

  // DOUBLE_ACKERMANN_SIDEWAYS: alternate Ackermann scales.
  void handle_ackerman_alt(double fwd, double roll, double pitch, double yaw);

  /* ── Main callback ───────────────────────────────────────────────────────── */
  void on_crsf_msg(const CRSFChannels16::SharedPtr msg);
};

}  // namespace rc_channels_to_cmd

#endif  // RC_CHANNELS_TO_CMD__RC_CHANNELS_TO_CMD_HPP_