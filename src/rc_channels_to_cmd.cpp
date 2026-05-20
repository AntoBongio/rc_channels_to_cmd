/*
 * Copyright 2026 Thales Alenia Space
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rc_channels_to_cmd/rc_channels_to_cmd.hpp"

using std::placeholders::_1;

namespace rc_channels_to_cmd
{

/* ══════════════════════════════════════════════════════════════════════════════
 * Static helpers
 * ══════════════════════════════════════════════════════════════════════════════ */

std::array<int32_t, 16> RcChannelsToCmd::to_array(const CRSFChannels16 & msg)
{
  // Flatten the 16 named fields into a uniform array so the rest of the
  // code can address channels by 0-based index instead of by field name.
  return {
    msg.ch1,  msg.ch2,  msg.ch3,  msg.ch4,
    msg.ch5,  msg.ch6,  msg.ch7,  msg.ch8,
    msg.ch9,  msg.ch10, msg.ch11, msg.ch12,
    msg.ch13, msg.ch14, msg.ch15, msg.ch16
  };
}

double RcChannelsToCmd::normalise(int32_t raw)
{
  // Two-slope mapping so the slight raw asymmetry (819 units low side,
  // 820 units high side) does not bias the zero point.
  const double d = static_cast<double>(raw);
  if (d >= CRSF_CENTRE) {
    return (d - CRSF_CENTRE) / (CRSF_MAX - CRSF_CENTRE);  //  0.0 … +1.0
  } else {
    return (d - CRSF_CENTRE) / (CRSF_CENTRE - CRSF_MIN);  // −1.0 …  0.0
  }
}

double RcChannelsToCmd::apply_deadband(double value, double deadband)
{
  // Zero anything within ±deadband of centre.
  // Rescale the remainder so output still reaches ±1.0 at full deflection
  // — no output discontinuity at the deadband boundary.
  if (std::abs(value) <= deadband) {
    return 0.0;
  }
  const double sign  = (value > 0.0) ? 1.0 : -1.0;
  const double inner = std::abs(value) - deadband;
  const double range = 1.0 - deadband;    // guaranteed > 0 for deadband < 1
  return sign * (inner / range);
}

double RcChannelsToCmd::apply_throttle_curve(double t)
{
  // Cubic curve — identical to joy_translator::apply_throttle_curve().
  // Gives finer resolution near centre and full authority at extremes.
  return t * t * t;
}

/* ══════════════════════════════════════════════════════════════════════════════
 * position_to_loco_mode
 * ══════════════════════════════════════════════════════════════════════════════ */

uint8_t RcChannelsToCmd::position_to_loco_mode(int position)
{
  // Maps 0-based CH6 position index to a LocomotionModeMsg constant.
  // The mapping is fixed and documented in the header; positions beyond
  // NUM_MODE_POSITIONS are clamped to IDLE for safety.
  switch (position) {
    case 0: return LocomotionModeMsg::IDLE;
    case 1: return LocomotionModeMsg::CRABBING;
    case 2: return LocomotionModeMsg::TURNINPLACE;
    case 3: return LocomotionModeMsg::ACKERMANN_FRONT;
    case 4: return LocomotionModeMsg::DOUBLE_ACKERMANN;
    case 5: return LocomotionModeMsg::DOUBLE_ACKERMANN_SIDEWAYS;
    default:
      return LocomotionModeMsg::IDLE;
  }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Constructor
 * ══════════════════════════════════════════════════════════════════════════════ */

RcChannelsToCmd::RcChannelsToCmd(const rclcpp::NodeOptions & options)
: Node("rc_channels_to_cmd", options)
{
  /* ── Channel indices (1-based in YAML → 0-based stored) ─────────────────── */
  ch_roll_  = static_cast<int>(declare_parameter<int64_t>("channels.roll",  1)) - 1;
  ch_pitch_ = static_cast<int>(declare_parameter<int64_t>("channels.pitch", 2)) - 1;
  ch_fwd_   = static_cast<int>(declare_parameter<int64_t>("channels.fwd",   3)) - 1;
  ch_yaw_   = static_cast<int>(declare_parameter<int64_t>("channels.yaw",   4)) - 1;
  ch_arm_   = static_cast<int>(declare_parameter<int64_t>("channels.arm",   5)) - 1;
  ch_mode_  = static_cast<int>(declare_parameter<int64_t>("channels.mode",  6)) - 1;

  for (const int idx : {ch_roll_, ch_pitch_, ch_fwd_, ch_yaw_, ch_arm_, ch_mode_}) {
    if (idx < 0 || idx > 15) {
      throw std::invalid_argument(
        "rc_channels_to_cmd: channel index out of range [1,16]: " +
        std::to_string(idx + 1));
    }
  }

  /* ── Arm threshold ───────────────────────────────────────────────────────── */
  arm_threshold_ = static_cast<int32_t>(
    declare_parameter<int64_t>("arm.threshold", ARM_DEFAULT_THRESHOLD));

  /* ── Per-axis deadband (normalised 0.0–1.0) ─────────────────────────────── */
  deadband_roll_  = declare_parameter<double>("deadband.roll",  0.05);
  deadband_pitch_ = declare_parameter<double>("deadband.pitch", 0.05);
  deadband_fwd_   = declare_parameter<double>("deadband.fwd",   0.05);
  deadband_yaw_   = declare_parameter<double>("deadband.yaw",   0.05);

  /* ── Per-axis output scale ───────────────────────────────────────────────── */
  scale_roll_  = declare_parameter<double>("scale.roll",  1.0);
  scale_pitch_ = declare_parameter<double>("scale.pitch", 1.0);
  scale_fwd_   = declare_parameter<double>("scale.fwd",   1.0);
  scale_yaw_   = declare_parameter<double>("scale.yaw",   1.0);

  /* ── Rover geometry (spot-turn angular velocity computation) ─────────────── */
  X_     = declare_parameter<double>("X",     740.0);   // mm
  Y_     = declare_parameter<double>("Y",     439.0);   // mm
  B_     = declare_parameter<double>("B",     200.0);   // mm
  C_     = declare_parameter<double>("C",     100.0);   // mm
  T_MIN_ = declare_parameter<double>("T_MIN", 1.22);    // rad

  /* ── Per-mode scale factors (mirror joy_translator parameters) ───────────── */
  translation_throttle_scale_  = declare_parameter<double>("translation.throttle_scale",  0.5);
  translation_roll_scale_      = declare_parameter<double>("translation.roll_scale",      0.55);
  spot_turning_throttle_scale_ = declare_parameter<double>("spot_turning.throttle_scale", 0.5);
  ackerman_throttle_scale_     = declare_parameter<double>("ackerman.throttle_scale",     0.5);
  ackerman_roll_scale_         = declare_parameter<double>("ackerman.roll_scale",         0.20);
  ackerman_alt_throttle_scale_ = declare_parameter<double>("ackerman_alt.throttle_scale", 0.5);
  ackerman_alt_roll_scale_     = declare_parameter<double>("ackerman_alt.roll_scale",     0.08);

  /* ── CH6 mode band boundaries ────────────────────────────────────────────── */
  // Compute equal-width defaults first, then allow YAML overrides.
  divide_mode_bands();
  for (int i = 0; i < NUM_MODE_POSITIONS; ++i) {
    const std::string key = "mode_bands.band_" + std::to_string(i + 1) + "_lower";
    mode_band_lower_[i] = static_cast<int32_t>(
      declare_parameter<int64_t>(key, static_cast<int64_t>(mode_band_lower_[i])));
  }

  /* ── Input topic (no hardcoded name) ─────────────────────────────────────── */
  const std::string input_topic =
    declare_parameter<std::string>("input_topic", "crsf/channels");

  /* ── Publishers ──────────────────────────────────────────────────────────── */
  twist_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
    create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10));

  /* ── Service clients ─────────────────────────────────────────────────────── */
  loco_mode_client_ = create_client<LocomotionModeSrv>("set_locomotion_mode");
  susp_mode_client_ = create_client<SuspensionModeSrv>("set_suspension_mode");

  /* ── Subscription ────────────────────────────────────────────────────────── */
  crsf_sub_ = create_subscription<CRSFChannels16>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&RcChannelsToCmd::on_crsf_msg, this, _1));

  /* ── Startup diagnostics ─────────────────────────────────────────────────── */
  RCLCPP_INFO(get_logger(),
    "rc_channels_to_cmd started. Input: '%s'", input_topic.c_str());
  RCLCPP_INFO(get_logger(),
    "  Axes  — roll:CH%d  pitch:CH%d  fwd:CH%d  yaw:CH%d",
    ch_roll_+1, ch_pitch_+1, ch_fwd_+1, ch_yaw_+1);
  RCLCPP_INFO(get_logger(),
    "  Switches — arm:CH%d (thr=%d)  mode:CH%d",
    ch_arm_+1, arm_threshold_, ch_mode_+1);
  for (int i = 0; i < NUM_MODE_POSITIONS; ++i) {
    const int32_t upper = (i < NUM_MODE_POSITIONS - 1)
      ? mode_band_lower_[i + 1] - 1
      : static_cast<int32_t>(CRSF_MAX);
    RCLCPP_INFO(get_logger(),
      "  CH6 pos %d → mode %d: raw [%d … %d]",
      i + 1, i + 1, mode_band_lower_[i], upper);
  }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * divide_mode_bands
 * ══════════════════════════════════════════════════════════════════════════════ */
void RcChannelsToCmd::divide_mode_bands()
{
  // The GX12 6-pos switch outputs these fixed raw CRSF values:
  // pos 1=172, pos 2=500, pos 3=828, pos 4=1156, pos 5=1484, pos 6=1812
  // Each band's lower threshold is set halfway between adjacent positions
  // so any hardware rounding still lands cleanly in the correct band.
  static constexpr std::array<int32_t, NUM_MODE_POSITIONS> switch_values = {
    172, 500, 828, 1156, 1482, 1810
  };

  // Band 1 starts at the minimum possible raw value.
  mode_band_lower_[0] = static_cast<int32_t>(CRSF_MIN);

  // Each subsequent band starts halfway between the previous and current
  // switch output value.
  for (int i = 1; i < NUM_MODE_POSITIONS; ++i) {
    mode_band_lower_[i] = (switch_values[i - 1] + switch_values[i]) / 2;
  }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * classify_mode_position
 * ══════════════════════════════════════════════════════════════════════════════ */

int RcChannelsToCmd::classify_mode_position(int32_t raw) const
{
  // Walk from the highest band downward.
  // Return the first band whose lower edge the raw value meets.
  // Returns a 0-based position index in [0, NUM_MODE_POSITIONS−1].
  for (int i = NUM_MODE_POSITIONS - 1; i >= 0; --i) {
    if (raw >= mode_band_lower_[i]) {
      return i;
    }
  }
  return 0;  // raw below even the first band lower edge → position 0
}

/* ══════════════════════════════════════════════════════════════════════════════
 * publish_zero_twist
 * ══════════════════════════════════════════════════════════════════════════════ */

void RcChannelsToCmd::publish_zero_twist()
{
  if (twist_pub_->trylock()) {
    twist_pub_->msg_ = geometry_msgs::msg::Twist{};  // all fields = 0
    twist_pub_->unlockAndPublish();
  }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * send_loco_mode
 *
 * Identical logic to joy_translator::send_lcm_mode():
 *   - early-return if nothing changed
 *   - blocking wait for the service to become available (logs while waiting)
 *   - async request; updates current_locomotion_mode_ in the callback
 * ══════════════════════════════════════════════════════════════════════════════ */

void RcChannelsToCmd::send_loco_mode(uint8_t mode)
{
  if (mode == current_locomotion_mode_) {
    return;
  }

  auto request = std::make_shared<LocomotionModeSrv::Request>();
  request->loc_mode = emrs::from_uint8_to_loco_mode(mode);

  while (!loco_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(),
        "Interrupted while waiting for set_locomotion_mode service. Exiting.");
      return;
    }
    RCLCPP_WARN(get_logger(),
      "set_locomotion_mode service not available, waiting…");
  }

  loco_mode_client_->async_send_request(
    request,
    [this, request](rclcpp::Client<LocomotionModeSrv>::SharedFuture) {
      current_locomotion_mode_ = request->loc_mode.loc_mode;
      RCLCPP_INFO(get_logger(),
        "Locomotion mode confirmed: %d", current_locomotion_mode_);
    });
}

/* ══════════════════════════════════════════════════════════════════════════════
 * send_susp_mode
 *
 * Mirrors send_loco_mode() / joy_translator::send_susp_mode():
 *   - early-return if nothing changed
 *   - blocking wait for the service to become available
 *   - async request; updates current_suspension_mode_ in the callback
 * ══════════════════════════════════════════════════════════════════════════════ */

void RcChannelsToCmd::send_susp_mode(uint8_t mode)
{
  if (mode == current_suspension_mode_) {
    return;
  }

  auto request = std::make_shared<SuspensionModeSrv::Request>();
  request->susp_mode = emrs::from_uint8_to_susp_mode(mode);

  while (!susp_mode_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(),
        "Interrupted while waiting for set_suspension_mode service. Exiting.");
      return;
    }
    RCLCPP_WARN(get_logger(),
      "set_suspension_mode service not available, waiting…");
  }

  susp_mode_client_->async_send_request(
    request,
    [this, request](rclcpp::Client<SuspensionModeSrv>::SharedFuture) {
      current_suspension_mode_ = request->susp_mode.susp_mode;
      RCLCPP_INFO(get_logger(),
        "Suspension mode confirmed: %d", current_suspension_mode_);
    });
}

/* ══════════════════════════════════════════════════════════════════════════════
 * Per-mode Twist handlers
 *
 * Inputs are already normalised (± 1.0) and deadbanded.
 * Scale factors are applied inside each handler.
 * ══════════════════════════════════════════════════════════════════════════════ */

// CRABBING ────────────────────────────────────────────────────────────────────
// Lateral translation: throttle on linear.x, atan2 steering on angular.z.
void RcChannelsToCmd::handle_translation(
  double fwd, double roll, double pitch, double yaw)
{
  double throttle_x  = fwd * translation_throttle_scale_;
  // double steer_angle = std::atan2(roll, std::abs(fwd)) * translation_roll_scale_;
  double steer_angle = std::atan2(roll, pitch) * translation_roll_scale_;
  if (twist_pub_->trylock()) {
    twist_pub_->msg_.linear.x  = apply_throttle_curve(throttle_x);
    twist_pub_->msg_.linear.y  = 0.0;
    twist_pub_->msg_.linear.z  = 0.0;
    twist_pub_->msg_.angular.x = 0.0;
    twist_pub_->msg_.angular.y = 0.0;
    twist_pub_->msg_.angular.z = steer_angle;
    twist_pub_->unlockAndPublish();
  }
}

// TURNINPLACE ─────────────────────────────────────────────────────────────────
// Spot turn: yaw rate on linear.z, per-axis geometry on angular.y / angular.z.
void RcChannelsToCmd::handle_spot_turning(
  double fwd, double roll, double pitch, double yaw)
{
  double throttle_yaw = yaw * spot_turning_throttle_scale_;
  if (twist_pub_->trylock()) {
    twist_pub_->msg_.linear.x  = 0.0;
    twist_pub_->msg_.linear.y  = 0.0;
    twist_pub_->msg_.linear.z  = apply_throttle_curve(throttle_yaw);
    twist_pub_->msg_.angular.x = 0.0;
    twist_pub_->msg_.angular.y =
      roll  * (Y_ + B_ * std::cos(T_MIN_) + C_) * 1e-3;
    twist_pub_->msg_.angular.z =
      pitch * X_ * 1e-3;
    twist_pub_->unlockAndPublish();
  }
}

// DOUBLE_ACKERMANN / ACKERMANN_FRONT ──────────────────────────────────────────
// Standard Ackermann: throttle on linear.x, atan2 steering on angular.z.
void RcChannelsToCmd::handle_ackerman(
  double fwd, double roll, double /*pitch*/, double /*yaw*/)
{
  double throttle_x  = fwd * ackerman_throttle_scale_;
  double steer_angle = std::atan2(roll, std::abs(fwd)) * ackerman_roll_scale_;
  if (twist_pub_->trylock()) {
    twist_pub_->msg_.linear.x  = apply_throttle_curve(throttle_x);
    twist_pub_->msg_.linear.y  = 0.0;
    twist_pub_->msg_.linear.z  = 0.0;
    twist_pub_->msg_.angular.x = 0.0;
    twist_pub_->msg_.angular.y = 0.0;
    twist_pub_->msg_.angular.z = steer_angle;
    twist_pub_->unlockAndPublish();
  }
}

// DOUBLE_ACKERMANN_SIDEWAYS ───────────────────────────────────────────────────
// Alternate Ackermann: same shape but different scale factors.
// Mirrors joy_translator::handle_ackerman_alt().
void RcChannelsToCmd::handle_ackerman_alt(
  double fwd, double roll, double /*pitch*/, double /*yaw*/)
{
  double throttle_x  = fwd * ackerman_alt_throttle_scale_;
  double steer_angle = std::atan2(roll, std::abs(fwd)) * ackerman_alt_roll_scale_;
  if (twist_pub_->trylock()) {
    twist_pub_->msg_.linear.x  = apply_throttle_curve(throttle_x);
    twist_pub_->msg_.linear.y  = 0.0;
    twist_pub_->msg_.linear.z  = 0.0;
    twist_pub_->msg_.angular.x = 0.0;
    twist_pub_->msg_.angular.y = 0.0;
    twist_pub_->msg_.angular.z = steer_angle;
    twist_pub_->unlockAndPublish();
  }
}

/* ══════════════════════════════════════════════════════════════════════════════
 * on_crsf_msg  — called once per CRSF frame
 * ══════════════════════════════════════════════════════════════════════════════ */

void RcChannelsToCmd::on_crsf_msg(const CRSFChannels16::SharedPtr msg)
{
  const auto ch = to_array(*msg);

  /* ── 1. Arm / disarm ─────────────────────────────────────────────────────
   *
   * CH5 is a 2-position switch. High position (raw > arm_threshold_) = armed,
   * low position = disarmed. The instant it drops to low the node publishes
   * a zero Twist. The locomotion mode service is intentionally NOT called
   * here — the mode switch (CH6) remains the sole owner of the mode state.
   * No movement command reaches cmd_vel while disarmed.
   *
   * Startup safety: arming is inhibited until CH5 has been seen in the low
   * position at least once. This prevents the robot from arming
   * automatically if the switch is already high when the node starts.
   * ─────────────────────────────────────────────────────────────────────── */
  const bool now_armed = (ch[ch_arm_] > arm_threshold_);

  if (!now_armed) {
    arm_seen_low_ = true;
  }

  if (!armed_ && now_armed && arm_seen_low_) {
    RCLCPP_INFO(get_logger(), "ARMED");
  }

  if (armed_ && !now_armed) {
    RCLCPP_WARN(get_logger(), "DISARMED — zeroing Twist");
    publish_zero_twist();
  }

  armed_ = now_armed && arm_seen_low_;

  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "now_armed: %s", now_armed ? "True" : "False");
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "arm_seen_low_: %s", arm_seen_low_ ? "True" : "False");
  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "armed_: %s", armed_ ? "True" : "False");

  /* ── 2. Mode selection (CH6) ─────────────────────────────────────────────
   *
   * Mode is always read and the service called regardless of arm state,
   * so the operator can pre-select a mode before arming.
   * Disarming does not reset the mode — CH6 is the sole owner of mode state.
   * ─────────────────────────────────────────────────────────────────────── */
  const int position = classify_mode_position(ch[ch_mode_]);
  // DEBUG
  switch(position)
  {
    case 1:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: IDLE");
    case 2:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: CRABBING");
    case 3:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: TURNINPLACE");
    case 4:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: ACKERMANN_FRONT");
    case 5:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: DOUBLE_ACKERMANN");
    case 6:
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Mode selected: DOUBLE_ACKERMANN_SIDEWAYS");
  }
  const uint8_t new_mode = position_to_loco_mode(position);

  if (new_mode != requested_locomotion_mode_) {
    RCLCPP_INFO(get_logger(),
      "CH6 position %d → requesting locomotion mode %d", position + 1, new_mode);
    requested_locomotion_mode_ = new_mode;
  }

  // send_loco_mode is idempotent (no-op if current == requested).
  send_loco_mode(requested_locomotion_mode_);

  /* ── 3. Movement commands — only when armed ──────────────────────────────
   *
   * While disarmed we publish zero every cycle so downstream nodes never
   * act on a stale non-zero command sitting in their subscriber queue.
   * ─────────────────────────────────────────────────────────────────────── */
  if (!armed_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,"Not armed, publishing zero twist....");    
    publish_zero_twist();
    return;
  }

  /* Normalise raw CRSF → [−1.0, +1.0], then apply deadband. */
  double roll  = apply_deadband(normalise(ch[ch_roll_]),  deadband_roll_);
  double pitch = apply_deadband(normalise(ch[ch_pitch_]), deadband_pitch_);
  double fwd   = apply_deadband(normalise(ch[ch_fwd_]),   deadband_fwd_);
  double yaw   = apply_deadband(normalise(ch[ch_yaw_]),   deadband_yaw_);

  /* Apply per-axis output scales. */
  roll  *= scale_roll_;
  pitch *= scale_pitch_;
  fwd   *= scale_fwd_;
  yaw   *= scale_yaw_;

  /* ── 4. Dispatch by confirmed locomotion mode ────────────────────────────
   *
   * Twist geometry differs per locomotion mode — mirrors the
   * joy_translator handle_lcm_vector dispatch table.
   *
   * We dispatch on current_locomotion_mode_ (confirmed by the service
   * callback) rather than requested_locomotion_mode_ so the Twist
   * kinematics always match the mode the hardware is actually executing.
   * ─────────────────────────────────────────────────────────────────────── */
  switch (current_locomotion_mode_) {
    case LocomotionModeMsg::CRABBING:
      handle_translation(fwd, roll, pitch, yaw);
      break;

    case LocomotionModeMsg::TURNINPLACE:
      handle_spot_turning(fwd, roll, pitch, yaw);
      break;

    case LocomotionModeMsg::ACKERMANN_FRONT:
    case LocomotionModeMsg::DOUBLE_ACKERMANN:
      handle_ackerman(fwd, roll, pitch, yaw);
      break;

    case LocomotionModeMsg::DOUBLE_ACKERMANN_SIDEWAYS:
      handle_ackerman_alt(fwd, roll, pitch, yaw);
      break;

    case LocomotionModeMsg::IDLE:
    default:
      RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *this->get_clock(), 1000,
        "Locomotion mode IDLE — no Twist published");
      publish_zero_twist();
      break;
  }
}

}  // namespace rc_channels_to_cmd