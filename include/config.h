#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"
#include <cstdint>

// Nero arm firmware version selector.
// These correspond to the arm's self-reported "software_version" field
// (e.g. pyAgxArm get_firmware()["software_version"]). AgileX calls these
// firmware versions; the arm's CAN packet calls the same field "software_version".
//   DEFAULT : software_version <= 1.10
//   V111    : software_version == 1.11
//   V112    : software_version >= 1.12
enum class FirmwareVersion : uint8_t {
  DEFAULT = 0,
  V111 = 1,
  V112 = 2,
};

class ControllerConfig {
public:
  std::string interface_name;
  std::string urdf_path;
  std::array<double, MOTOR_DOF> default_kp;
  std::array<double, MOTOR_DOF> default_kd;
  std::array<double, MOTOR_DOF> joint_vel_max;
  std::array<double, MOTOR_DOF> joint_acc_max;
  // Third-order limit for the Ruckig profile. It MUST be finite:
  // ruckig::Ruckig<MOTOR_DOF> is the jerk-limited solver, and
  // InputParameter::initialize() leaves max_jerk at +infinity, which
  // validate() accepts (it only rejects NaN and negatives) and the
  // position_third_step1/step2 root solvers then turn into NaN
  // intermediates -- an intermittent ErrorExecutionTimeCalculation /
  // ErrorSynchronizationCalculation on whichever ticks land in one of
  // those branches. Leaving it unset is what made the arm drop a
  // command mid-motion.
  //
  // What the value buys: a_max/j is how long the acceleration ramp
  // lasts, i.e. how much rounding goes on the start of a move. The
  // default below is that ratio at 5 ms, ~1.25 ticks of the 250 Hz
  // control loop -- as close to the previous (infinite) behaviour as
  // a well-conditioned solve gets, so this fixes the numerics without
  // retuning the arm's feel. Lower it deliberately if you want the
  // rounding, don't lower it by accident.
  std::array<double, MOTOR_DOF> joint_jerk_max;
  std::array<double, MOTOR_DOF> home_position;
  int over_current_cnt_max;
  double controller_freq_hz;
  bool gravity_compensation;
  float gravity_comp_scale;
  bool gripper_on;
  FirmwareVersion firmware_version;

  ControllerConfig(
      std::string interface_name = "can0",
      std::string urdf_path = "../urdf/nero_description.urdf",
      std::array<double, MOTOR_DOF> default_kp = {10.0, 10.0, 10.0, 10.0, 10.0,
                                                   10.0, 10.0},
      // kd=0.8 matches pyAgxArm reference value for nero motors
      std::array<double, MOTOR_DOF> default_kd = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8,
                                                   0.8},
      std::array<double, MOTOR_DOF> joint_vel_max = {1.5, 1.5, 1.5, 1.5,
                                                     1.5, 1.5, 1.5},
      std::array<double, MOTOR_DOF> joint_acc_max = {2.0, 2.0, 2.0, 2.0, 2.0,
                                                     2.0, 2.0},
      // 400 = joint_acc_max / 0.005 s; see the member declaration.
      std::array<double, MOTOR_DOF> joint_jerk_max = {400.0, 400.0, 400.0,
                                                      400.0, 400.0, 400.0,
                                                      400.0},
      std::array<double, MOTOR_DOF> home_position = {0.0, 0.0, 0.0, 0.0, 0.0,
                                                     0.0, 0.0},
      int over_current_cnt_max = 20, double controller_freq_hz = 250.0,
      bool gravity_compensation = true, float gravity_comp_scale = 1.0f,
      bool gripper_on = false,
      FirmwareVersion firmware_version = FirmwareVersion::DEFAULT)
      : interface_name(interface_name), urdf_path(urdf_path),
        default_kp(default_kp), default_kd(default_kd),
        joint_vel_max(joint_vel_max), joint_acc_max(joint_acc_max),
        joint_jerk_max(joint_jerk_max), home_position(home_position),
        over_current_cnt_max(over_current_cnt_max),
        controller_freq_hz(controller_freq_hz),
        gravity_compensation(gravity_compensation),
        gravity_comp_scale(gravity_comp_scale),
        gripper_on(gripper_on),
        firmware_version(firmware_version) {}
};

#endif // CONFIG_H
