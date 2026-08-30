#include "controller_base.h"
#include "utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

NeroController::NeroController(ControllerConfig controller_config)
    : controller_config_(controller_config),
      nero_interface_(controller_config.interface_name, controller_config.gripper_on,
                      controller_config.firmware_version),
      solver_(controller_config.urdf_path),
      otg_(1.0 / controller_config.controller_freq_hz),
      gain_(controller_config.default_kp, controller_config.default_kd),
      gravity_compensation_enabled_(controller_config.gravity_compensation),
      gravity_comp_scale_(controller_config.gravity_comp_scale) {
  if (!std::isfinite(controller_config.gravity_comp_scale) ||
      controller_config.gravity_comp_scale < 0.0f) {
    throw std::invalid_argument(
        "Gravity compensation scale must be finite and non-negative");
  }
  for (int i = 0; i < MOTOR_DOF; ++i) {
    // Ruckig's own validate() only rejects NaN and negatives, so an
    // infinite jerk limit reaches the profile solver and fails there
    // rather than here. Reject it at construction instead.
    if (!std::isfinite(controller_config.joint_jerk_max[i]) ||
        controller_config.joint_jerk_max[i] <= 0.0) {
      throw std::invalid_argument(
          "Joint jerk limit of joint " + std::to_string(i) +
          " must be finite and positive");
    }
  }
  start_time_us_ = get_time_us();
  target_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_acceleration_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_gripper_pos_ = 1.0f;
  minimum_duration_ = 0.0f;
}

NeroController::~NeroController() { stop(); }

void NeroController::resetToHome(const double timeout_sec,
                                 const double position_tolerance,
                                 const double velocity_tolerance,
                                 const double settle_time_sec) {
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    throw std::invalid_argument("Homing timeout must be finite and positive");
  }
  if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0) {
    throw std::invalid_argument(
        "Homing position tolerance must be finite and positive");
  }
  if (!std::isfinite(velocity_tolerance) || velocity_tolerance < 0.0) {
    throw std::invalid_argument(
        "Homing velocity tolerance must be finite and non-negative");
  }
  if (!std::isfinite(settle_time_sec) || settle_time_sec < 0.0) {
    throw std::invalid_argument(
        "Homing settle time must be finite and non-negative");
  }

  setTarget(
      controller_config_.home_position,
      1.0f,
      3.0f // minimum duration
  );

  const auto started_at = std::chrono::steady_clock::now();
  auto settled_since = started_at;
  bool settling = false;

  while (true) {
    auto current_state = getCurrentState();
    double max_position_error = 0.0;
    double max_velocity = 0.0;
    bool position_feedback_valid = true;
    bool feedback_valid = true;
    for (size_t i = 0; i < MOTOR_DOF; ++i) {
      if (!std::isfinite(current_state.pos[i])) {
        position_feedback_valid = false;
        feedback_valid = false;
        max_position_error = std::numeric_limits<double>::infinity();
      } else if (position_feedback_valid) {
        max_position_error = std::max(
            max_position_error,
            std::abs(current_state.pos[i] -
                     controller_config_.home_position[i]));
      }

      if (!std::isfinite(current_state.vel[i])) {
        feedback_valid = false;
        max_velocity = std::numeric_limits<double>::infinity();
      } else if (std::isfinite(max_velocity)) {
        max_velocity = std::max(max_velocity, std::abs(current_state.vel[i]));
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const bool within_limits =
        feedback_valid && max_position_error <= position_tolerance &&
        max_velocity <= velocity_tolerance;

    if (within_limits) {
      if (!settling) {
        settling = true;
        settled_since = now;
      }
      const double settled_for =
          std::chrono::duration<double>(now - settled_since).count();
      if (settled_for >= settle_time_sec) {
        spdlog::info(
            "Arm reset to home successfully (position error {:.4f} rad, "
            "velocity {:.4f} rad/s, settled {:.2f} s)",
            max_position_error, max_velocity, settled_for);
        return;
      }
    } else {
      settling = false;
    }

    const double elapsed =
        std::chrono::duration<double>(now - started_at).count();
    if (elapsed >= timeout_sec) {
      // Cancel the remaining trajectory and hold the latest measured pose.
      // This leaves the controller energized without continuing indefinitely.
      if (position_feedback_valid) {
        const float gripper_position =
            std::isfinite(current_state.gripper_pos)
                ? static_cast<float>(current_state.gripper_pos)
                : 1.0f;
        setTarget(current_state.pos, gripper_position, 0.0f);
      } else {
        // Never feed NaN/Inf positions back into Ruckig. With invalid position
        // feedback there is no trustworthy pose to hold, so disable safely.
        stop();
      }

      std::ostringstream message;
      message << "Homing timed out after " << elapsed
              << " s (max position error " << max_position_error
              << " rad, max velocity " << max_velocity << " rad/s"
              << (feedback_valid ? ")" : ", invalid feedback)");
      spdlog::error("{}", message.str());
      throw std::runtime_error(message.str());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool NeroController::start() {
  if (control_loop_running_.load()) {
    spdlog::warn("Controller already running");
    return false;
  }

  reset_arm(nero_interface_);
  reset_gripper(nero_interface_);

  // INITIALIZE TARGET TO CURRENT POSITION TO PREVENT HOMING GLITCH
  target_position_ = nero_interface_.get_current_state().pos;
  target_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_acceleration_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  target_gripper_pos_ = 1.0f;
  new_target_flag_.store(false);

  should_stop_.store(false);
  control_loop_running_.store(true);
  control_loop_thread_ = std::thread(&NeroController::controlLoop, this);

  return true;
}

void NeroController::stop() {
  if (!control_loop_running_.load()) {
    return;
  }

  should_stop_.store(true);

  if (control_loop_thread_.joinable()) {
    control_loop_thread_.join();
  }

  control_loop_running_.store(false);
  nero_interface_.set_to_damping_mode();
  spdlog::info("NeroController set to damping mode");

  disable_arm(nero_interface_);
  disable_gripper(nero_interface_);
}

void NeroController::setTarget(
    const std::array<double, MOTOR_DOF> &new_target_pos,
    const float new_target_gripper_pos,
    const float minimum_duration,
    const std::array<double, MOTOR_DOF> &new_target_vel,
    const std::array<double, MOTOR_DOF> &new_target_acc) {
  if (!std::isfinite(minimum_duration) || minimum_duration < 0.0f) {
    throw std::invalid_argument(
        "Target minimum duration must be finite and non-negative");
  }
  if (!std::isfinite(new_target_gripper_pos)) {
    throw std::invalid_argument("Target gripper position must be finite");
  }
  for (int i = 0; i < MOTOR_DOF; ++i) {
    if (!std::isfinite(new_target_pos[i]) ||
        !std::isfinite(new_target_vel[i]) ||
        !std::isfinite(new_target_acc[i])) {
      throw std::invalid_argument("Joint target values must be finite");
    }
    if (std::abs(new_target_vel[i]) >
        controller_config_.joint_vel_max[i] + 1e-9) {
      throw std::invalid_argument("Joint target velocity exceeds configured limit");
    }
    if (std::abs(new_target_acc[i]) >
        controller_config_.joint_acc_max[i] + 1e-9) {
      throw std::invalid_argument(
          "Joint target acceleration exceeds configured limit");
    }
  }

  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ = new_target_pos;
  target_velocity_ = new_target_vel;
  target_acceleration_ = new_target_acc;
  target_gripper_pos_ = new_target_gripper_pos;
  minimum_duration_ = minimum_duration;
  new_target_flag_.store(true);
  spdlog::debug("Target set to: [{}]", ::join(new_target_pos));
}

bool NeroController::isRunning() const { return control_loop_running_.load(); }

std::array<double, MOTOR_DOF> NeroController::getCurrentTarget() {
  std::lock_guard<std::mutex> lock(target_mutex_);
  return target_position_;
}

JointState NeroController::getCurrentState() {
  return nero_interface_.get_current_state();
}

void NeroController::setGain(const Gain &gain) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  gain_ = gain;
  spdlog::info("Gain updated: kp[0]={:.2f}, kd[0]={:.2f}", gain.kp[0], gain.kd[0]);
}

void NeroController::enableGravityCompensation(bool enable) {
  gravity_compensation_enabled_.store(enable);
  spdlog::info("Gravity compensation {}", enable ? "enabled" : "disabled");
}

void NeroController::setGravityCompScale(float scale) {
  if (!std::isfinite(scale) || scale < 0.0f) {
    throw std::invalid_argument(
        "Gravity compensation scale must be finite and non-negative");
  }
  gravity_comp_scale_.store(scale);
  spdlog::info("Gravity compensation scale set to {:.2f}", scale);
}

void NeroController::setMode(ControlMode control_mode, MoveMode move_mode) {
  ArmController arm_controller = (move_mode == MoveMode::MIT) ? ArmController::MIT : ArmController::POSITION_VELOCITY;
  nero_interface_.set_arm_mode(control_mode, move_mode, 100, arm_controller);
  spdlog::info("Mode updated: ControlMode={}, MoveMode={}", (int)control_mode, (int)move_mode);
}

void NeroController::driverProtection() {
  bool over_current = false;
  for (int i = 0; i < MOTOR_DOF; ++i) {
    DriverStatus driver_status = nero_interface_.get_driver_status(i);
    if (driver_status.driver_overcurrent) {
      over_current = true;
      spdlog::warn("Over current detected once on joint {}", i);
      break;
    }
    if (driver_status.driver_error_status) {
      nero_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Driver error detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.driver_overheating) {
      nero_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Driver overheating detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.collision_status) {
      nero_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Collision detected on joint " +
                               std::to_string(i) +
                               ". Please restart the program.");
    }
    if (driver_status.stall_status) {
      nero_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Stall detected on joint " + std::to_string(i) +
                               ". Please restart the program.");
    }
  }

  if (over_current) {
    over_current_cnt_++;
    if (over_current_cnt_ > controller_config_.over_current_cnt_max) {
      nero_interface_.set_emergency_stop(EmergencyStop::STOP);
      throw std::runtime_error("Over current detected too many times. Please "
                               "restart the program.");
    }
  } else {
    over_current_cnt_ = 0;
  }
}

void NeroController::controlLoop() {
  spdlog::info("Starting Nero controller control loop at {}Hz",
               controller_config_.controller_freq_hz);

  ruckig::InputParameter<MOTOR_DOF> input;
  ruckig::OutputParameter<MOTOR_DOF> output;

  JointState initial_joint_state = nero_interface_.get_current_state();
  input.current_position = initial_joint_state.pos;
  input.current_velocity = initial_joint_state.vel;
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    input.target_position = target_position_;
    input.target_velocity = target_velocity_;
    input.target_acceleration = target_acceleration_;
  }

  input.max_velocity = controller_config_.joint_vel_max;
  input.max_acceleration = controller_config_.joint_acc_max;
  // Must be finite -- see the joint_jerk_max comment in config.h. Left
  // unset it defaults to +infinity, which validate() accepts and the
  // third-order profile solver then fails on intermittently.
  input.max_jerk = controller_config_.joint_jerk_max;

  JointState output_joint_cmd;

  // Consecutive otg_.update() failures, for rate-limiting the error log.
  int ruckig_failure_count = 0;

  RateLimiter rate_limiter(controller_config_.controller_freq_hz);
  while (!should_stop_.load()) {
    if (new_target_flag_.exchange(false)) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      input.control_interface = ruckig::ControlInterface::Position;
      input.target_position = target_position_;
      input.target_velocity = target_velocity_;
      input.target_acceleration = target_acceleration_;
      input.minimum_duration = minimum_duration_;
      trajectory_active_.store(true);
      spdlog::debug("New target received: [{}]", ::join(input.target_position));
    }
    auto current_joint_state = nero_interface_.get_current_state();

    // Check if we're in compliant mode (all kp effectively zero)
    bool all_kp_zero = false;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      all_kp_zero = true;
      for (int i = 0; i < MOTOR_DOF; ++i) {
        if (gain_.kp[i] > 0.001) { all_kp_zero = false; break; }
      }
    }

    if (all_kp_zero) {
      // COMPLIANT MODE: Skip trajectory entirely.
      // Track actual arm position so there is no spring-back.
      // Only apply gravity compensation torque + damping.
      trajectory_active_.store(false);
      new_target_flag_.store(false);

      std::array<double, MOTOR_DOF> gravity_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      if (gravity_compensation_enabled_.load()) {
        gravity_compensation = solver_.inverse_dynamics(current_joint_state.pos,
                                                       current_joint_state.vel,
                                                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        float scale = gravity_comp_scale_.load();
        for (int i = 0; i < MOTOR_DOF; ++i) {
          gravity_compensation[i] *= scale;
        }
      }

      // Update Ruckig state and internal target to track the arm.
      // The acceleration has to be re-anchored alongside them: nothing
      // measures it, and whatever output.pass_to_input() last wrote
      // describes a trajectory that is no longer running. Leaving it
      // stale starts the next trajectory from a state the arm is not
      // in, which is one of the ways the profile solver fails.
      input.current_position = current_joint_state.pos;
      input.current_velocity = current_joint_state.vel;
      input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      Gain current_gain;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_position_ = current_joint_state.pos;
        target_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        output_joint_cmd.pos = current_joint_state.pos;
        output_joint_cmd.vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        output_joint_cmd.torque = gravity_compensation;
        current_gain = gain_;
      }

      nero_interface_.set_joint_pos_vel_torque(output_joint_cmd, current_gain);

    } else if (trajectory_active_.load()) {
      ruckig::Result result = otg_.update(input, output);

      if (result == ruckig::Result::Working ||
          result == ruckig::Result::Finished) {
        output_joint_cmd.pos = output.new_position;
        output_joint_cmd.vel = output.new_velocity;

        Gain current_gain;
        if (gravity_compensation_enabled_.load()) {
          std::array<double, MOTOR_DOF> gravity_compensation =
              solver_.inverse_dynamics(current_joint_state.pos,
                                       current_joint_state.vel,
                                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
          float scale = gravity_comp_scale_.load();
          for(int i=0; i<MOTOR_DOF; ++i) {
            gravity_compensation[i] *= scale;
          }
          output_joint_cmd.torque = gravity_compensation;
        } else {
          output_joint_cmd.torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          current_gain = gain_;
        }
        nero_interface_.set_joint_pos_vel_torque(output_joint_cmd, current_gain);
        output.pass_to_input(input);
        ruckig_failure_count = 0;

        if (result == ruckig::Result::Finished) {
          double terminal_speed = 0.0;
          for (const double velocity : output.new_velocity) {
            terminal_speed = std::max(terminal_speed, std::abs(velocity));
          }

          if (terminal_speed > 1e-5) {
            // A fixed position paired with nonzero velocity is not a valid
            // idle MIT command. If the producer misses its next streaming
            // knot, use Ruckig's velocity interface to decelerate from the
            // completed output before entering position hold.
            input.control_interface = ruckig::ControlInterface::Velocity;
            input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            input.minimum_duration = 0.0;
            trajectory_active_.store(true);
            spdlog::debug(
                "Nonzero terminal velocity ({:.4f} rad/s); starting "
                "acceleration-limited stop",
                terminal_speed);
          } else {
            const std::array<double, MOTOR_DOF> zero = {
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            input.control_interface = ruckig::ControlInterface::Position;
            input.target_velocity = zero;
            input.target_acceleration = zero;
            {
              std::lock_guard<std::mutex> lock(target_mutex_);
              target_position_ = output.new_position;
              target_velocity_ = zero;
              target_acceleration_ = zero;
            }
            trajectory_active_.store(false);
            spdlog::debug("Trajectory completed at zero velocity");
          }
        }
      } else {
        // A failed calculation must not drop a control command. Every
        // other branch of this loop ends in set_joint_pos_vel_torque();
        // returning early here left the previous MIT command latched for
        // a tick with no gravity compensation behind it, which is what
        // the arm's momentary stall was.
        //
        // Re-anchor only the *current* state to measured feedback, so the
        // retry below starts from something consistent and finite. The
        // commanded target is deliberately kept: discarding it (the old
        // behaviour) abandoned the motion at wherever the arm physically
        // happened to be and waited for the producer's next knot, turning
        // a one-tick numerical failure into a visible hitch plus a
        // backstep to the lagging measured pose.
        if (ruckig_failure_count == 0 || ruckig_failure_count % 250 == 0) {
          // Rate-limited: we now retry every tick rather than idling, so
          // a persistent failure would otherwise log at the loop rate.
          spdlog::error("Ruckig update failed with result {} ({} so far)",
                        static_cast<int>(result), ruckig_failure_count + 1);
        }
        ++ruckig_failure_count;

        const std::array<double, MOTOR_DOF> zero = {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        input.control_interface = ruckig::ControlInterface::Position;
        input.current_position = current_joint_state.pos;
        input.current_velocity = current_joint_state.vel;
        input.current_acceleration = zero;
        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          input.target_position = target_position_;
          input.target_velocity = target_velocity_;
          input.target_acceleration = target_acceleration_;
        }

        // Hold this tick at the measured pose -- not at output_joint_cmd,
        // which zero-initialises and would command the arm to the zero
        // pose if the very first update() of a run failed.
        std::array<double, MOTOR_DOF> gravity_compensation = zero;
        if (gravity_compensation_enabled_.load()) {
          gravity_compensation = solver_.inverse_dynamics(
              current_joint_state.pos, current_joint_state.vel, zero);
          const float scale = gravity_comp_scale_.load();
          for (int i = 0; i < MOTOR_DOF; ++i) {
            gravity_compensation[i] *= scale;
          }
        }
        Gain current_gain;
        {
          std::lock_guard<std::mutex> lock(target_mutex_);
          current_gain = gain_;
        }
        output_joint_cmd.pos = current_joint_state.pos;
        output_joint_cmd.vel = zero;
        output_joint_cmd.torque = gravity_compensation;
        nero_interface_.set_joint_pos_vel_torque(output_joint_cmd, current_gain);

        // Left active on purpose: the input above is now consistent, so
        // the next tick re-solves toward the real target instead of
        // stranding the arm until the producer sends another knot.
        trajectory_active_.store(true);
      }
    } else {
      // No trajectory active, hold the last target position
      std::array<double, MOTOR_DOF> gravity_compensation = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      if (gravity_compensation_enabled_.load()) {
        gravity_compensation = solver_.inverse_dynamics(current_joint_state.pos,
                                                       current_joint_state.vel,
                                                       {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        const float scale = gravity_comp_scale_.load();
        for (int i = 0; i < MOTOR_DOF; ++i) {
          gravity_compensation[i] *= scale;
        }
      }
      
      // Update Ruckig input continuously with sensors so that a new
      // trajectory starts from the actual physical state. Acceleration
      // is zeroed rather than left alone for the same reason as in the
      // compliant branch above: the arm is holding, so its acceleration
      // is zero, and a stale value from the last trajectory would be
      // carried into the next calculation.
      input.current_position = current_joint_state.pos;
      input.current_velocity = current_joint_state.vel;
      input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      Gain current_gain;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        output_joint_cmd.pos = target_position_;
        output_joint_cmd.vel = target_velocity_;
        output_joint_cmd.torque = gravity_compensation;
        current_gain = gain_;
      }
      nero_interface_.set_joint_pos_vel_torque(output_joint_cmd, current_gain);
    }

    if (controller_config_.gripper_on) {
      nero_interface_.set_gripper(target_gripper_pos_, 0.1f, GripperCode::ENABLE);
    }

    // TODO: check if joint state is within limits
    driverProtection();
    rate_limiter.wait();
  }
  spdlog::info("Control loop stopped");
}
