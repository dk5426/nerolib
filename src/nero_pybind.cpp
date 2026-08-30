#include "common.h"
#include "config.h"
#include "controller_base.h"
#include "nero_interface.h"

#include <cstdio>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(_nerolib, m) {
  py::enum_<FirmwareVersion>(m, "FirmwareVersion")
      .value("DEFAULT", FirmwareVersion::DEFAULT)
      .value("V111", FirmwareVersion::V111)
      .value("V112", FirmwareVersion::V112)
      .export_values();
  py::class_<JointState>(m, "JointState")
      .def(py::init<>())
      .def(
          py::init<std::array<double, MOTOR_DOF>, std::array<double, MOTOR_DOF>,
                   std::array<double, MOTOR_DOF>, double>())
      .def_readwrite("timestamp", &JointState::timestamp)
      .def_readwrite("pos", &JointState::pos)
      .def_readwrite("vel", &JointState::vel)
      .def_readwrite("torque", &JointState::torque)
      .def_readwrite("gripper_pos", &JointState::gripper_pos)
      .def("__add__", [](const JointState &self,
                         const JointState &other) { return self + other; })
      .def("__mul__", [](const JointState &self, const float &scalar) {
        return self * scalar;
      });
  py::class_<Gain>(m, "Gain")
      .def(py::init<>())
      .def(py::init<std::array<double, MOTOR_DOF>,
                    std::array<double, MOTOR_DOF>>())
      .def_readwrite("kp", &Gain::kp)
      .def_readwrite("kd", &Gain::kd)
      .def("__add__",
           [](const Gain &self, const Gain &other) { return self + other; })
      .def("__mul__",
           [](const Gain &self, const float &scalar) { return self * scalar; });
  py::enum_<ControlMode>(m, "ControlMode")
      .value("STANDBY", ControlMode::STANDBY)
      .value("CAN_COMMAND", ControlMode::CAN_COMMAND)
      .value("TEACH_MODE", ControlMode::TEACH_MODE)
      .value("ETHERNET", ControlMode::ETHERNET)
      .value("WIFI", ControlMode::WIFI)
      .value("REMOTE", ControlMode::REMOTE)
      .value("LINKAGE_TEACHING", ControlMode::LINKAGE_TEACHING)
      .value("OFFLINE_TRAJECTORY", ControlMode::OFFLINE_TRAJECTORY)
      .export_values();

  py::enum_<MoveMode>(m, "MoveMode")
      .value("POSITION", MoveMode::POSITION)
      .value("JOINT", MoveMode::JOINT)
      .value("LINEAR", MoveMode::LINEAR)
      .value("CIRCULAR", MoveMode::CIRCULAR)
      .value("MIT", MoveMode::MIT)
      .value("CPV", MoveMode::CPV)
      .export_values();

  py::class_<NeroController>(m, "NeroController")
      .def(py::init<ControllerConfig>())
      .def("reset_to_home", &NeroController::resetToHome,
           py::arg("timeout_sec") = 20.0,
           py::arg("position_tolerance") = 0.02,
           py::arg("velocity_tolerance") = 0.05,
           py::arg("settle_time_sec") = 0.5,
           py::call_guard<py::gil_scoped_release>())
      .def("start", &NeroController::start)
      .def("stop", &NeroController::stop)
      .def("is_running", &NeroController::isRunning)
      .def("get_current_state", &NeroController::getCurrentState)
      .def("get_current_target", &NeroController::getCurrentTarget)
      .def("set_gain", &NeroController::setGain, py::arg("gain"))
      .def("enable_gravity_compensation", &NeroController::enableGravityCompensation, py::arg("enable"))
      .def("set_gravity_comp_scale", &NeroController::setGravityCompScale, py::arg("scale"))
      .def("set_mode", &NeroController::setMode, py::arg("control_mode"), py::arg("move_mode"))
      .def("set_target", &NeroController::setTarget,
           py::arg("new_target_pos"),
           py::arg("new_target_gripper_pos"),
           py::arg("minimum_duration") = 0.0f,
           py::arg("new_target_vel") =
               std::array<double, MOTOR_DOF>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
           py::arg("new_target_acc") =
               std::array<double, MOTOR_DOF>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  py::class_<ControllerConfig>(m, "ControllerConfig")
      .def(py::init<>())
      .def_readwrite("interface_name", &ControllerConfig::interface_name)
      .def_readwrite("urdf_path", &ControllerConfig::urdf_path)
      .def_readwrite("default_kp", &ControllerConfig::default_kp)
      .def_readwrite("default_kd", &ControllerConfig::default_kd)
      .def_readwrite("joint_vel_max", &ControllerConfig::joint_vel_max)
      .def_readwrite("joint_acc_max", &ControllerConfig::joint_acc_max)
      .def_readwrite("joint_jerk_max", &ControllerConfig::joint_jerk_max)
      .def_readwrite("home_position", &ControllerConfig::home_position)
      .def_readwrite("over_current_cnt_max",
                     &ControllerConfig::over_current_cnt_max)
      .def_readwrite("controller_freq_hz",
                     &ControllerConfig::controller_freq_hz)
      .def_readwrite("gravity_compensation",
                     &ControllerConfig::gravity_compensation)
      .def_readwrite("gravity_comp_scale", &ControllerConfig::gravity_comp_scale)
      .def_readwrite("gripper_on", &ControllerConfig::gripper_on)
      .def_readwrite("firmware_version", &ControllerConfig::firmware_version);

  // ---------------------------------------------------------------------
  // Low-level interface: diagnostics, firmware query, enable/disable.
  // NeroController is the normal entry point; this is for tooling that has
  // to inspect or reset the arm without running a control loop.
  // ---------------------------------------------------------------------

  py::enum_<ArmStatus>(m, "ArmStatus")
      .value("NORMAL", ArmStatus::NORMAL)
      .value("EMERGENCY_STOP", ArmStatus::EMERGENCY_STOP)
      .value("NO_SOLUTION", ArmStatus::NO_SOLUTION)
      .value("SINGULARITY", ArmStatus::SINGULARITY)
      .value("TARGET_ANGLE_EXCEEDS_LIMIT", ArmStatus::TARGET_ANGLE_EXCEEDS_LIMIT)
      .value("JOINT_COMMUNICATION_EXCEPTION",
             ArmStatus::JOINT_COMMUNICATION_EXCEPTION)
      .value("JOINT_BRAKE_NOT_RELEASED", ArmStatus::JOINT_BRAKE_NOT_RELEASED)
      .value("COLLISION", ArmStatus::COLLISION)
      .value("OVERSPEED_DURING_TEACHING", ArmStatus::OVERSPEED_DURING_TEACHING)
      .value("JOINT_STATUS_ABNORMAL", ArmStatus::JOINT_STATUS_ABNORMAL)
      .value("OTHER_EXCEPTION", ArmStatus::OTHER_EXCEPTION)
      .value("TEACHING_RECORD", ArmStatus::TEACHING_RECORD)
      .value("TEACHING_EXECUTION", ArmStatus::TEACHING_EXECUTION)
      .value("TEACHING_PAUSE", ArmStatus::TEACHING_PAUSE)
      .value("MAIN_CONTROLLER_NTC_OVER_TEMPERATURE",
             ArmStatus::MAIN_CONTROLLER_NTC_OVER_TEMPERATURE)
      .value("RELEASE_RESISTOR_NTC_OVER_TEMPERATURE",
             ArmStatus::RELEASE_RESISTOR_NTC_OVER_TEMPERATURE)
      .export_values();

  py::enum_<ArmController>(m, "ArmController")
      .value("POSITION_VELOCITY", ArmController::POSITION_VELOCITY)
      .value("MIT", ArmController::MIT)
      .value("INVALID", ArmController::INVALID)
      .export_values();

  py::enum_<EmergencyStop>(m, "EmergencyStop")
      .value("INVALID", EmergencyStop::INVALID)
      .value("STOP", EmergencyStop::STOP)
      .value("RESUME", EmergencyStop::RESUME)
      .export_values();

  // Bitfields cannot be bound by pointer-to-member, so read them in lambdas.
  py::class_<DriverStatus>(m, "DriverStatus")
      .def_property_readonly("status", [](const DriverStatus &s) { return s.status; })
      .def_property_readonly("voltage_too_low", [](const DriverStatus &s) { return s.voltage_too_low; })
      .def_property_readonly("motor_overheating", [](const DriverStatus &s) { return s.motor_overheating; })
      .def_property_readonly("driver_overcurrent", [](const DriverStatus &s) { return s.driver_overcurrent; })
      .def_property_readonly("driver_overheating", [](const DriverStatus &s) { return s.driver_overheating; })
      .def_property_readonly("collision_status", [](const DriverStatus &s) { return s.collision_status; })
      .def_property_readonly("driver_error_status", [](const DriverStatus &s) { return s.driver_error_status; })
      .def_property_readonly("driver_enable_status", [](const DriverStatus &s) { return s.driver_enable_status; })
      .def_property_readonly("stall_status", [](const DriverStatus &s) { return s.stall_status; })
      .def("__repr__", [](const DriverStatus &s) {
        return "<DriverStatus enabled=" + std::string(s.driver_enable_status ? "1" : "0") +
               " error=" + std::string(s.driver_error_status ? "1" : "0") +
               " raw=0x" + [&] { char b[8]; std::snprintf(b, sizeof(b), "%02X", s.status); return std::string(b); }() + ">";
      });

  py::class_<GripperStatus>(m, "GripperStatus")
      .def_property_readonly("status", [](const GripperStatus &s) { return s.status; })
      .def_property_readonly("voltage_too_low", [](const GripperStatus &s) { return s.voltage_too_low; })
      .def_property_readonly("motor_overheating", [](const GripperStatus &s) { return s.motor_overheating; })
      .def_property_readonly("driver_overcurrent", [](const GripperStatus &s) { return s.driver_overcurrent; })
      .def_property_readonly("driver_overheating", [](const GripperStatus &s) { return s.driver_overheating; })
      .def_property_readonly("sensor_status", [](const GripperStatus &s) { return s.sensor_status; })
      .def_property_readonly("driver_error_status", [](const GripperStatus &s) { return s.driver_error_status; })
      .def_property_readonly("driver_enable_status", [](const GripperStatus &s) { return s.driver_enable_status; })
      .def_property_readonly("homing_status", [](const GripperStatus &s) { return s.homing_status; });

  py::class_<NeroInterface>(m, "NeroInterface")
      .def(py::init<std::string, bool, FirmwareVersion>(),
           py::arg("interface_name"), py::arg("gripper_active") = false,
           py::arg("firmware_version") = FirmwareVersion::DEFAULT)
      // --- read-only: safe with the arm powered down -------------------
      .def("query_firmware_version", &NeroInterface::query_firmware_version,
           py::arg("timeout_sec") = 1.0f,
           py::call_guard<py::gil_scoped_release>())
      .def("get_firmware_version", &NeroInterface::get_firmware_version)
      .def("get_driver_status", &NeroInterface::get_driver_status, py::arg("motor_id"))
      .def("get_gripper_status", &NeroInterface::get_gripper_status)
      .def("get_arm_status", &NeroInterface::get_arm_status)
      .def("get_control_mode", &NeroInterface::get_control_mode)
      .def("get_move_mode", &NeroInterface::get_move_mode)
      .def("get_current_state", &NeroInterface::get_current_state)
      .def("is_arm_enabled", &NeroInterface::is_arm_enabled)
      .def("is_gripper_enabled", &NeroInterface::is_gripper_enabled)
      .def("get_nero_interface_name", &NeroInterface::get_nero_interface_name)
      // --- these MOVE or POWER the arm ---------------------------------
      .def("enable_arm", &NeroInterface::enable_arm,
           py::call_guard<py::gil_scoped_release>())
      .def("disable_arm", &NeroInterface::disable_arm,
           py::call_guard<py::gil_scoped_release>())
      .def("enable_gripper", &NeroInterface::enable_gripper,
           py::call_guard<py::gil_scoped_release>())
      .def("disable_gripper", &NeroInterface::disable_gripper,
           py::call_guard<py::gil_scoped_release>())
      .def("set_to_damping_mode", &NeroInterface::set_to_damping_mode,
           py::call_guard<py::gil_scoped_release>())
      .def("set_emergency_stop", &NeroInterface::set_emergency_stop,
           py::arg("emergency_stop"))
      .def("set_arm_mode", &NeroInterface::set_arm_mode,
           py::arg("ctrl_mode"), py::arg("move_mode"), py::arg("speed_rate"),
           py::arg("arm_controller"))
      .def_static("firmware_version_from_string",
                  &NeroInterface::firmware_version_from_string,
                  py::arg("version"));

  // Blocking helpers that retry until the arm reports the requested state.
  m.def("reset_arm", &::reset_arm, py::arg("interface"),
        py::arg("timeout_sec") = 5.0f, py::call_guard<py::gil_scoped_release>());
  m.def("enable_arm", &::enable_arm, py::arg("interface"),
        py::arg("timeout_sec") = 5.0f, py::call_guard<py::gil_scoped_release>());
  m.def("disable_arm", &::disable_arm, py::arg("interface"),
        py::arg("timeout_sec") = 5.0f, py::call_guard<py::gil_scoped_release>());

  // Convenience: open the bus, ask the arm its version, close. Read-only.
  m.def(
      "query_firmware_version",
      [](const std::string &interface_name, float timeout_sec) {
        NeroInterface iface(interface_name, false, FirmwareVersion::DEFAULT);
        return iface.query_firmware_version(timeout_sec);
      },
      py::arg("interface_name"), py::arg("timeout_sec") = 1.0f,
      py::call_guard<py::gil_scoped_release>());
}
