from typing import overload

class FirmwareVersion:
    """Nero arm firmware selector; see ControllerConfig.firmware_version."""

    DEFAULT: FirmwareVersion
    V111: FirmwareVersion
    V112: FirmwareVersion

class ControlMode:
    STANDBY: ControlMode
    CAN_COMMAND: ControlMode
    TEACH_MODE: ControlMode
    ETHERNET: ControlMode
    WIFI: ControlMode
    REMOTE: ControlMode
    LINKAGE_TEACHING: ControlMode
    OFFLINE_TRAJECTORY: ControlMode

class MoveMode:
    POSITION: MoveMode
    JOINT: MoveMode
    LINEAR: MoveMode
    CIRCULAR: MoveMode
    MIT: MoveMode
    CPV: MoveMode

class ArmStatus:
    NORMAL: ArmStatus
    EMERGENCY_STOP: ArmStatus
    NO_SOLUTION: ArmStatus
    SINGULARITY: ArmStatus
    TARGET_ANGLE_EXCEEDS_LIMIT: ArmStatus
    JOINT_COMMUNICATION_EXCEPTION: ArmStatus
    JOINT_BRAKE_NOT_RELEASED: ArmStatus
    COLLISION: ArmStatus
    OVERSPEED_DURING_TEACHING: ArmStatus
    JOINT_STATUS_ABNORMAL: ArmStatus
    OTHER_EXCEPTION: ArmStatus
    TEACHING_RECORD: ArmStatus
    TEACHING_EXECUTION: ArmStatus
    TEACHING_PAUSE: ArmStatus
    MAIN_CONTROLLER_NTC_OVER_TEMPERATURE: ArmStatus
    RELEASE_RESISTOR_NTC_OVER_TEMPERATURE: ArmStatus

class ArmController:
    POSITION_VELOCITY: ArmController
    MIT: ArmController
    INVALID: ArmController

class EmergencyStop:
    INVALID: EmergencyStop
    STOP: EmergencyStop
    RESUME: EmergencyStop

class DriverStatus:
    @property
    def status(self) -> int: ...
    @property
    def voltage_too_low(self) -> bool: ...
    @property
    def motor_overheating(self) -> bool: ...
    @property
    def driver_overcurrent(self) -> bool: ...
    @property
    def driver_overheating(self) -> bool: ...
    @property
    def collision_status(self) -> bool: ...
    @property
    def driver_error_status(self) -> bool: ...
    @property
    def driver_enable_status(self) -> bool: ...
    @property
    def stall_status(self) -> bool: ...

class GripperStatus:
    @property
    def status(self) -> int: ...
    @property
    def voltage_too_low(self) -> bool: ...
    @property
    def motor_overheating(self) -> bool: ...
    @property
    def driver_overcurrent(self) -> bool: ...
    @property
    def driver_overheating(self) -> bool: ...
    @property
    def sensor_status(self) -> bool: ...
    @property
    def driver_error_status(self) -> bool: ...
    @property
    def driver_enable_status(self) -> bool: ...
    @property
    def homing_status(self) -> bool: ...

class ControllerConfig:
    interface_name: str
    urdf_path: str
    default_kp: list[float]
    default_kd: list[float]
    joint_vel_max: list[float]
    joint_acc_max: list[float]
    joint_jerk_max: list[float]
    home_position: list[float]
    over_current_cnt_max: int
    controller_freq_hz: float
    gravity_compensation: bool
    gravity_comp_scale: float
    gripper_on: bool
    firmware_version: FirmwareVersion
    def __init__(self) -> None: ...

class Gain:
    kp: list[float]
    kd: list[float]
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self,
        kp: list[float],
        kd: list[float],
    ) -> None: ...
    def __add__(self, other: Gain) -> Gain: ...
    def __mul__(self, scalar: float) -> Gain: ...

class JointState:
    timestamp: float
    pos: list[float]
    vel: list[float]
    torque: list[float]
    gripper_pos: float
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(
        self,
        pos: list[float],
        vel: list[float],
        torque: list[float],
        gripper_pos: float,
    ) -> None: ...
    def __add__(self, other: JointState) -> JointState: ...
    def __mul__(self, scalar: float) -> JointState: ...

class NeroController:
    def __init__(
        self,
        controller_config: ControllerConfig,
    ) -> None: ...
    def start(self) -> bool: ...
    def stop(self) -> None: ...
    def reset_to_home(
        self,
        timeout_sec: float = 20.0,
        position_tolerance: float = 0.02,
        velocity_tolerance: float = 0.05,
        settle_time_sec: float = 0.5,
    ) -> None: ...
    def is_running(self) -> bool: ...
    def get_current_state(self) -> JointState: ...
    def get_current_target(self) -> list[float]: ...
    def set_gain(self, gain: Gain) -> None: ...
    def enable_gravity_compensation(self, enable: bool) -> None: ...
    def set_gravity_comp_scale(self, scale: float) -> None: ...
    def set_mode(self, control_mode: ControlMode, move_mode: MoveMode) -> None: ...
    def set_target(
        self,
        new_target_pos: list[float],
        new_target_gripper_pos: float,
        minimum_duration: float = 0.0,
        new_target_vel: list[float] = [0.0] * 7,
        new_target_acc: list[float] = [0.0] * 7,
    ) -> None: ...

class NeroInterface:
    """Low-level CAN interface. NeroController is the normal entry point;
    use this for diagnostics, firmware queries and enable/disable tooling."""

    def __init__(
        self,
        interface_name: str,
        gripper_active: bool = False,
        firmware_version: FirmwareVersion = ...,
    ) -> None: ...
    # --- read-only: safe with the arm powered down ---
    def query_firmware_version(self, timeout_sec: float = 1.0) -> str: ...
    def get_firmware_version(self) -> FirmwareVersion: ...
    def get_driver_status(self, motor_id: int) -> DriverStatus: ...
    def get_gripper_status(self) -> GripperStatus: ...
    def get_arm_status(self) -> ArmStatus: ...
    def get_control_mode(self) -> ControlMode: ...
    def get_move_mode(self) -> MoveMode: ...
    def get_current_state(self) -> JointState: ...
    def is_arm_enabled(self) -> bool: ...
    def is_gripper_enabled(self) -> bool: ...
    def get_nero_interface_name(self) -> str: ...
    # --- these POWER or MOVE the arm ---
    def enable_arm(self) -> None: ...
    def disable_arm(self) -> None: ...
    def enable_gripper(self) -> None: ...
    def disable_gripper(self) -> None: ...
    def set_to_damping_mode(self) -> None: ...
    def set_emergency_stop(self, emergency_stop: EmergencyStop) -> None: ...
    def set_arm_mode(
        self,
        ctrl_mode: ControlMode,
        move_mode: MoveMode,
        speed_rate: int,
        arm_controller: ArmController,
    ) -> None: ...
    @staticmethod
    def firmware_version_from_string(version: str) -> FirmwareVersion: ...

# Blocking helpers: retry until the arm reports the requested state.
def reset_arm(interface: NeroInterface, timeout_sec: float = 5.0) -> None: ...
def enable_arm(interface: NeroInterface, timeout_sec: float = 5.0) -> None: ...
def disable_arm(interface: NeroInterface, timeout_sec: float = 5.0) -> None: ...

# Open the bus, ask the arm its version, close. Read-only.
def query_firmware_version(
    interface_name: str, timeout_sec: float = 1.0
) -> str: ...
