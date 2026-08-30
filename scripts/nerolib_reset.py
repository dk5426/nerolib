#!/usr/bin/env python3
"""
Nero arm reset and status check  (nerolib equivalent of pyagxarm_reset.py)
=========================================================================
Reads firmware, dumps per-joint driver status, then cycles disable -> enable
on every joint to clear latched motor errors. Use it when an arm refuses to
enable, reports a driver error, or comes up in a bad state after an e-stop.

  >>> THIS POWERS THE MOTORS DOWN AND BACK UP. <<<

Disabling cuts motor power: an unsupported arm WILL FALL, hard. Before
running the reset, put the arm in a low, resting, supported pose, or hold it.
There is a confirmation prompt.

The read-only parts are safe on their own -- use --status-only to inspect
firmware and driver flags without touching motor power at all.

Usage:
    python3 scripts/nerolib_reset.py --status-only        # safe: inspect only
    python3 scripts/nerolib_reset.py                      # inspect + reset
    python3 scripts/nerolib_reset.py can0 --firmware V112
    python3 scripts/nerolib_reset.py --status-only --watch 10

Requires the CAN interfaces to be up:
    sudo ip link set can0 up type can bitrate 1000000
"""

import argparse
import sys
import time

DEFAULT_INTERFACES = ["can_left", "can_right"]
JOINTS = range(7)  # nerolib indexes motors 0-6; the arm labels them 1-7

# Driver status bits worth calling out, in report order.
FLAGS = [
    ("driver_enable_status", "enabled"),
    ("driver_error_status", "ERROR"),
    ("voltage_too_low", "low-voltage"),
    ("motor_overheating", "motor-hot"),
    ("driver_overheating", "driver-hot"),
    ("driver_overcurrent", "overcurrent"),
    ("collision_status", "collision"),
    ("stall_status", "stalled"),
]


def load_nerolib():
    # Imported after argparse so --help works without nerolib installed.
    try:
        from nerolib import FirmwareVersion, NeroInterface
    except ImportError:
        sys.exit("nerolib not found -- activate the nerolib environment first")
    return FirmwareVersion, NeroInterface


def enum_name(value):
    return getattr(value, "name", str(value))


def joint_line(idx, status):
    """One-line summary for a joint, flagging only what is notable."""
    on = status.driver_enable_status
    bits = []
    for attr, label in FLAGS:
        if attr == "driver_enable_status":
            continue
        if getattr(status, attr):
            bits.append(label)
    detail = ", ".join(bits) if bits else "clean"
    return (f"    joint {idx + 1}: {'ENABLED ' if on else 'disabled'}  "
            f"raw=0x{status.status:02X}  {detail}")


def dump_status(iface, label):
    print(f"\n  {label}")
    states = [iface.get_driver_status(i) for i in JOINTS]
    for i, st in enumerate(states):
        print(joint_line(i, st))
    enabled = [st.driver_enable_status for st in states]
    errors = [st.driver_error_status for st in states]
    print(f"    summary: {sum(enabled)}/7 enabled, {sum(errors)} in error")
    return states


def inspect(iface, NeroInterface):
    """Read-only: firmware, arm status, per-joint driver flags."""
    version = iface.query_firmware_version(1.0)
    if version:
        detected = NeroInterface.firmware_version_from_string(version)
        print(f"    firmware      : {version}  -> FirmwareVersion.{detected.name}")
        if detected != iface.get_firmware_version():
            print(f"    ** MISMATCH: this script is speaking "
                  f"{enum_name(iface.get_firmware_version())} but the arm reports "
                  f"{detected.name}.")
            print(f"    ** Re-run with --firmware {detected.name}")
    else:
        print("    firmware      : no reply (arm powered off, or wrong bus?)")

    print(f"    arm status    : {enum_name(iface.get_arm_status())}")
    print(f"    control mode  : {enum_name(iface.get_control_mode())}")
    print(f"    move mode     : {enum_name(iface.get_move_mode())}")
    dump_status(iface, "driver status:")


def do_reset(iface, settle):
    """Cycle motor power to clear latched driver errors. MOVES/DROPS THE ARM."""
    print("\n    sending DISABLE to all joints...")
    iface.disable_arm()
    time.sleep(settle)
    dump_status(iface, "after disable:")

    print("\n    sending ENABLE to all joints...")
    iface.enable_arm()
    time.sleep(settle)
    states = dump_status(iface, "after enable:")

    ok = all(st.driver_enable_status for st in states)
    err = any(st.driver_error_status for st in states)
    if ok and not err:
        print("\n    RESET OK -- all 7 joints enabled, no errors latched.")
    elif ok:
        print("\n    PARTIAL -- all joints enabled but an error bit is still set.")
        print("    Power-cycle the arm's controller box if it persists.")
    else:
        bad = [i + 1 for i, st in enumerate(states) if not st.driver_enable_status]
        print(f"\n    FAILED -- joints {bad} did not come back enabled.")
        print("    Check the e-stop, the 48V supply, and the CAN wiring.")
    return ok and not err


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("interfaces", nargs="*", default=None,
                    help=f"CAN interfaces (default: {' '.join(DEFAULT_INTERFACES)})")
    ap.add_argument("--firmware", choices=("DEFAULT", "V111", "V112"), default="V112",
                    help="wire format to speak (default V112)")
    ap.add_argument("--status-only", action="store_true",
                    help="inspect only; never touches motor power (SAFE)")
    ap.add_argument("--watch", type=float, default=0.0,
                    help="with --status-only, re-poll driver status for N seconds")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="seconds to wait after each enable/disable (default 0.5)")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args()

    FirmwareVersion, NeroInterface = load_nerolib()
    interfaces = args.interfaces or DEFAULT_INTERFACES
    fw = getattr(FirmwareVersion, args.firmware)

    print("=" * 70)
    print("  NERO ARM RESET & STATUS CHECK")
    print("=" * 70)
    print(f"  interfaces : {', '.join(interfaces)}")
    print(f"  firmware   : {args.firmware}")
    print(f"  mode       : {'STATUS ONLY (safe)' if args.status_only else 'RESET (powers motors)'}")

    if not args.status_only:
        print()
        print("  " + "!" * 66)
        print("  !! DISABLE CUTS MOTOR POWER. AN UNSUPPORTED ARM WILL FALL.  !!")
        print("  !! Put the arm in a low, supported pose before continuing.  !!")
        print("  " + "!" * 66)
        if not args.yes:
            if input("\n  Type 'reset' to continue: ").strip().lower() != "reset":
                print("  aborted. Nothing was changed.")
                return 1

    results = {}
    for name in interfaces:
        print("\n" + "-" * 70)
        print(f"  [{name}]")
        print("-" * 70)
        try:
            iface = NeroInterface(name, False, fw)
        except Exception as exc:                        # noqa: BLE001
            print(f"    could not open: {exc}")
            print(f"    is it up?  sudo ip link set {name} up type can bitrate 1000000")
            results[name] = None
            continue

        try:
            inspect(iface, NeroInterface)

            if args.status_only:
                if args.watch > 0:
                    print(f"\n    watching for {args.watch:.0f}s (Ctrl-C to stop)...")
                    end = time.time() + args.watch
                    while time.time() < end:
                        time.sleep(1.0)
                        dump_status(iface, f"t={args.watch - (end - time.time()):.0f}s")
                results[name] = True
            else:
                results[name] = do_reset(iface, args.settle)
        except KeyboardInterrupt:
            print("\n    interrupted.")
            results[name] = None
        except Exception as exc:                        # noqa: BLE001
            print(f"    error: {exc}")
            results[name] = False

    print("\n" + "=" * 70)
    for name, ok in results.items():
        label = {True: "OK", False: "FAILED", None: "SKIPPED"}[ok]
        print(f"  {name:12s} : {label}")
    print("=" * 70)
    if not args.status_only:
        print("  Motors are left ENABLED and holding. Support the arm before")
        print("  disabling it again or powering down.")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
