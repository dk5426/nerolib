#!/usr/bin/env python3
"""
Ask the arm what firmware it is running
=======================================
Nerolib has to be told which CAN wire format to speak
(ControllerConfig.firmware_version) and does not auto-detect it. This script
asks the arm directly and tells you which constant to configure.

Read-only and safe: it opens the CAN socket, sends a single request frame
(CAN 0x4AF, the same exchange pyAgxArm's get_firmware() uses) and reads the
reply. It never enables motors, sets a mode, or commands motion, so it is safe
to run on a powered-but-idle arm.

Usage:
    python3 scripts/get_firmware.py                     # can_left + can_right
    python3 scripts/get_firmware.py can0
    python3 scripts/get_firmware.py can_left can_right --timeout 2.0

Requires the CAN interface to be up:
    sudo ip link set can0 up type can bitrate 1000000
"""

import argparse
import sys

DEFAULT_INTERFACES = ["can_left", "can_right"]


def load_nerolib():
    # Imported after argparse so --help works without nerolib installed.
    try:
        from nerolib import FirmwareVersion, NeroInterface
    except ImportError:
        sys.exit("nerolib not found -- activate the nerolib environment first")
    return FirmwareVersion, NeroInterface


def describe(version, FirmwareVersion, NeroInterface):
    """Map a reported version onto the constant and its wire behaviour."""
    fw = NeroInterface.firmware_version_from_string(version)
    notes = {
        FirmwareVersion.DEFAULT: "MIT code 0x04, 8-bit t_ff with per-joint "
                                 "scaling + CRC, ~188 mNm resolution at the shoulder",
        FirmwareVersion.V111: "MIT code 0x06, 12-bit t_ff over +/-16 Nm, no CRC, "
                              "~7.8 mNm resolution",
        FirmwareVersion.V112: "MIT code 0x06, 12-bit t_ff over +/-16 Nm, no CRC, "
                              "~7.8 mNm resolution, CPV mode available",
    }
    return fw, notes.get(fw, "")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("interfaces", nargs="*", default=None,
                    help=f"CAN interfaces to query (default: {' '.join(DEFAULT_INTERFACES)})")
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="seconds to wait for each arm's reply (default 1.0)")
    args = ap.parse_args()

    FirmwareVersion, NeroInterface = load_nerolib()
    interfaces = args.interfaces or DEFAULT_INTERFACES

    print("=" * 70)
    print("  NERO FIRMWARE QUERY  (read-only -- does not power the arm)")
    print("=" * 70)

    found = 0
    for name in interfaces:
        print(f"\n  [{name}]")
        try:
            iface = NeroInterface(name, False, FirmwareVersion.DEFAULT)
        except Exception as exc:                        # noqa: BLE001
            print(f"    could not open: {exc}")
            print(f"    is the interface up?  sudo ip link set {name} up "
                  f"type can bitrate 1000000")
            continue

        version = iface.query_firmware_version(args.timeout)
        if not version:
            print("    no reply.")
            print("    The interface opened, so either the arm is powered off,")
            print("    it is on a different bus, or the bitrate is wrong.")
            continue

        found += 1
        fw, note = describe(version, FirmwareVersion, NeroInterface)
        print(f"    software_version : {version}")
        print(f"    use              : FirmwareVersion.{fw.name}")
        print(f"    wire format      : {note}")

    print("\n" + "=" * 70)
    if found:
        print("  Set this in your ControllerConfig, e.g.:")
        print("      config.firmware_version = FirmwareVersion.V112")
    else:
        print("  No arm answered. Nothing was changed on any arm.")
    print("=" * 70)
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
