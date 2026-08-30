#!/usr/bin/env python3
"""
Compliant-mode / Wuji gravity-compensation check on real hardware
=================================================================
Puts one arm into compliant mode (kp = 0, gravity compensation on) and
measures how far it drifts while you leave it alone. That drift is the
number that tells you whether the payload model is right:

    holds position, moves easily by hand   -> payload modelled correctly
    sinks downward                         -> hand mass UNDER-modelled
    rises / pushes back                    -> hand mass OVER-modelled

>>> THIS POWERS THE ARM. Read the safety notes before running. <<<

  * start() power-cycles the motors -- the arm WILL FALL if it is not
    resting in a supported pose when you begin.
  * Begin with the arm hanging in a safe, low, supported pose.
  * Keep a hand on the arm and the e-stop within reach.
  * stop() puts the arm in damping mode and lowers it slowly, then waits
    for you to press Enter before cutting the motors.

Usage:
    python3 test/test_left_compliant.py                        # left arm, Wuji hand
    python3 test/test_left_compliant.py --side right
    python3 test/test_left_compliant.py --no-wuji              # bare arm, no hand
    python3 test/test_left_compliant.py --duration 20 --scale 0.8
    python3 test/test_left_compliant.py --dry-run              # no hardware; prints the plan
"""

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

URDF = {
    ("left", True): "urdf/nero_cone-e_left_fixed_wuji.urdf",
    ("left", False): "urdf/nero_cone-e_left_fixed.urdf",
    ("right", True): "urdf/right_arm_final_wuji.urdf",
    ("right", False): "urdf/right_arm_final.urdf",
}

DEFAULT_CAN = {"left": "can_left", "right": "can_right"}


def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--can", default=None, help="CAN interface (default: can_<side>)")
    ap.add_argument("--firmware", choices=("DEFAULT", "V111", "V112"), default="V112")
    ap.add_argument("--no-wuji", action="store_true",
                    help="use the bare-arm URDF (no hand fitted)")
    ap.add_argument("--duration", type=float, default=15.0,
                    help="seconds to hold compliant and measure drift")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="gravity_comp_scale; leave at 1.0 unless de-rating on purpose")
    ap.add_argument("--kp", type=float, default=0.0, help="position gain (0 = fully compliant)")
    ap.add_argument("--kd", type=float, default=0.5, help="damping gain")
    ap.add_argument("--rate", type=float, default=5.0, help="print rate, Hz")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would happen and exit; touches no hardware")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    return ap.parse_args()


def main():
    args = parse_args()
    can_port = args.can or DEFAULT_CAN[args.side]
    wuji = not args.no_wuji
    urdf = REPO / URDF[(args.side, wuji)]

    print("=" * 68)
    print("  COMPLIANT / GRAVITY-COMPENSATION TEST")
    print("=" * 68)
    print(f"  side        : {args.side}")
    print(f"  CAN         : {can_port}")
    print(f"  firmware    : {args.firmware}")
    print(f"  hand fitted : {'Wuji' if wuji else 'none'}")
    print(f"  urdf        : {urdf.relative_to(REPO)}")
    print(f"  kp / kd     : {args.kp} / {args.kd}")
    print(f"  gc scale    : {args.scale}")
    print(f"  duration    : {args.duration} s")
    print("=" * 68)

    if not urdf.exists():
        sys.exit(f"URDF not found: {urdf}")

    if args.dry_run:
        print("\n--dry-run: no hardware touched. Drop the flag to run for real.")
        return 0

    print("\n  !! THIS WILL POWER THE ARM. It will FALL if unsupported. !!")
    print("     Start with the arm in a low, supported pose.")
    if not args.yes:
        if input("\n  Type 'go' to continue: ").strip().lower() != "go":
            print("  aborted.")
            return 1

    from nerolib import (
        ControllerConfig,
        ControlMode,
        FirmwareVersion,
        Gain,
        MoveMode,
        NeroController,
    )

    config = ControllerConfig()
    config.interface_name = can_port
    config.urdf_path = str(urdf)
    config.firmware_version = getattr(FirmwareVersion, args.firmware)
    config.gravity_compensation = True
    config.gravity_comp_scale = args.scale

    nero = NeroController(config)

    if not nero.start():
        sys.exit("controller failed to start")

    try:
        nero.set_mode(ControlMode.CAN_COMMAND, MoveMode.MIT)
        nero.enable_gravity_compensation(True)
        nero.set_gain(Gain([args.kp] * 7, [args.kd] * 7))

        time.sleep(0.5)  # let the first commands land before sampling
        start_state = nero.get_current_state()
        q0 = list(start_state.pos)

        print("\n  Compliant. Leave the arm ALONE to measure drift,")
        print("  then move it by hand to check it holds where you leave it.\n")
        print(f"  {'t':>5}  {'max drift':>10}   per-joint drift (rad)")

        period = 1.0 / args.rate
        t_end = time.time() + args.duration
        worst = 0.0
        while time.time() < t_end:
            st = nero.get_current_state()
            drift = [p - r for p, r in zip(st.pos, q0)]
            md = max(abs(d) for d in drift)
            worst = max(worst, md)
            remaining = t_end - time.time()
            print(f"  {args.duration - remaining:5.1f}  {md:10.4f}   "
                  + " ".join(f"{d:+.3f}" for d in drift))
            time.sleep(period)

        final = nero.get_current_state()
        drift = [p - r for p, r in zip(final.pos, q0)]

        print("\n" + "=" * 68)
        print("  RESULT")
        print("=" * 68)
        print("  start pose : " + " ".join(f"{v:+.3f}" for v in q0))
        print("  end pose   : " + " ".join(f"{v:+.3f}" for v in final.pos))
        print("  drift      : " + " ".join(f"{d:+.3f}" for d in drift))
        print("  torque     : " + " ".join(f"{t:+.2f}" for t in final.torque))
        print(f"\n  worst drift over the run: {worst:.4f} rad "
              f"({worst * 57.2958:.2f} deg)")

        if worst < 0.02:
            print("\n  PASS  -- holds position. Payload model looks right.")
        elif worst < 0.05:
            print("\n  MARGINAL -- some drift. Check the sign below and consider")
            print("           regenerating the URDF with a measured hand mass.")
        else:
            print("\n  FAIL  -- significant drift; the payload model is off.")

        if worst >= 0.02:
            j2 = drift[1] if len(drift) > 1 else 0.0
            direction = "sinking (mass UNDER-modelled)" if j2 < 0 else \
                        "rising (mass OVER-modelled)"
            print(f"           joint2 drift {j2:+.3f} rad -> {direction}")
            print("\n           Fix by regenerating with the real hand mass, e.g.:")
            print("             python3 scripts/make_wuji_urdf.py --hand-mass 0.62")
            print("           Prefer that over raising --scale: scale multiplies the")
            print("           WHOLE arm's torque, over-compensating the proximal joints.")

    finally:
        print("\n  stopping (damping mode -- the arm will lower slowly)...")
        nero.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
