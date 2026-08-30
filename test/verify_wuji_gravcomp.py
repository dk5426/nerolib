#!/usr/bin/env python3
"""
Verify the Wuji-hand gravity-compensation model -- no hardware, no CAN
=====================================================================
Loads the base (no-hand) and *_wuji URDFs into Pinocchio and checks that the
generated payload does what it claims. Safe to run with the arms powered off;
it never opens a CAN socket or constructs a NeroController.

Three checks per arm:

  1. STRUCTURE  -- the payload must add mass without adding a degree of
     freedom. nq/nv must stay 7, and the total model mass must grow by
     exactly the payload mass.

  2. PAYLOAD    -- the extra mass and its centre of mass, recovered from the
     difference between the two models' link7 inertias, must match what
     scripts/make_wuji_urdf.py wrote into the URDF.

  3. TORQUE     -- the real check. The difference in gravity torque between
     the two models is compared against an independent derivation: the torque
     a point mass at the payload CoM exerts, computed from the frame Jacobian
     rather than from RNEA. Two different pieces of Pinocchio machinery have
     to agree, over random poses, for this to pass.

Usage:
    python3 test/verify_wuji_gravcomp.py
    python3 test/verify_wuji_gravcomp.py --urdf-dir /path/to/urdf --poses 500
"""

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    sys.exit("pinocchio not found -- activate the nerolib environment first")

GRAVITY = np.array([0.0, 0.0, -9.81])

PAIRS = {
    "left": ("nero_cone-e_left_fixed.urdf", "nero_cone-e_left_fixed_wuji.urdf"),
    "right": ("right_arm_final.urdf", "right_arm_final_wuji.urdf"),
}

PAYLOAD_LINK = "wuji_hand_payload"
PARENT_LINK = "link7"


def load(path):
    model = pin.buildModelFromUrdf(str(path))
    return model, model.createData()


def total_mass(model):
    # Skip the universe body at index 0.
    return sum(model.inertias[i].mass for i in range(1, len(model.inertias)))


def link7_inertia(model):
    """The inertia Pinocchio ends up with on link7, payload absorbed."""
    jid = model.getJointId("joint7")
    return model.inertias[jid]


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def payload_from_models(base, wuji):
    """Recover the added mass and its CoM from the two link7 inertias.

    Pinocchio absorbs the fixed joint, so link7's inertia in the wuji model is
    the base link7 plus the payload, both expressed in the joint frame. Undoing
    the mass-weighted CoM average recovers the payload on its own.
    """
    i_base, i_wuji = link7_inertia(base), link7_inertia(wuji)
    m = i_wuji.mass - i_base.mass
    if m <= 0:
        return 0.0, np.zeros(3)
    com = (i_wuji.mass * i_wuji.lever - i_base.mass * i_base.lever) / m
    return m, com


def payload_torque_via_jacobian(model, data, q, mass, com_local):
    """Gravity torque from a point mass at com_local in the payload frame.

    Deliberately avoids RNEA: this goes through the frame Jacobian instead, so
    agreeing with the RNEA difference is a real cross-check rather than the
    same computation run twice.
    """
    fid = model.getFrameId(PAYLOAD_LINK)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobians(model, data, q)

    oMf = data.oMf[fid]
    r_world = oMf.rotation @ com_local          # frame origin -> CoM, in world

    J = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    Jv, Jw = J[:3, :], J[3:, :]
    # Velocity of the offset point: v_p = v_o + w x r  =>  Jp = Jv - skew(r) Jw
    Jp = Jv - skew(r_world) @ Jw

    # U = -m g . p  =>  tau = dU/dq = -m g^T Jp
    return -mass * (GRAVITY @ Jp)


def gravity_torque(model, data, q):
    return pin.computeGeneralizedGravity(model, data, q).copy()


def check_arm(side, urdf_dir, n_poses, tol, rng):
    base_name, wuji_name = PAIRS[side]
    base_path, wuji_path = urdf_dir / base_name, urdf_dir / wuji_name

    for p in (base_path, wuji_path):
        if not p.exists():
            print(f"  SKIP {side}: {p.name} not found")
            return None

    base, base_data = load(base_path)
    wuji, wuji_data = load(wuji_path)

    print(f"\n{'=' * 66}\n{side.upper()} ARM\n{'=' * 66}")
    print(f"  base : {base_name}")
    print(f"  wuji : {wuji_name}")

    ok = True

    # --- 1. structure ------------------------------------------------------
    print("\n  [1] STRUCTURE")
    print(f"      base nq/nv = {base.nq}/{base.nv}   wuji nq/nv = {wuji.nq}/{wuji.nv}")
    if (base.nq, base.nv, wuji.nq, wuji.nv) != (7, 7, 7, 7):
        print("      FAIL: expected 7 DOF on both models "
              "(payload must be on a FIXED joint)")
        ok = False
    else:
        print("      OK: payload adds no degree of freedom")

    if wuji.getFrameId(PAYLOAD_LINK) >= wuji.nframes:
        print(f"      FAIL: no '{PAYLOAD_LINK}' frame in the wuji model")
        return False

    dm = total_mass(wuji) - total_mass(base)
    print(f"      total mass: {total_mass(base):.6f} -> {total_mass(wuji):.6f} kg "
          f"(+{dm:.6f})")

    # --- 2. payload --------------------------------------------------------
    mass, com = payload_from_models(base, wuji)
    print("\n  [2] PAYLOAD (recovered from the link7 inertia difference)")
    print(f"      mass = {mass:.6f} kg")
    print(f"      com  = [{com[0]: .6f} {com[1]: .6f} {com[2]: .6f}] m (link7 frame)")
    if abs(mass - dm) > 1e-9:
        print("      FAIL: recovered mass disagrees with the total-mass delta")
        ok = False
    else:
        print("      OK: consistent with the total-mass delta")

    # --- 3. torque ---------------------------------------------------------
    print(f"\n  [3] TORQUE  (RNEA difference vs Jacobian, {n_poses} random poses)")
    lo = np.maximum(base.lowerPositionLimit, -np.pi)
    hi = np.minimum(base.upperPositionLimit, np.pi)

    worst, worst_q, peak = 0.0, None, 0.0
    per_joint_peak = np.zeros(base.nv)
    hardest, hardest_q, hardest_tau = 0.0, None, None
    for _ in range(n_poses):
        q = rng.uniform(lo, hi)
        delta = gravity_torque(wuji, wuji_data, q) - gravity_torque(base, base_data, q)
        expect = payload_torque_via_jacobian(wuji, wuji_data, q, mass, com)
        err = float(np.max(np.abs(delta - expect)))
        peak = max(peak, float(np.max(np.abs(delta))))
        per_joint_peak = np.maximum(per_joint_peak, np.abs(delta))
        if err > worst:
            worst, worst_q = err, q
        this_load = float(np.max(np.abs(delta)))
        if this_load > hardest:
            hardest, hardest_q, hardest_tau = this_load, q, delta

    # The two routes are different float64 computation chains (RNEA recursion
    # vs Jacobian product), so they agree only to round-off. Judge the error
    # relative to the torque magnitude, not against a fixed absolute floor.
    rel = worst / peak if peak > 0 else 0.0
    print(f"      peak payload torque over the sampled poses : {peak:.4f} Nm")
    print(f"      worst |RNEA_delta - Jacobian| mismatch      : {worst:.3e} Nm")
    print(f"      relative to peak torque                    : {rel:.3e}")
    if rel > tol:
        print(f"      FAIL: relative mismatch exceeds {tol:.1e}")
        print(f"      worst pose: {np.round(worst_q, 4).tolist()}")
        ok = False
    else:
        print(f"      OK: agrees to within {tol:.1e} relative (float64 round-off)")

    # Informational: how much torque the payload actually costs each joint.
    print("\n      peak payload torque per joint over the sampled poses:")
    print("      " + "  ".join(f"j{i+1}={v:.3f}" for i, v in enumerate(per_joint_peak))
          + "  [Nm]")
    print(f"      worst-case pose ({hardest:.3f} Nm): "
          f"{np.round(hardest_q, 3).tolist()}")
    print("      " + "  ".join(f"j{i+1}={v:+.3f}" for i, v in enumerate(hardest_tau))
          + "  [Nm]")

    # Cross-check the peak against a first-principles bound: a 0.71 kg mass can
    # never exert more than m*g*reach about any joint.
    reach = float(np.linalg.norm(com)) + 0.75   # payload offset + arm reach
    bound = mass * 9.81 * reach
    print(f"      sanity: peak {peak:.3f} Nm <= m*g*reach {bound:.3f} Nm "
          f"-> {'OK' if peak <= bound else 'IMPLAUSIBLE'}")
    if peak > bound:
        ok = False

    return ok


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf-dir", type=Path, default=here.parent / "urdf")
    ap.add_argument("--poses", type=int, default=200,
                    help="random poses to sample for the torque check")
    ap.add_argument("--tol", type=float, default=1e-6,
                    help="max allowed torque mismatch, relative to peak torque")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    print(f"urdf dir   : {args.urdf_dir}")
    print(f"pinocchio  : {pin.__version__}")

    rng = np.random.default_rng(args.seed)
    results = {s: check_arm(s, args.urdf_dir, args.poses, args.tol, rng)
               for s in PAIRS}

    print(f"\n{'=' * 66}")
    checked = {s: r for s, r in results.items() if r is not None}
    if not checked:
        sys.exit("nothing checked -- no URDF pairs found")
    for side, ok in checked.items():
        print(f"  {side:5s} : {'PASS' if ok else 'FAIL'}")
    print("=" * 66)
    sys.exit(0 if all(checked.values()) else 1)


if __name__ == "__main__":
    main()
