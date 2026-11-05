"""SesAnt robot constants for mjlab: actuators, initial state, collisions, action scale.

Facts about the robot, with where they come from. Values marked (est.) are not measured on hardware.
Joint names and signs follow the CAD (sesant/cad/scripts/poses.py):
  <leg>_hip   yaw about Z, + = counter-clockwise seen from above      legs: FL FR BR BL (front = -X)
  <leg>_knee  about the leg's tangential axis, + = foot swings inward (RealAnt/firmware: + = outward)
  neck_pan, neck_tilt (+ = look down)
  arm_<l|r>_yaw (+ = outward), arm_<l|r>_pitch (+ = claw down), arm_<l|r>_jaw (+ = open; arm_<l|r>_jaw2 follows it
  through an equality constraint = the printed gear pair, no actuator)
"""

from __future__ import annotations

from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from .assets import SESANT_XML

LEGS: tuple[str, ...] = ("FL", "FR", "BR", "BL")
LEG_JOINTS: tuple[str, ...] = tuple(f"{k}_{j}" for k in LEGS for j in ("hip", "knee"))
FOOT_SITES: tuple[str, ...] = LEGS
FOOT_GEOMS: tuple[str, ...] = tuple(f"{k}_foot_collision" for k in LEGS)
BASE_BODY = "base"


def get_spec(asset: Path | None = None) -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(asset or SESANT_XML))


# ── Servos ─────────────────────────────────────────────────────────────────
# MG90S: ~2.2 kg*cm stall at 5-6 V -> 0.21 N*m (est.), ~0.1 s / 60 deg. A hobby servo is a stiff position loop
# that saturates at stall; kp 3 N*m/rad reaches the limit at 4 deg of error. Armature: rotor x ~1:300 gearbox (est.).
# SCS0009 (StackChan's neck servo): ~2.3 kg*cm (est.).
MG90S_EFFORT = 0.21
SCS0009_EFFORT = 0.22
STIFFNESS = 3.0
DAMPING = 0.03
ARMATURE = 0.001

LEG_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hip", ".*_knee"), stiffness=STIFFNESS, damping=DAMPING, effort_limit=MG90S_EFFORT, armature=ARMATURE)
NECK_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=("neck_.*",), stiffness=STIFFNESS, damping=DAMPING, effort_limit=SCS0009_EFFORT, armature=ARMATURE)
ARM_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=(r"arm_._yaw", r"arm_._pitch", r"arm_._jaw"), stiffness=STIFFNESS, damping=DAMPING, effort_limit=MG90S_EFFORT, armature=ARMATURE)

# ── Initial state: standing, knees vertical (RealAnt `standup`), head level, arms in the carry pose (level, claws closed) ──
STAND_Z = 0.0518   # base height standing, from the CAD (jumper_sim.py writes sesant_sim.json with the same number)
CARRY_PITCH = 0.0       # rad: arms level while walking (lifting them crowds the front knee servos)
INIT_STATE = EntityCfg.InitialStateCfg(pos=(0.0, 0.0, STAND_Z), joint_pos={r"(FL|FR|BR|BL)_.*": 0.0, r"neck_.*": 0.0, r"arm_._(yaw|jaw|jaw2)": 0.0, r"arm_._pitch": CARRY_PITCH}, joint_vel={".*": 0.0})

# ── Collisions: robot vs ground only (convex hulls overlap at the joints by construction) ───────────────
_foot = r"^(FL|FR|BR|BL)_foot_collision$"
GROUND_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(r".*_collision",),
    contype=1,
    conaffinity=0,
    condim={_foot: 3, ".*": 1},
    priority={_foot: 1, ".*": 0},
    friction={_foot: (0.8, 0.005, 0.0001)},
    solref=(0.01, 1.0),
)

ARTICULATION = EntityArticulationInfoCfg(
    actuators=(LEG_ACTUATOR_CFG, NECK_ACTUATOR_CFG, ARM_ACTUATOR_CFG),
    soft_joint_pos_limit_factor=0.9,
)

#: Leg action scale (rad per unit action). Not mjlab's 0.25*effort/stiffness rule: for a stiff, weak hobby
#: servo that would be 1 deg. 0.3 rad (17 deg) covers a walking stride; the knees' 50-65 deg poses are reached
#: through the default-offset + larger actions when needed.
ACTION_SCALE: dict[str, float] = {j: 0.3 for j in LEG_JOINTS}


def get_sesant_robot_cfg() -> EntityCfg:
    """A fresh SesAnt EntityCfg (new instance each call, so tasks can mutate it)."""
    return EntityCfg(init_state=INIT_STATE, collisions=(GROUND_ONLY_COLLISION,), spec_fn=get_spec, articulation=ARTICULATION)
