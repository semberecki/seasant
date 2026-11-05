"""Environment config for sesant.flat: mjlab's velocity skeleton (as its Go1 flat task), scaled to SesAnt.

What is SesAnt-specific, and why:
  * only the 8 leg actuators are in the action space; neck and arms stay at their defaults under their PD (arms in the carry pose: level, claws closed)
  * the actor gets no base linear velocity (no sensor) and no joint velocity (MG90S has no feedback; the firmware
    streams an open-loop position estimate, which is what `joint_pos` stands for on hardware)
  * speeds, foot heights, pushes and reset heights are scaled to a 0.47 kg robot standing 5 cm tall
  * the IMU sits on the base here; on the robot it is the CoreS3's, in the head: rotate it by the measured
    neck angles (meas fields pan/tilt) before feeding it to a policy
"""

from __future__ import annotations

import math
from pathlib import Path

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import TerminationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RingPatternCfg, TerrainHeightSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from ..common.constants import ACTION_SCALE, BASE_BODY, FOOT_GEOMS, FOOT_SITES, LEG_JOINTS, get_sesant_robot_cfg, get_spec

LIN_X = (-0.15, 0.25)     # m/s (est. from MG90S speed and a ~4 cm stride)
LIN_Y = (-0.10, 0.10)
ANG_Z = (-0.8, 0.8)       # rad/s
FOOT_HEIGHT = 0.015       # m swing target


def env_cfg(asset: Path | None = None, play: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()
    robot = get_sesant_robot_cfg()
    if asset is not None:
        robot.spec_fn = lambda: get_spec(asset)
    cfg.scene.entities = {"robot": robot}

    # flat ground: no terrain generator, no height scan
    cfg.sim.njmax = 300
    cfg.sim.mujoco.ccd_iterations = 50
    cfg.sim.contact_sensor_maxmatch = 64
    cfg.sim.nconmax = None
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None
    cfg.scene.sensors = tuple(s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan")
    for g in ("actor", "critic"):
        cfg.observations[g].terms.pop("height_scan", None)
    cfg.rewards["upright"].params.pop("terrain_sensor_names", None)
    cfg.curriculum.pop("terrain_levels", None)
    cfg.terminations.pop("out_of_terrain_bounds", None)
    cfg.terminations["fell_over"] = TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": math.radians(60.0)})

    for sensor in cfg.scene.sensors or ():
        if sensor.name == "foot_height_scan":
            assert isinstance(sensor, TerrainHeightSensorCfg)
            sensor.frame = tuple(ObjRef(type="site", name=s, entity="robot") for s in FOOT_SITES)
            sensor.pattern = RingPatternCfg.single_ring(radius=0.01, num_samples=4)
    cfg.scene.sensors = (cfg.scene.sensors or ()) + (
        ContactSensorCfg(
            name="feet_ground_contact",
            primary=ContactMatch(mode="geom", pattern=FOOT_GEOMS, entity="robot"),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found", "force"), reduce="netforce", num_slots=1, track_air_time=True),
    )

    # actions: legs only
    act = cfg.actions["joint_pos"]
    assert isinstance(act, JointPositionActionCfg)
    act.actuator_names = list(LEG_JOINTS)
    act.scale = dict(ACTION_SCALE)

    # observations: what the robot can measure
    legs = SceneEntityCfg("robot", joint_names=list(LEG_JOINTS))
    for g in cfg.observations.values():
        for name in ("joint_pos", "joint_vel"):
            if name in g.terms:
                g.terms[name].params = dict(g.terms[name].params or {}) | {"asset_cfg": legs}
    cfg.observations["actor"].terms.pop("base_lin_vel", None)
    cfg.observations["actor"].terms.pop("joint_vel", None)

    # commands, resets, pushes: robot scale
    tw = cfg.commands["twist"]
    assert isinstance(tw, UniformVelocityCommandCfg)
    tw.ranges.lin_vel_x, tw.ranges.lin_vel_y, tw.ranges.ang_vel_z = LIN_X, LIN_Y, ANG_Z
    tw.rel_standing_envs = 0.2
    cfg.curriculum["command_vel"].params["velocity_stages"] = [
        {"step": 0, "lin_vel_x": (-0.08, 0.12), "ang_vel_z": (-0.4, 0.4)},
        {"step": 2000 * 24, "lin_vel_x": LIN_X, "ang_vel_z": ANG_Z},
    ]
    cfg.events["reset_base"].params["pose_range"]["z"] = (0.002, 0.01)
    cfg.events["push_robot"].params["velocity_range"] = {"x": (-0.1, 0.1), "y": (-0.1, 0.1), "z": (-0.05, 0.05),
                                                         "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.5, 0.5)}
    cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
    cfg.events["base_com"].params["asset_cfg"].body_names = (BASE_BODY,)

    # rewards
    cfg.rewards["pose"].params["std_standing"] = {r".*_hip": 0.1, r".*_knee": 0.15}
    cfg.rewards["pose"].params["std_walking"] = {r".*_hip": 0.4, r".*_knee": 0.6}
    cfg.rewards["pose"].params["std_running"] = {r".*_hip": 0.4, r".*_knee": 0.6}
    cfg.rewards["pose"].params["asset_cfg"] = legs
    cfg.rewards["upright"].params["asset_cfg"].body_names = (BASE_BODY,)
    cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = (BASE_BODY,)
    for name in ("foot_clearance", "foot_slip"):
        cfg.rewards[name].params["asset_cfg"].site_names = FOOT_SITES
    cfg.rewards["foot_clearance"].params["target_height"] = FOOT_HEIGHT
    cfg.rewards["foot_swing_height"].params["target_height"] = FOOT_HEIGHT
    cfg.rewards["body_ang_vel"].weight = 0.0
    cfg.rewards["angular_momentum"].weight = 0.0
    cfg.rewards["air_time"].weight = 0.0

    cfg.viewer.body_name = BASE_BODY
    cfg.viewer.distance = 0.6
    cfg.viewer.elevation = -15.0

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}
    return cfg
