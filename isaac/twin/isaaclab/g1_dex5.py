"""Isaac Lab config for the merged G1 + Dex5-1P twin asset (DT7-lite).

Describes `isaac/assets/g1_dex5/g1_dex5_1p.usd` -- the 69-DOF articulation built by
`tools/merge_dex5_urdf.py` -- as an Isaac Lab `ArticulationCfg`, plus a `CameraCfg`
for the D435i taken from `isaac/twin/g1_contract.yaml`.

NO ISAAC LAB IS REQUIRED TO IMPORT THIS FILE. The module is plain data structures;
`build_articulation_cfg()` and `build_camera_cfg()` lazy-import `isaaclab` only when
called. The tests in `tools/tests/test_isaaclab_cfg.py` therefore run anywhere, and
the numbers they guard are the same numbers Isaac Lab would receive.

WHAT THIS IS FOR
----------------
Training against the twin rather than against a generic G1. Body actuator groups
mirror `ferox-g1-locomotion/policy_g1_baseline/params/env.yaml`, which is the config
that produced the deployed policy, so a policy trained through this cfg sees the same
motor groups, gains and armature the current one did. The hands are added as their
own group -- the body groups are left untouched, so this cannot change body dynamics.

RULE-HAND-NAME (C-14) APPLIES HERE, AND IS WHY THE HAND GROUP IS EXPLICIT
------------------------------------------------------------------------
Isaac Lab resolves `joint_names_expr` with regular expressions against the
articulation's own DOF order, which for the hands is interleaved and non-contiguous
(left 29..63, right 34..68). The body groups below use regex because Unitree's body
names make that unambiguous (`.*_hip_pitch_.*` cannot match a finger). The hand group
does NOT: it lists all 40 joints by name, in URDF document order, per hand.

That is deliberate and slightly verbose. A pattern like `Pitch_.*` would silently
sweep in `Pitch_13L` through `Pitch_54R` across BOTH hands in whatever order the
articulation happens to have them, which is exactly the failure C-14 exists to stop.
"""
from __future__ import annotations

from typing import Any, Dict, List

# --------------------------------------------------------------------------- asset

USD_PATH = "isaac/assets/g1_dex5/g1_dex5_1p.usd"
ARTICULATION_ROOT = "/g1_29dof_rev_1_0/pelvis"
EXPECT_DOF = 69          # 29 body + 2 x 20 hand
EXPECT_BODIES = 79
EXPECT_TOTAL_MASS = 35.004757

# --------------------------------------------------------------------- body joints

# The 29 body joints in the SDK order the driver speaks, from
# ferox-g1-locomotion/policy_g1_baseline/params/env.yaml:joint_sdk_names.
BODY_JOINTS_SDK: List[str] = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

# The same 29 in the order the SIM articulation reports them. Isaac orders DOFs
# breadth-first over the tree, which is why this is interleaved rather than
# per-limb. Asserted bit-identical to the merged asset by the Isaac suite, and
# derived from BODY_JOINTS_SDK via deploy.yaml's joint_ids_map by the tests here.
BODY_JOINTS_SIM: List[str] = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint", "left_ankle_pitch_joint",
    "right_ankle_pitch_joint", "left_shoulder_roll_joint",
    "right_shoulder_roll_joint", "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_elbow_joint",
    "right_elbow_joint", "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]

# --------------------------------------------------------------------- hand joints

# Dex5-1P, URDF document order, per hand. This is limits.py's order: the flat
# 20-vector the driver clamps. Written out rather than generated so that a Unitree
# rename breaks the test instead of silently reordering a policy's action space.
#
# Note index 12: Roll_41R on the right, Link_41L on the LEFT. Unitree's own
# asymmetry, and the reason a name list is not merely defensive here.
HAND_JOINTS: Dict[str, List[str]] = {
    "left": [
        "Yaw_11L", "Roll_12L", "Pitch_13L", "Pitch_14L",
        "Roll_21L", "Pitch_22L", "Pitch_23L", "Pitch_24L",
        "Roll_31L", "Pitch_32L", "Pitch_33L", "Pitch_34L",
        "Link_41L", "Pitch_42L", "Pitch_43L", "Pitch_44L",
        "Roll_51L", "Pitch_52L", "Pitch_53L", "Pitch_54L",
    ],
    "right": [
        "Yaw_11R", "Roll_12R", "Pitch_13R", "Pitch_14R",
        "Roll_21R", "Pitch_22R", "Pitch_23R", "Pitch_24R",
        "Roll_31R", "Pitch_32R", "Pitch_33R", "Pitch_34R",
        "Roll_41R", "Pitch_42R", "Pitch_43R", "Pitch_44R",
        "Roll_51R", "Pitch_52R", "Pitch_53R", "Pitch_54R",
    ],
}
# Indices, in the lists above, of the four joints per hand that are not
# independently actuated. See C-13: held at zero, not mimic-coupled.
PASSIVE_INDICES = (4, 8, 12, 16)


def hand_joint_names(side: str | None = None) -> List[str]:
    """All 40 hand joints, or one hand's 20, by NAME. Never by slice (C-14)."""
    if side is not None:
        return list(HAND_JOINTS[side])
    return HAND_JOINTS["left"] + HAND_JOINTS["right"]


def active_hand_joint_names(side: str | None = None) -> List[str]:
    """The 16 actuated joints per hand -- the passive four removed BY NAME."""
    sides = [side] if side is not None else ["left", "right"]
    out: List[str] = []
    for s in sides:
        passive = {HAND_JOINTS[s][i] for i in PASSIVE_INDICES}
        out += [n for n in HAND_JOINTS[s] if n not in passive]
    return out


# ----------------------------------------------------------------------- actuators

# Body groups mirror env.yaml's four Unitree motor models exactly -- same
# joint_names_expr, same effort/velocity limits, same stiffness/damping maps, same
# armature. Copied so a policy trained through this cfg sees what the deployed one
# saw; the tests assert the gains still match deploy.yaml joint for joint.
ACTUATORS: Dict[str, Dict[str, Any]] = {
    "N7520-14.3": {
        "joint_names_expr": [".*_hip_pitch_.*", ".*_hip_yaw_.*", "waist_yaw_joint"],
        "effort_limit_sim": 88, "velocity_limit_sim": 32.0,
        "stiffness": {".*_hip_.*": 100.0, "waist_yaw_joint": 200.0},
        "damping": {".*_hip_.*": 2.0, "waist_yaw_joint": 5.0},
        "armature": 0.01,
    },
    "N7520-22.5": {
        "joint_names_expr": [".*_hip_roll_.*", ".*_knee_.*"],
        "effort_limit_sim": 139, "velocity_limit_sim": 20.0,
        "stiffness": {".*_hip_roll_.*": 100.0, ".*_knee_.*": 150.0},
        "damping": {".*_hip_roll_.*": 2.0, ".*_knee_.*": 4.0},
        "armature": 0.01,
    },
    "N5020-16": {
        "joint_names_expr": [".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_roll.*",
                             ".*_ankle_.*", "waist_roll_joint", "waist_pitch_joint"],
        "effort_limit_sim": 25, "velocity_limit_sim": 37,
        "stiffness": 40.0,
        "damping": {".*_shoulder_.*": 1.0, ".*_elbow_.*": 1.0,
                    ".*_wrist_roll.*": 1.0, ".*_ankle_.*": 2.0,
                    "waist_.*_joint": 5.0},
        "armature": 0.01,
    },
    "W4010-25": {
        "joint_names_expr": [".*_wrist_pitch.*", ".*_wrist_yaw.*"],
        "effort_limit_sim": 5, "velocity_limit_sim": 22,
        "stiffness": 40.0, "damping": 1.0, "armature": 0.01,
    },
    # The hands, as their OWN group. Explicit names, not a pattern -- RULE-HAND-NAME.
    # Gains are the conservative import defaults DT3 settled on (stiffness 20,
    # damping 2, no tuning loop); they hold every commanded pose to <=0.001 rad with
    # the root pinned. They have NOT been tested against a payload -- see OQ-3.3.
    "dex5_1p": {
        "joint_names_expr": HAND_JOINTS["left"] + HAND_JOINTS["right"],
        "effort_limit_sim": 0.93,     # Dex5-1 URDF effort, left file (see C-13 note)
        "velocity_limit_sim": 3.14,
        "stiffness": 20.0, "damping": 2.0, "armature": 0.001,
    },
}

# Standing pose, from env.yaml init_state.joint_pos. Hands open at zero.
INIT_JOINT_POS: Dict[str, float] = {
    "left_hip_pitch_joint": -0.1, "right_hip_pitch_joint": -0.1,
    ".*_knee_joint": 0.3, ".*_ankle_pitch_joint": -0.2,
    ".*_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.25, "right_shoulder_roll_joint": -0.25,
    ".*_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.15, "right_wrist_roll_joint": -0.15,
}
INIT_POS = (0.0, 0.0, 0.80)
SOFT_JOINT_POS_LIMIT_FACTOR = 0.9

# ------------------------------------------------------------------------- camera

# From isaac/twin/g1_contract.yaml. K is `assumed` (C-3) -- factory-typical D435i at
# 1280x720 -- and stays that way until a real camera_info capture lands. The G1
# ground-truth bag could not answer it: no camera container.
CAMERA = {
    "prim_path": "{ENV_REGEX_NS}/Robot/torso_link/camera_link/camera_color_frame"
                 "/camera_color_optical_frame",
    "width": 1280,
    "height": 720,
    "rate_hz": 30.0,
    "K": [908.0, 0.0, 640.0, 0.0, 908.0, 360.0, 0.0, 0.0, 1.0],
    "distortion_model": "plumb_bob",
    "optical_frame": "camera_color_optical_frame",
    "mount_link": "torso_link",
    # base_link -> camera_link, contract tf_static. NOT published by the driver by
    # default (camera_tf_enable false) -- the twin mirrors that behind CAMERA_TF=1.
    "mount_xyz": (0.0576235, 0.01753, 0.41987),
    "mount_rpy": (0.0, 0.8307767239493009, 0.0),
    "clipping_range": (0.28, 4.0),   # min_z / clip_distance from realsense params
    "data_types": ("rgb", "distance_to_image_plane"),
}


def focal_from_K(K, width: int, horizontal_aperture: float = 20.955):
    """USD focalLength that reproduces the contract's fx at a given aperture.

    Isaac Lab's CameraCfg takes focal length and aperture, not fx/fy. Solving from K
    rather than typing a focal length is what keeps this consistent with the running
    twin, which does the same in isaac/twin/sensors.py and asserts the read-back.
    """
    fx = K[0]
    return fx * horizontal_aperture / width


# -------------------------------------------------------------- lazy cfg builders


def build_articulation_cfg(usd_path: str | None = None):
    """The Isaac Lab ArticulationCfg. Imports isaaclab only when called."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    import isaaclab.sim as sim_utils

    actuators = {
        name: ImplicitActuatorCfg(**spec) for name, spec in ACTUATORS.items()
    }
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path or USD_PATH,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=1.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=INIT_POS, joint_pos=dict(INIT_JOINT_POS), joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=SOFT_JOINT_POS_LIMIT_FACTOR,
        actuators=actuators,
    )


def build_camera_cfg():
    """The Isaac Lab CameraCfg for the D435i colour stream."""
    from isaaclab.sensors import CameraCfg
    import isaaclab.sim as sim_utils

    aperture = 20.955
    return CameraCfg(
        prim_path=CAMERA["prim_path"],
        update_period=1.0 / CAMERA["rate_hz"],
        width=CAMERA["width"],
        height=CAMERA["height"],
        data_types=list(CAMERA["data_types"]),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=focal_from_K(CAMERA["K"], CAMERA["width"], aperture),
            horizontal_aperture=aperture,
            clipping_range=CAMERA["clipping_range"],
        ),
    )
