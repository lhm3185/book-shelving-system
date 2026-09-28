"""Ridgeback–Franka runtime profile.

Carter/M0609 alternatives are intentionally excluded from the production
runtime. Robot-specific names remain centralized here.
"""

import os


class RobotProfile:
    def __init__(
        self,
        *,
        name,
        root,
        base_link,
        arm_joints,
        grip_joints,
        grip_open,
        grip_close,
        ee_frame,
        hand_link,
        finger_links,
        vel_limit,
        lula,
        camera_prim,
        camera_offset,
        lidar_prim,
        deck_z,
        drive_stiffness=0.0,
        drive_damping=0.0,
        art_root="",
        tray_match_tol=0.03,
        ik_seed_limit=0.0,
    ):
        self.name = name
        self.root = root
        self.base_link = base_link
        self.arm_joints = arm_joints
        self.grip_joints = grip_joints
        self.grip_open = grip_open
        self.grip_close = grip_close
        self.ee_frame = ee_frame
        self.hand_link = hand_link
        self.finger_links = finger_links
        self.vel_limit = vel_limit
        self.lula = lula
        self.camera_prim = camera_prim
        self.camera_offset = camera_offset
        self.lidar_prim = lidar_prim
        self.deck_z = deck_z
        self.drive_stiffness = drive_stiffness
        self.drive_damping = drive_damping
        self._art_root = art_root
        self.tray_match_tol = tray_match_tol
        self.ik_seed_limit = ik_seed_limit

    @property
    def articulation_root(self):
        return f"{self.root}/{self._art_root}" if self._art_root else self.root

    @property
    def dof(self):
        return len(self.arm_joints)

    GRIP_MAX_M = 0.04

    def grip_targets(self, width_m):
        fraction = min(max(float(width_m) / self.GRIP_MAX_M, 0.0), 1.0)
        return [
            closed + (opened - closed) * fraction
            for closed, opened in zip(self.grip_close, self.grip_open)
        ]

    def grip_width(self, positions):
        closed, opened = self.grip_close[0], self.grip_open[0]
        if abs(opened - closed) < 1e-9:
            return 0.0
        return float(
            (positions[0] - closed)
            / (opened - closed)
            * self.GRIP_MAX_M
        )

    def check(self, stage):
        return [
            path
            for path in (self.root, f"{self.root}/{self.base_link}")
            if not stage.GetPrimAtPath(path).IsValid()
        ]


FRANKA = RobotProfile(
    name="franka",
    root="/World/ridgeback_franka",
    base_link="panda_link0",
    arm_joints=[f"panda_joint{i}" for i in range(1, 8)],
    grip_joints=["panda_finger_joint1", "panda_finger_joint2"],
    grip_open=[0.04, 0.04],
    grip_close=[0.0, 0.0],
    ee_frame="right_gripper",
    hand_link="panda_hand",
    finger_links=["panda_leftfinger", "panda_rightfinger"],
    vel_limit=[2.175] * 4 + [2.61] * 3,
    lula=("supported", "Franka"),
    camera_prim=(
        "panda_hand/rsd455/RSD455/"
        "Camera_OmniVision_OV9782_Color"
    ),
    camera_offset=(0.0, 0.0, 0.0),
    lidar_prim="base_link/lidar_link",
    deck_z=0.286,
)


def profile(name=None):
    key = (name or os.environ.get("ARM_ROBOT") or "franka").strip().lower()
    if key != "franka":
        raise ValueError(
            f"Unsupported robot '{key}'; this runtime only supports Ridgeback–Franka"
        )
    return FRANKA
