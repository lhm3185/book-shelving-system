"""로봇마다 다른 이름·치수를 **한 곳에** 모은다.

왜: 로봇이 ridgeback_franka → nova-carter + M0609(+RG2) 로 바뀐다. 관절 이름·프레임·그리퍼
    방식이 전부 달라, 코드 곳곳에 박아 두면 바꿀 때마다 빠뜨리는 곳이 생기고 되돌리기도 어렵다.
    **기존 경로(franka)는 검증이 끝났으므로 그대로 두고**, 새 로봇은 값만 채워 고른다.

고르는 법
    ARM_ROBOT=m0609 ./scripts/run_isaac_sim.sh --gui     # 환경변수
    profile("m0609")                                      # 코드에서 직접

M0609 값의 출처 (2026-09-19 실측 / `M0609_PORT_PLAN.md`)
    - 관절 이름·한계: GPU PC `~/Isaac_Sim_b-1/src_pra/M0609/` 의 descriptor·URDF
    - 그리퍼: **`finger_joint` 은 명령해도 0.0008 rad 밖에 안 움직인다** — 명령 대상이 아니다.
      실제로 도는 것은 양쪽 knuckle (0.256 / 0.300 rad). 폐루프 링크라 연동이 깔끔하지 않다.
      우리 파이프라인은 파지할 때 **고정 조인트로 책을 붙이므로** 그리퍼가 물리적으로 쥘 필요는 없다.
    - 손목 카메라: RG2 의 `angle_bracket` 에 RealSense D455 가 이미 붙어 있고,
      카메라 prim 이름이 기존과 같다(`Camera_OmniVision_OV9782_Color`). link_6 기준 오프셋 실측.

**경로는 2026-09-19 수령한 AMR 담당 에셋 기준으로 확정했다.**
레벨: `~/Desktop/Collected_ing_library_env_v5-firstFinal/ing_library_env_v5.usd`
(같은 이름의 바탕화면 단독 USD 에는 **로봇이 없다** — 참조가 안 풀린다. 자립본 폴더를 쓸 것)
"""
import os


class RobotProfile:
    def __init__(self, name, root, base_link, arm_joints, grip_joints, grip_open, grip_close,
                 ee_frame, hand_link, finger_links, vel_limit, lula, camera_prim, camera_offset,
                 deck_z, drive_stiffness=0.0, drive_damping=0.0, art_root="",
                 tray_match_tol=0.03):
        self.name = name
        self.root = root                  # articulation root prim
        self.base_link = base_link        # IK·좌표의 기준 링크
        self.arm_joints = arm_joints      # 팔 관절 이름 (순서 = 명령 순서)
        self.grip_joints = grip_joints    # **실제 명령을 받는** 그리퍼 관절
        self.grip_open = grip_open        # 열림 지령값 (grip_joints 와 같은 길이)
        self.grip_close = grip_close
        self.ee_frame = ee_frame          # Lula 엔드이펙터 프레임 이름
        self.hand_link = hand_link        # 파지 고정 조인트를 매다는 링크
        self.finger_links = finger_links  # 손끝 방향을 재는 두 링크 (없으면 빈 리스트)
        self.vel_limit = vel_limit
        self.lula = lula                  # ("supported", "Franka") 또는 ("files", descriptor, urdf)
        self.camera_prim = camera_prim
        self.camera_offset = camera_offset  # 손목 링크 기준 (x, y, z) m
        # 트레이가 놓이는 면의 **월드 높이**. 로봇마다 다르다 — 예전에는 코드에 0.286 이
        # 박혀 있어 새 로봇에서 트레이가 37 cm 아래에 놓였다 (2026-09-20).
        self.deck_z = deck_z
        # 실행 시점에 올릴 팔 구동 게인. 0 이면 손대지 않는다 (에셋 값을 그대로 쓴다).
        self.drive_stiffness = drive_stiffness
        self.drive_damping = drive_damping
        # **articulation root 가 로봇 루트와 다를 수 있다.** 받은 에셋은 chassis_link 가 root 라
        # 루트 prim 으로 SingleArticulation 을 만들면 관절 명령이 안 먹는다 (2026-09-20).
        self._art_root = art_root
        # 트레이 칸에서 책을 찾을 때 허용 거리. **파지는 찾은 책의 실제 중심으로 하므로**
        # 이 값은 정밀도가 아니라 "그 칸에 책이 있나" 를 보는 기준이다.
        self.tray_match_tol = tray_match_tol

    @property
    def articulation_root(self):
        return f"{self.root}/{self._art_root}" if self._art_root else self.root

    @property
    def dof(self):
        return len(self.arm_joints)

    # 상위 코드는 그리퍼를 **폭(m)** 으로 다룬다 (SetGripper). Franka 는 손가락 관절이 곧 폭이라
    # 그대로지만, RG2 는 **각도**이고 좌우 부호가 반대다. 변환을 여기서만 한다.
    GRIP_MAX_M = 0.04

    def grip_targets(self, width_m):
        """폭(m) → 각 그리퍼 관절의 지령값"""
        f = min(max(float(width_m) / self.GRIP_MAX_M, 0.0), 1.0)
        return [c + (o - c) * f for c, o in zip(self.grip_close, self.grip_open)]

    def grip_width(self, positions):
        """그리퍼 관절 현재값 → 폭(m). 대표 관절 하나로 역산한다"""
        c, o = self.grip_close[0], self.grip_open[0]
        if abs(o - c) < 1e-9:
            return 0.0
        return float((positions[0] - c) / (o - c) * self.GRIP_MAX_M)

    def check(self, stage):
        """에셋을 받았을 때 이 프로파일이 맞는지 확인한다. 빠진 prim 목록을 돌려준다."""
        missing = []
        for path in (self.root, f"{self.root}/{self.base_link}"):
            if not stage.GetPrimAtPath(path).IsValid():
                missing.append(path)
        return missing


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
    vel_limit=[2.175] * 4 + [2.61] * 3,          # URDF 실측
    lula=("supported", "Franka"),
    camera_prim="panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
    camera_offset=(0.0, 0.0, 0.0),               # 기존 경로는 카메라 오프셋을 따로 쓰지 않는다
    deck_z=0.286,                                # ridgeback 데크 윗면
    # Franka 는 에셋 게인 그대로 4/4 가 나왔다 — 건드리지 않는다
)

M0609 = RobotProfile(
    name="m0609",
    # **AMR 담당 에셋 기준** (2026-09-19 수령, Collected_ing_library_env_v5-firstFinal).
    # 로봇 prim 이름이 우리가 만들었던 `carter_m0609` 가 아니라 `Nova_Carter_ROS` 다.
    # 팔은 카터 하위가 아니라 **형제**로 들어가 있다 (chassis_link 옆 m0609).
    root="/World/Nova_Carter_ROS",
    base_link="m0609/base_link",
    arm_joints=[f"joint_{i}" for i in range(1, 7)],   # **6축**
    # finger_joint 는 명령해도 안 움직인다 (2026-09-19 실측 0.0008 rad). 양쪽 knuckle 로 대칭 명령한다
    grip_joints=["left_inner_knuckle_joint", "right_inner_knuckle_joint"],
    grip_open=[-0.60, +0.60],
    grip_close=[0.0, 0.0],
    ee_frame="link_6",
    hand_link="m0609/link_6",
    finger_links=["m0609/onrobot_rg2ft/left_inner_finger",
                  "m0609/onrobot_rg2ft/right_inner_finger"],
    vel_limit=[2.618, 2.618, 3.1416, 3.927, 3.927, 3.927],   # M0609 URDF
    # verify_carter_m0609.py 에서 IK 가 실제로 풀린 조합 (2026-09-18 확인)
    lula=("files", os.path.expanduser("~/Isaac_Sim_b-1/src_pra/M0609/descriptor/m0609_description.yaml"),
          os.path.expanduser("~/Isaac_Sim_b-1/src_pra/M0609/doosan-robot2/urdf/m0609.urdf")),
    camera_prim="m0609/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color",
    camera_offset=(0.0115, 0.0450, 0.0525),      # link_6 기준, 회전 X축 180° (2026-09-19 실측)
    deck_z=0.655,                                # 받침판(Cube) 윗면 — 팔 베이스와 같은 높이
    # 받은 에셋은 URDF 임포트 기본값(강성 40~1135)이라 위치 지령을 못 따라간다.
    # 1e5 면 어깨·팔꿈치가 처지고 1e6 은 불안정했다 → 1e7/1e5 (2026-09-18 실측)
    drive_stiffness=1.0e7,
    drive_damping=1.0e5,
    art_root="chassis_link",                     # 받은 에셋의 articulation root
    # 받침판 위 트레이에서 책이 칸 중심에서 3.5 cm 까지 벗어난다 (2026-09-20 실측).
    # 물리적으로 자리를 잡는 위치와 계산한 칸 중심이 약간 다르다.
    tray_match_tol=0.06,
)

_ALL = {p.name: p for p in (FRANKA, M0609)}


def profile(name=None):
    """이름으로 프로파일을 고른다. 안 주면 ARM_ROBOT 환경변수, 그것도 없으면 검증된 franka."""
    key = (name or os.environ.get("ARM_ROBOT") or "franka").strip().lower()
    if key not in _ALL:
        raise ValueError(f"모르는 로봇 '{key}' — 있는 것: {sorted(_ALL)}")
    return _ALL[key]
