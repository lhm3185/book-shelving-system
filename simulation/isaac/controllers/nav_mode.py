"""주행 모드 스위치 — 순간이동 주행기와 Nav2 가 **같은 평면 조인트를 건드리므로** 둘이 동시에 켜지면 안 된다.

2026-09-25 데스크탑 확인: 팀원 로봇 USD 의 Nav2BaseController(OmniGraph)는 바퀴가 아니라
`dummy_base_prismatic_x/y · revolute_z` 평면 조인트 3개에 속도를 주고, 우리 `navigation_executor`
는 루트 XForm 을 직접 쓴다. 둘 다 살아 있으면 서로 싸운다.

    SIM_NAV_MODE=teleport   (기본) navigation_executor 가 goto/patrol 을 받아 루트를 옮긴다. 지금까지의 동작
    SIM_NAV_MODE=nav2       navigation_executor 가 goto/patrol 을 **거절**한다(로그 + 실패 보고). 주행은 Nav2 →
                            OmniGraph 몫. 정밀 도킹(standoff·lateral·rotate_base, manipulation_executor)은 모드와
                            무관하게 그대로 — 그게 '도킹 층' 이다 (웹 클로드 v43 회신 §2)
"""
import os

NAV_MODES = ("teleport", "nav2")


def read_nav_mode(env=None):
    """환경에서 모드를 읽는다. 모르는 값이면 teleport 로 두고 이유를 돌려준다 → `(mode, note)`."""
    env = os.environ if env is None else env
    raw = str(env.get("SIM_NAV_MODE", "teleport")).strip().lower()
    if raw in NAV_MODES:
        return raw, ""
    return "teleport", f"SIM_NAV_MODE={raw!r} 는 모르는 값 — teleport 로 둔다 (가능: {NAV_MODES})"


def teleport_allowed(mode, kind):
    """이 모드에서 `kind`(goto·patrol·cancel…) 를 순간이동 주행기가 처리해도 되는가."""
    if kind in ("goto", "patrol"):
        return mode == "teleport"
    return True                         # cancel 등은 어느 모드에서든 받는다
