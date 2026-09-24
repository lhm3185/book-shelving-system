"""
`manipulation.yaml` 이 **검증된 값을 실제로 담고 있는지** 본다.

2026-09-24 에 같은 사고가 **세 번** 났다. 밤새 숫자를 낸 설정이 실행 셸의
`MAN_EXTRA` 환경변수에만 있고 저장소에는 없었다:

    slot_x_snap_to_gap       코드 기본 false   (비전 x 흔들림 280 mm 를 잡는 장치)
    slot_y_from_shelf_front  코드 기본 false   (깊이 오차 42~263 mm 를 피하는 장치)
    scan_command             코드 기본 scan_sweep  (아래 판에서 404 로 죽는 경로)

저장소를 받은 사람은 **검증 안 된 설정으로 돌린다.** 그리고 그 사실을 아무도 모른다.

세 번 났으면 개별로 막을 일이 아니다. **목록 자체를 여기에 고정한다.** 누가 yaml
에서 한 줄을 빼면 이 시험이 죽는다. 값을 바꿔야 하면 **여기도 같이 바꾸게** 되고,
그러면 "왜 바꿨나" 가 커밋에 남는다.
"""

import os

import yaml

YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '..', 'config', 'manipulation.yaml')

#: 검증된 값. **코드 기본과 다른 것만** 여기 있다 — 같은 것은 적을 이유가 없다.
#: (파라미터 이름, 검증된 값, 코드 기본, 안 넣으면 무슨 일이 나는가)
VERIFIED = [
    ('slot_x_snap_to_gap', True, False,
     '검출 x 흔들림 280 mm 가 그대로 목표가 된다 — 옆 책을 파고든다'),
    ('slot_y_from_shelf_front', True, False,
     '빈칸 깊이 관측(+42~263 mm 뒤)이 그대로 삽입 깊이가 된다'),
    ('scan_command', 'scan_shelf', 'scan_sweep',
     '아래 판 스캔이 404 로 죽는다 (관절 6 여유 0.100 rad)'),
    ('enable_perception_bridge', True, False,
     '/detect_target_slot 액션 자체가 안 뜬다 — 빈칸 검출 경로가 사라진다'),
    ('align_base_before_work', True, False,
     '작업 전 차체 정렬을 안 한다 — 스캔이 서가 오른쪽 절반을 못 본다'),
]


def _params():
    with open(YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)['manipulation_node']['ros__parameters']


def test_every_verified_switch_is_in_the_file():
    """**하나라도 빠지면 그 판은 검증된 조합이 아니다**."""
    p = _params()
    missing = [(k, why) for k, _v, _d, why in VERIFIED if k not in p]
    assert not missing, f'yaml 에 없다 (없으면: {missing})'


def test_every_verified_switch_has_the_verified_value():
    p = _params()
    wrong = {k: (p[k], v) for k, v, _d, _w in VERIFIED if k in p and p[k] != v}
    assert not wrong, f'값이 다르다 (파일값, 검증값): {wrong}'


def test_the_list_only_holds_things_that_differ_from_the_code_default():
    """코드 기본과 같은 것을 여기 적으면 **목록이 늘어나기만 하고 뜻이 옅어진다**."""
    for k, v, d, _why in VERIFIED:
        assert v != d, f'{k} 는 코드 기본과 같다 — 목록에서 빼라'


def test_each_entry_says_what_breaks_without_it():
    """**이유 없는 항목은 다음 사람이 지운다.** 왜 필요한지가 같이 있어야 한다."""
    for k, _v, _d, why in VERIFIED:
        assert why and len(why) > 10, k


def test_the_executor_is_the_one_we_measured_with():
    """`mock` 으로 두면 Isaac 없이 돌지만, 산출물의 숫자는 `sim` 에서 나왔다."""
    assert _params()['executor'] == 'sim'
