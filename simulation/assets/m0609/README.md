# M0609 운동학 파일 (Lula IK 용)

로봇팔이 IK 를 풀 때 **이 두 파일이 없으면 팔이 아예 안 움직인다.**
Franka 는 Isaac 내장 설정(`lula=("supported", "Franka")`)을 쓰지만 M0609 는 외부 파일이다.
**USD 변환본으로 대체되지 않는다** — Lula 는 URDF 와 자체 기술서를 읽지 USD 를 안 본다.

| 파일 | 출처 | 라이선스 |
| --- | --- | --- |
| `urdf/m0609.urdf` | [doosan-robotics/doosan-robot2](https://github.com/doosan-robotics/doosan-robot2) | **BSD 3-Clause** (`urdf/LICENSE.doosan-robot2`) |
| `descriptor/m0609_description.yaml` | 우리가 위 URDF 에서 생성 | 이 저장소와 같음 |

## 왜 저장소에 넣었나

2026-09-20 까지 이 파일들은 **연구실 GPU PC 의 `~/Isaac_Sim_b-1/` 에만** 있었다.
집 데스크탑에 두 번째 작업 환경을 세울 때 이것 때문에 막혔고,
USB 백업 11 GB 전체를 뒤져도 없었다. **작은 파일 때문에 PC 를 바꿀 때마다 막히는 건 낭비다.**

## 쓰는 쪽

`simulation/isaac/config/robot_profiles.py` 의 `M0609.lula` 가 경로를 들고 있다.
기본값은 **저장소 사본**이고, 예전 경로(`~/Isaac_Sim_b-1/...`)가 있으면 그쪽도 쓴다.

## descriptor 에 대해

`m0609_description.yaml` 은 Lula 가 읽는 로봇 기술서다 — cspace(관절 목록), 관절 한계,
자기충돌 판정용 근사 구(sphere)가 들어 있다. 파일 머리말에 적힌 대로 **URDF 의 관절·링크
이름과 한계에서 유도**했고, 충돌 구는 근사값이다.

URDF 를 바꾸면 **이 파일도 같이 봐야 한다.** 둘이 어긋나면 IK 가 조용히 이상한 해를 낸다.
