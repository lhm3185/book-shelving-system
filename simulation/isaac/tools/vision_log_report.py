#!/usr/bin/env python3
"""비전 로그에서 **트레이 책 검출이 무엇에게 밀리는지** 를 숫자로 뽑는다 (시뮬 없이, 읽기 전용).

왜 (2026-09-28 밤, 새 텍스처 레벨):
    트레이에 한 권만 남으면 비전의 "신뢰도 1등" 이 서가 책으로 넘어갔다. ROI y 상한을 0.42 → 0.40 으로
    내려 보는 중인데, **어느 축으로 가르는 것이 가장 넉넉한가** 는 재 본 적이 없다. 트레이 책과 끼어든
    책은 y 말고도 z · 화면 행(py) · 신뢰도에서 다를 수 있다. 문턱을 고르기 전에 두 무리의 분포를 본다.

읽는 줄 (`full_cycle.sh` 의 vision.log 또는 합친 판 로그):
    Book picked(best): xyz=(x, y, z), center=(a, b), conf=0.88, ... (후보 N권)
    ROI 밖 책: (x, y, z) @arm_base_link
    ROI 밖 책 N권 제외 (남은 M권). ROI [..] ~ [..] @arm_base_link
    책 좌표 정함 (...) / 책 관측이 안 정해진다 / 안전 범위 밖 / 파지 관측 거절      ← 조작 노드

내는 것:
    1. 뽑힌 검출을 y 로 묶은 무리별 통계 (개수 · 신뢰도 · 좌표 범위 · 화면 중심 범위 · ROI 면까지 여유)
    2. ROI 가 버린 검출의 무리별 통계와 **어느 축이 얼마 차이로 버렸는가**
    3. 시간순 1등의 흐름 (A×12 → B×3 → A×2) — 끼어든 무리가 언제 이겼는가
    4. 트레이 칸 y 를 주면(--slots) 무리마다 가장 가까운 칸과 거리
    5. 축별 분리 여유 — 트레이 무리와 그 밖 무리 사이가 가장 넓은 축

**보고 전용이다. 문턱을 정해 주지 않는다.** 모든 좌표는 팔 기준(arm_base_link), 단위 m.
"""
import argparse
import re
import sys
from statistics import median

_NUM = r'[-+−]?\d+(?:\.\d+)?'
_TIME = re.compile(r'(\d{2}:\d{2}:\d{2})')
_PICK = re.compile(
    r'Book picked\(best\):\s*xyz=\(\s*(%s),\s*(%s),\s*(%s)\),\s*center=\(\s*(%s),\s*(%s)\),'
    r'\s*conf=(%s)(?:.*?후보\s*(\d+)권)?' % ((_NUM,) * 6))
_DROP = re.compile(r'ROI 밖 책:\s*\(\s*(%s),\s*(%s),\s*(%s)\)' % ((_NUM,) * 3))
_ROI = re.compile(r'ROI\s*\[([^\]]+)\]\s*~\s*\[([^\]]+)\]')
_NODE = re.compile(r'(책 좌표 정함.*|책 관측이 안 정해진다.*|비전 책 좌표.*안전 범위 밖.*|파지 관측 거절.*)')

AXES = ('x', 'y', 'z')


def _f(text):
    return float(text.replace('−', '-'))


def parse_lines(lines):
    """로그 줄 → `{'picks': [...], 'drops': [...], 'roi': (lo, hi) | None, 'node': [...]}`."""
    picks, drops, node, roi = [], [], [], None
    for line in lines:
        tm = _TIME.search(line)
        when = tm.group(1) if tm else ''
        m = _PICK.search(line)
        if m:
            picks.append({
                'time': when, 'xyz': (_f(m.group(1)), _f(m.group(2)), _f(m.group(3))),
                'center': (_f(m.group(4)), _f(m.group(5))), 'conf': _f(m.group(6)),
                'candidates': int(m.group(7)) if m.group(7) else None})
            continue
        m = _DROP.search(line)
        if m:
            drops.append({'time': when, 'xyz': (_f(m.group(1)), _f(m.group(2)), _f(m.group(3)))})
            continue
        m = _ROI.search(line)
        if m:
            try:
                lo = tuple(_f(v) for v in m.group(1).split(','))
                hi = tuple(_f(v) for v in m.group(2).split(','))
                if len(lo) == 3 and len(hi) == 3:
                    roi = (lo, hi)
            except ValueError:
                pass
        m = _NODE.search(line)
        if m:
            node.append({'time': when, 'text': m.group(1).strip()})
    return {'picks': picks, 'drops': drops, 'roi': roi, 'node': node}


def group_by_y(items, y_group_m=0.04):
    """y 가 가까운 것끼리 묶는다 (조작 노드 `steady_book` 과 같은 자 — 첫 점 기준 ±y_group_m)."""
    groups = []
    for item in items:
        for g in groups:
            if abs(item['xyz'][1] - g[0]['xyz'][1]) <= y_group_m:
                g.append(item)
                break
        else:
            groups.append([item])
    return groups


def _span(values):
    return (min(values), max(values))


def roi_margin(xyz, roi):
    """점에서 ROI 여섯 면까지의 여유 중 **가장 작은 것**과 그 면. 밖이면 음수 (가장 크게 벗어난 면)."""
    lo, hi = roi
    worst, face = None, ''
    for i, axis in enumerate(AXES):
        for value, name in ((xyz[i] - lo[i], f'{axis} 하한'), (hi[i] - xyz[i], f'{axis} 상한')):
            if worst is None or value < worst:
                worst, face = value, name
    return worst, face


def summarize(group, roi=None, slots=None):
    """무리 하나의 통계."""
    out = {'n': len(group), 'first': group[0]['time'], 'last': group[-1]['time']}
    for i, axis in enumerate(AXES):
        vals = [p['xyz'][i] for p in group]
        out[axis] = _span(vals)
        out[axis + '_mid'] = median(vals)
    if 'conf' in group[0]:
        confs = [p['conf'] for p in group]
        out['conf'] = (min(confs), median(confs), max(confs))
        out['center_a'] = _span([p['center'][0] for p in group])
        out['center_b'] = _span([p['center'][1] for p in group])
    if roi is not None:
        margins = [roi_margin(p['xyz'], roi) for p in group]
        out['roi_margin'] = min(margins, key=lambda m: m[0])
    if slots:
        y_mid = out['y_mid']
        nearest = min(range(len(slots)), key=lambda k: abs(slots[k] - y_mid))
        out['slot'] = (nearest, y_mid - slots[nearest])
    return out


def pick_flow(picks, groups):
    """시간순으로 어느 무리가 1등이었는가 → `[(무리 번호, 연속 횟수, 시작 시각), ...]`."""
    owner = {}
    for index, g in enumerate(groups):
        for p in g:
            owner[id(p)] = index
    flow = []
    for p in picks:
        index = owner[id(p)]
        if flow and flow[-1][0] == index:
            flow[-1] = (index, flow[-1][1] + 1, flow[-1][2])
        else:
            flow.append((index, 1, p['time']))
    return flow


def separation(tray, others):
    """
    트레이 무리들과 그 밖 무리들 사이가 **축마다 얼마나 떨어져 있는가**.

    각 축에서 두 집합의 구간이 겹치면 0, 아니면 구간 사이 거리. 가장 넓은 축이 가르기 좋은 축이다.
    `center_a` · `center_b`(화면 좌표)와 신뢰도도 같은 식으로 본다 — 단위가 다르니 축끼리 크기를 견주지 말고
    **0 인가 아닌가** 와 그 값을 본다.
    """
    result = {}
    if not tray or not others:
        return result
    keys = list(AXES) + [k for k in ('center_a', 'center_b') if k in tray[0] and k in others[0]]
    for key in keys:
        t_lo = min(s[key][0] for s in tray)
        t_hi = max(s[key][1] for s in tray)
        o_lo = min(s[key][0] for s in others)
        o_hi = max(s[key][1] for s in others)
        if t_hi < o_lo:
            result[key] = (o_lo - t_hi, f'트레이 ≤ {t_hi:.3f} < {o_lo:.3f} ≤ 그 밖')
        elif o_hi < t_lo:
            result[key] = (t_lo - o_hi, f'그 밖 ≤ {o_hi:.3f} < {t_lo:.3f} ≤ 트레이')
        else:
            result[key] = (0.0, f'겹친다 (트레이 {t_lo:.3f}~{t_hi:.3f} · 그 밖 {o_lo:.3f}~{o_hi:.3f})')
    if 'conf' in tray[0] and 'conf' in others[0]:
        result['conf'] = (
            min(s['conf'][0] for s in tray) - max(s['conf'][2] for s in others),
            f'트레이 최저 {min(s["conf"][0] for s in tray):.2f} · 그 밖 최고 '
            f'{max(s["conf"][2] for s in others):.2f} (음수면 그 밖이 이긴 프레임이 있다)')
    return result


def is_tray(summary, slots, slot_tol):
    return bool(slots) and 'slot' in summary and abs(summary['slot'][1]) <= slot_tol


def _fmt_span(span, scale=1.0, digits=3):
    return f'{span[0] * scale:+.{digits}f} ~ {span[1] * scale:+.{digits}f}'


def report(parsed, y_group_m=0.04, slots=None, slot_tol=0.05, roi=None):
    roi = roi or parsed['roi']
    lines = []
    picks, drops = parsed['picks'], parsed['drops']
    lines.append(f'뽑힌 검출 {len(picks)}개 · ROI 가 버린 검출 {len(drops)}개 · '
                 f'ROI {"없음" if roi is None else f"{list(roi[0])} ~ {list(roi[1])}"} @arm_base_link')
    groups = group_by_y(picks, y_group_m)
    sums = [summarize(g, roi, slots) for g in groups]
    lines.append('')
    lines.append(f'[1] 뽑힌 검출 — y 로 묶은 무리 {len(groups)}개 (묶는 폭 ±{y_group_m * 1000:.0f} mm)')
    for index, s in enumerate(sums):
        tag = ''
        if 'slot' in s:
            tag = (f' · 칸 {s["slot"][0]} 에서 {s["slot"][1] * 1000:+.0f} mm'
                   f'{"" if is_tray(s, slots, slot_tol) else "  ← **트레이 칸이 아니다**"}')
        lines.append(f'  {chr(65 + index % 26)}  {s["n"]:3d}개  {s["first"]}~{s["last"]}  '
                     f'conf {s["conf"][0]:.2f}/{s["conf"][1]:.2f}/{s["conf"][2]:.2f}{tag}')
        lines.append(f'       x {_fmt_span(s["x"])} · y {_fmt_span(s["y"])} · z {_fmt_span(s["z"])} · '
                     f'화면 ({s["center_a"][0]:.0f}~{s["center_a"][1]:.0f}, '
                     f'{s["center_b"][0]:.0f}~{s["center_b"][1]:.0f})')
        if 'roi_margin' in s:
            lines.append(f'       ROI 면까지 최소 여유 {s["roi_margin"][0] * 1000:+.0f} mm ({s["roi_margin"][1]})')
    if picks:
        lines.append('')
        lines.append('[2] 시간순 1등의 흐름')
        flow = pick_flow(picks, groups)
        lines.append('  ' + ' → '.join(f'{chr(65 + i % 26)}×{n}({t})' for i, n, t in flow))
    dsums = []
    if drops:
        dgroups = group_by_y(drops, y_group_m)
        dsums = [summarize(g, roi, slots) for g in dgroups]
        lines.append('')
        lines.append(f'[3] ROI 가 버린 검출 — 무리 {len(dgroups)}개 (많은 순 8개)')
        for s in sorted(dsums, key=lambda v: -v['n'])[:8]:
            extra = ''
            if 'roi_margin' in s:
                extra = f' · ROI 에서 {-s["roi_margin"][0] * 1000:.0f} mm 밖 ({s["roi_margin"][1]})'
            lines.append(f'  {s["n"]:3d}개  x {_fmt_span(s["x"])} · y {_fmt_span(s["y"])} · '
                         f'z {_fmt_span(s["z"])}{extra}')
    if slots:
        tray = [s for s in sums if is_tray(s, slots, slot_tol)]
        others = [s for s in sums if not is_tray(s, slots, slot_tol)]
        lines.append('')
        lines.append(f'[4] 뽑힌 검출 중 트레이 무리 {len(tray)}개 · 그 밖 {len(others)}개 '
                     f'(칸 y {slots}, 허용 ±{slot_tol * 1000:.0f} mm)')
        sep = separation(tray, others)
        if sep:
            lines.append('[5] 축별 분리 여유 — 뽑힌 검출 안에서 트레이와 그 밖 사이')
            for key, (gap, text) in sep.items():
                unit = '' if key.startswith('center') or key == 'conf' else ' m'
                lines.append(f'  {key:9s} {gap:+.3f}{unit}  {text}')
        elif not others:
            lines.append('[5] 뽑힌 검출은 전부 트레이 무리다 — 끼어든 것이 없다')
        if tray and dsums:
            sep2 = separation(tray, dsums)
            lines.append('[5b] 트레이 무리와 **ROI 가 버린 검출** 사이 (ROI 를 옮기면 무엇이 들어오는가)')
            for key, (gap, text) in sep2.items():
                lines.append(f'  {key:9s} {gap:+.3f} m  {text}')
    if parsed['node']:
        lines.append('')
        lines.append(f'[6] 조작 노드의 판단 {len(parsed["node"])}줄 (마지막 12줄)')
        for entry in parsed['node'][-12:]:
            lines.append(f'  {entry["time"]} {entry["text"][:150]}')
    return '\n'.join(lines)


def _floats(text):
    return [float(v) for v in text.replace('[', '').replace(']', '').split(',') if v.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='vision.log 또는 합친 판 로그')
    ap.add_argument('--y-group', type=float, default=0.04, help='y 묶는 폭 (m). 조작 노드 기본과 같다')
    ap.add_argument('--slots', default='', help='트레이 칸 y 목록 (팔 기준, 쉼표). 예: -0.19,-0.006,0.18,0.364')
    ap.add_argument('--slot-tol', type=float, default=0.05, help='칸으로 볼 y 허용치 (m)')
    ap.add_argument('--roi-min', default='', help='ROI 하한 x,y,z — 로그에 ROI 줄이 없을 때')
    ap.add_argument('--roi-max', default='', help='ROI 상한 x,y,z')
    ap.add_argument('--since', default='', help='HH:MM:SS 이후만')
    ap.add_argument('--until', default='', help='HH:MM:SS 이전만')
    args = ap.parse_args(argv)
    with open(args.log, encoding='utf-8', errors='replace') as handle:
        lines = handle.readlines()
    if args.since or args.until:
        kept = []
        for line in lines:
            tm = _TIME.search(line)
            if not tm:
                continue
            if args.since and tm.group(1) < args.since:
                continue
            if args.until and tm.group(1) > args.until:
                continue
            kept.append(line)
        lines = kept
    parsed = parse_lines(lines)
    roi = None
    if args.roi_min and args.roi_max:
        roi = (tuple(_floats(args.roi_min)), tuple(_floats(args.roi_max)))
    print(report(parsed, args.y_group, _floats(args.slots) if args.slots else None, args.slot_tol, roi))
    return 0


if __name__ == '__main__':
    sys.exit(main())
