#!/usr/bin/env python3
"""네 권 풀 사이클 판(`full_cycle.sh`)의 판정 — 사람이 로그를 읽지 않는다.

`judge_run.py` 는 한 권짜리 판(`sim.log`·`cyc.log`·`man.log`)을 읽는다. 풀 사이클 판은 파일 이름도 다르고
(`isaac.log`), 한 판에 배치가 여러 번이라 그 판정기로는 "code 를 못 읽었다" 가 나온다 (2026-09-29).

    python3 judge_cycle.py cli_exchange/handoff_0929/runs/0141_new_dsag
    python3 judge_cycle.py runs/* --books 4 --brief

읽는 것 (`isaac.log` 또는 `isaac.log.gz` 의 `### ` 줄):
    [꽂은 자세] 중심이 목표에서 -2.8 mm · … → 수평 기울기 +0.32°
    [겹침] 옆 책과 겹치지 않음 (그 판의 서가 책 43권과 대조) · 좌 +26.1 mm (…) · 우 +33.4 mm (…)
    작업 <job> SUCCEEDED {'phase': 'verify', … 'placement_verified': True, 'checks': {…}}
    작업 <job> FAILED {'error_code': 403, 'message': '…'}
    루트 출발 자리 (+4.9859, -5.6067) · 팔 베이스 출발 자리 (…) yaw +90.00°
    [주행] 도착 (+4.986, -5.607) · [주행] 회전 끝 — **실측** yaw +90.00°
    [스윙] d(reorient) 처짐 152 mm … **내려앉는다** · [스윙] **경고** …

판정:
    합격        배치가 기대한 권수만큼 · 전부 검증 통과 · 겹침 없음 · 실패 코드 없음 · 출발 자리로 복귀
    충돌 의심    합격인데 `[스윙] **경고**` 가 있다 (바닥선 아래로 지나갔거나 내려앉는 경로를 그대로 썼다)
    불합격      그 밖

**문턱을 여기서 정하지 않는다.** 배치 검증은 실행기의 `placement_verified` 를 그대로 읽고, 복귀만 자리 5 mm ·
각도 0.1° 로 본다(출발 자리와 같은가). 모든 좌표는 월드, 단위는 줄에 적힌 그대로다.
"""
import argparse
import glob
import gzip
import math
import os
import re
import sys

_NUM = r'[-+]?\d+(?:\.\d+)?'
_T = re.compile(r'(\d{2}:\d{2}:\d{2})')
_POSE = re.compile(r'\[꽂은 자세\] 중심이 목표에서 (%s) mm.*?수평 기울기 (%s)°' % (_NUM, _NUM))
_JAM = re.compile(r'\[겹침\] (.*)')
_JAM_LR = re.compile(r'좌 (%s) mm.*?우 (%s) mm' % (_NUM, _NUM))
_JAM_N = re.compile(r'서가 책 (\d+)권')
_DONE = re.compile(r"작업 (\S+) (SUCCEEDED|FAILED) (\{.*)")
_START = re.compile(r'루트 출발 자리 \((%s), (%s)\)' % (_NUM, _NUM))
_START_YAW = re.compile(r'팔 베이스 출발 자리 \([^)]*\) yaw (%s)°' % _NUM)
_ARRIVE = re.compile(r'\[주행\] 도착 \((%s), (%s)\)' % (_NUM, _NUM))
_YAW = re.compile(r'\[주행\] 회전 끝.*?실측\*{0,2} yaw (%s)°' % _NUM)
_SAG = re.compile(r'\[스윙\] d\(reorient\) 처짐 (%s) mm' % _NUM)


def read_lines(run_dir):
    """판 폴더에서 실행기 로그 줄을 읽는다. 없으면 빈 목록."""
    for name in ('isaac.log', 'isaac.log.gz', 'sim.log'):
        path = os.path.join(run_dir, name)
        if not os.path.exists(path):
            continue
        opener = gzip.open if name.endswith('.gz') else open
        with opener(path, 'rt', encoding='utf-8', errors='replace') as handle:
            return [ln.rstrip('\n') for ln in handle]
    return []


def _when(line):
    """줄 앞머리의 시각(UTC, 실행기 로그). 본문에 섞인 시각은 보지 않는다."""
    m = _T.search(line[:40])
    return m.group(1) if m else ''


def _secs(hms):
    h, m, s = (int(v) for v in hms.split(':'))
    return h * 3600 + m * 60 + s


def parse(lines):
    """로그 줄 → 판의 사실들."""
    out = {'places': [], 'fails': [], 'start': None, 'start_yaw': None, 'arrive': None, 'yaw': None,
           'sags': [], 'warnings': [], 'scans': [], 'ready': False}
    pose = jam = None
    for raw in lines:
        if '### ' not in raw:
            continue
        line = raw.split('### ', 1)[1]
        when = _when(raw)
        if '준비 완료' in line:
            out['ready'] = True
        m = _POSE.search(line)
        if m:
            pose = {'time': when, 'center_mm': float(m.group(1)), 'skew_deg': float(m.group(2))}
            continue
        m = _JAM.search(line)
        if m:
            text = m.group(1)
            lr = _JAM_LR.search(text)
            n = _JAM_N.search(text)
            jam = {'clear': '겹치지 않음' in text,
                   'left_mm': float(lr.group(1)) if lr else None,
                   'right_mm': float(lr.group(2)) if lr else None,
                   'against': int(n.group(1)) if n else None, 'text': text[:120]}
            continue
        m = _DONE.search(line)
        if m:
            body = m.group(3)
            if "'phase': 'verify'" in body or "'placement_verified'" in body and m.group(2) == 'FAILED':
                code = re.search(r"'error_code': (\d+)", body)
                msg = re.search(r"'message': '([^']*)'", body)
                checks = dict(re.findall(r"'(upright|depth|spine|x|floor|skew|no_jam)': (True|False)", body))
                entry = {'time': when, 'job': m.group(1), 'status': m.group(2),
                         'verified': "'placement_verified': True" in body,
                         'checks_bad': sorted(k for k, v in checks.items() if v != 'True'),
                         'code': int(code.group(1)) if code else 0,
                         'message': msg.group(1) if msg else '', 'pose': pose, 'jam': jam}
                if pose is not None:
                    out['places'].append(entry)
                if m.group(2) == 'FAILED':
                    out['fails'].append(entry)
                pose = jam = None
            elif m.group(2) == 'FAILED':
                code = re.search(r"'error_code': (\d+)", body)
                msg = re.search(r"'message': '([^']*)'", body)
                out['fails'].append({'time': when, 'job': m.group(1), 'status': 'FAILED', 'verified': False,
                                     'code': int(code.group(1)) if code else 0,
                                     'message': msg.group(1) if msg else '', 'pose': None, 'jam': None,
                                     'checks_bad': []})
            elif "'phase': 'scan_done'" in body:
                dur = re.search(r'\((%s)s\)\s*$' % _NUM, line)
                out['scans'].append({'time': when, 'seconds': float(dur.group(1)) if dur else None})
            continue
        m = _START.search(line)
        if m and out['start'] is None:
            out['start'] = (float(m.group(1)), float(m.group(2)))
        m = _START_YAW.search(line)
        if m and out['start_yaw'] is None:
            out['start_yaw'] = float(m.group(1))
        m = _ARRIVE.search(line)
        if m:
            out['arrive'] = (float(m.group(1)), float(m.group(2)))
        m = _YAW.search(line)
        if m:
            out['yaw'] = float(m.group(1))
        m = _SAG.search(line)
        if m:
            out['sags'].append({'time': when, 'sag_mm': float(m.group(1)), 'flag': '내려앉는다' in line,
                                'remedy': ''})
        if out['sags'] and not out['sags'][-1]['remedy']:
            if "[스윙] d 를 씨앗" in line:
                out['sags'][-1]['remedy'] = '앞 권의 관절값을 씨앗으로 다시 풀었다'
            elif '[스윙] 바깥으로 뻗어' in line and 'd 처짐' in line:
                out['sags'][-1]['remedy'] = '되돌림 경로로 바꿨다'
            elif '조각으로 쓴다' in line:
                out['sags'][-1]['remedy'] = '경유점으로 나눴다'
        if '[스윙] **경고**' in line:
            out['warnings'].append({'time': when, 'text': line[:160]})
    return out


def verdict(facts, books=4, home_tol_m=0.005, yaw_tol_deg=0.1):
    """(판정, 이유 목록). 판정은 '합격' · '충돌 의심' · '불합격'."""
    why = []
    places = facts['places']
    good = [p for p in places if p['status'] == 'SUCCEEDED' and p['verified']]
    if len(good) < books:
        why.append(f'배치 {len(good)}/{books}')
    for p in places:
        if p['checks_bad']:
            why.append(f'{p["time"]} 검사 불통과 {p["checks_bad"]}')
        if p['jam'] is not None and not p['jam']['clear']:
            why.append(f'{p["time"]} 겹침 — {p["jam"]["text"]}')
    for f in facts['fails']:
        why.append(f'{f["time"]} 실패 {f["code"]} {f["message"][:60]}')
    if facts['start'] is None or facts['arrive'] is None:
        why.append('출발 또는 도착 자리를 못 읽었다')
    else:
        d = math.hypot(facts['arrive'][0] - facts['start'][0], facts['arrive'][1] - facts['start'][1])
        if d > home_tol_m:
            why.append(f'복귀 자리가 출발에서 {d * 1000:.1f} mm')
    if facts['start_yaw'] is not None and facts['yaw'] is not None:
        if abs(facts['yaw'] - facts['start_yaw']) > yaw_tol_deg:
            why.append(f'복귀 yaw {facts["yaw"]:+.2f}° (출발 {facts["start_yaw"]:+.2f}°)')
    elif facts['yaw'] is None:
        why.append('복귀 yaw 를 못 읽었다')
    if why:
        return '불합격', why
    unfixed = [s for s in facts['sags'] if s['flag'] and not s['remedy']]
    if unfixed:
        return '충돌 의심', [f'{s["time"]} d(reorient) 가 {s["sag_mm"]:.0f} mm 내려앉은 채 갔다' for s in unfixed]
    if facts['warnings']:
        return '충돌 의심', [w['time'] + ' ' + w['text'] for w in facts['warnings']]
    return '합격', []


def table(facts):
    rows = []
    for i, p in enumerate(facts['places'], 1):
        pose, jam = p['pose'] or {}, p['jam'] or {}
        left, right, against = jam.get('left_mm'), jam.get('right_mm'), jam.get('against')
        side = '' if left is None else f' (좌 {left:+.1f} / 우 {right:+.1f} mm'
        side += '' if against is None or left is None else f', 서가 책 {against}권'
        side += ')' if left is not None else ''
        ok = p['verified'] and p['status'] == 'SUCCEEDED'
        tail = '검증 통과' if ok else f'**{p["code"]} {p["message"][:40]}**'
        if p['status'] == 'FAILED' and p['verified']:
            tail = f'꽂기는 통과, 그 뒤 **{p["code"]} {p["message"][:40]}**'
        rows.append(
            f'  {i}권  {p["time"]}  중심 {pose.get("center_mm", float("nan")):+.1f} mm · 기울기 '
            f'{pose.get("skew_deg", float("nan")):+.2f}° · '
            f'{"겹침 없음" if jam.get("clear") else "**겹침**"}{side} · {tail}')
    return rows


def report(run_dir, books=4, brief=False):
    lines = read_lines(run_dir)
    name = os.path.basename(os.path.normpath(run_dir))
    if not lines:
        return f'불합격  {name}    실행기 로그가 없다 (isaac.log / isaac.log.gz)', '불합격'
    facts = parse(lines)
    result, why = verdict(facts, books)
    places = facts['places']
    centers = [p['pose']['center_mm'] for p in places if p['pose']]
    skews = [p['pose']['skew_deg'] for p in places if p['pose']]
    span = ''
    if len(places) >= 2:
        span = f' · 첫 배치 → 마지막 배치 {_secs(places[-1]["time"]) - _secs(places[0]["time"])} s'
    head = (f'{result}  {name}    배치 '
            f'{len([p for p in places if p["verified"] and p["status"] == "SUCCEEDED"])}/{books}'
            + (f' · 중심 {min(centers):+.1f} ~ {max(centers):+.1f} mm · 기울기 {min(skews):+.2f} ~ {max(skews):+.2f}°'
               if centers else '') + span)
    if brief:
        return head + (f'    ← {why[0]}' if why else ''), result
    out = [head]
    out.extend(table(facts))
    if facts['start'] and facts['arrive']:
        out.append(f'  복귀  출발 ({facts["start"][0]:+.4f}, {facts["start"][1]:+.4f}) → 도착 '
                   f'({facts["arrive"][0]:+.3f}, {facts["arrive"][1]:+.3f})'
                   + ('' if facts['yaw'] is None else f' · yaw {facts["yaw"]:+.2f}°'))
    sweeps = [s for s in facts['scans'] if s['seconds'] is not None and s['seconds'] >= 15.0]
    if facts['scans']:
        out.append(f'  스캔  {len(facts["scans"])}회 중 팔 스윕(15 s 이상) {len(sweeps)}회 — '
                   f'{[s["seconds"] for s in facts["scans"]]} s')
    for s in facts['sags']:
        note = ''
        if s['flag']:
            note = f'  → {s["remedy"]}' if s['remedy'] else '  ← **내려앉은 채 갔다**'
        out.append(f'  처짐  {s["time"]}  d(reorient) {s["sag_mm"]:.0f} mm{note}')
    for w in why:
        out.append(f'    ✘ {w}')
    return '\n'.join(out), result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('--books', type=int, default=4, help='기대하는 배치 권수')
    ap.add_argument('--brief', action='store_true', help='한 줄씩만')
    args = ap.parse_args(argv)
    dirs = [d for pat in args.dirs for d in sorted(glob.glob(pat)) if os.path.isdir(d)]
    if not dirs:
        print('판 폴더가 없다')
        return 2
    bad = 0
    for d in dirs:
        text, result = report(d, args.books, args.brief)
        print(text)
        bad += result == '불합격'
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
