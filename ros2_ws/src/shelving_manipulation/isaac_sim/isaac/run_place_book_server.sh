#!/usr/bin/env bash
# (호환용) 예전 실행 경로. 통합 실행기로 넘긴다 — 공식 명령은 scripts/run_isaac_sim.sh 다.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
echo "### 이 스크립트는 통합 실행기로 대체됐다: $REPO_ROOT/scripts/run_isaac_sim.sh" >&2
exec "$REPO_ROOT/scripts/run_isaac_sim.sh" "$@"
