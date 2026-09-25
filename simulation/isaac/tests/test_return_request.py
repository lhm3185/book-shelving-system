"""입력 "N번 서가에 K권" → TrayJob 재료 — 분류 코드 접두어로 서가를 정하는 규칙을 지킨다."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "scripts", "demo"))

from return_request import env_lines, job_from_request, shelf_prefixes  # noqa: E402

PRE = {"shelf_01": ["0", "1", "2", "3", "4"], "shelf_02": ["5", "6", "7", "8", "9"]}


def test_서가별_권수가_분류_코드로_바뀐다():
    books, rfids, codes = job_from_request([("shelf_02", 2), ("shelf_01", 2)], PRE)
    assert books == ["book_001", "book_002", "book_003", "book_004"]
    assert rfids == ["rfid_001", "rfid_002", "rfid_003", "rfid_004"]
    assert [c[0] for c in codes] == ["5", "5", "0", "0"]          # 접두어만 본다


def test_모르는_서가는_거절한다():
    with pytest.raises(ValueError):
        job_from_request([("shelf_09", 1)], PRE)
    with pytest.raises(ValueError):
        job_from_request([("shelf_01", 0)], PRE)


def test_실측_shelf_map_을_읽는다():
    pre = shelf_prefixes()
    assert set(pre) >= {"shelf_01", "shelf_02"}
    assert "5" in pre["shelf_02"] and "0" in pre["shelf_01"]


def test_env_줄은_full_cycle_이_받는_꼴이다():
    lines = env_lines(["book_001"], ["rfid_001"], ["500.0"])
    assert lines[0] == "SIM_BOOK_IDS=\"['book_001']\""
