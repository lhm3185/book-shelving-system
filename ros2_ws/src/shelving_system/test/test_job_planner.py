"""Tests for the book shelving job planner."""

from unittest.mock import Mock

import pytest

from shelving_system.job_planner import (
    InvalidTrayJobError,
    JobPlanner,
    JobPlanningError,
)
from shelving_system.yaml_data_manager import (
    ShelfNotFoundError,
    YamlDataManager,
)


@pytest.fixture
def data_manager() -> Mock:
    """Create a mocked YAML data manager."""
    manager = Mock(spec=YamlDataManager)

    shelves = {
        "005.7": {
            "shelf_id": "shelf_01",
            "frame_id": "map",
            "observation_pose": {
                "position": {
                    "x": 3.0,
                    "y": 1.0,
                    "z": 0.0,
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0,
                },
            },
        },
        "813.6": {
            "shelf_id": "shelf_02",
            "frame_id": "map",
            "observation_pose": {
                "position": {
                    "x": 5.0,
                    "y": 1.0,
                    "z": 0.0,
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.0,
                    "w": 0.0,
                },
            },
        },
    }

    def find_shelf(classification_code: str) -> dict:
        if classification_code not in shelves:
            raise ShelfNotFoundError(
                f"No shelf for {classification_code}"
            )

        return shelves[classification_code]

    locations = {
        "home": {
            "frame_id": "map",
            "position": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0,
            },
        },
        "return_station": {
            "frame_id": "map",
            "position": {
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0,
            },
        },
    }

    manager.get_shelf_for_classification.side_effect = find_shelf
    manager.get_location_pose.side_effect = (
        lambda name: locations[name]
    )
    manager.get_book_profile.return_value = {
        "profile_name": "standard",
        "width": 0.18,
        "height": 0.24,
        "thickness": 0.035,
        "safety_margin": 0.01,
    }

    return manager


@pytest.fixture
def job_planner(data_manager: Mock) -> JobPlanner:
    """Create a job planner with a mocked data manager."""
    return JobPlanner(data_manager=data_manager)


def test_create_single_book_plan(
    job_planner: JobPlanner,
) -> None:
    """A valid single-book job should create one task."""
    plan = job_planner.create_plan(
        job_id="job_001",
        tray_id="tray_001",
        book_ids=["book_001"],
        rfid_tags=["rfid_001"],
        classification_codes=["005.7"],
    )

    assert plan.job_id == "job_001"
    assert plan.tray_id == "tray_001"
    assert len(plan.tasks) == 1

    task = plan.tasks[0]

    assert task.job_id == "job_001"
    assert task.tray_id == "tray_001"
    assert task.book_id == "book_001"
    assert task.rfid_tag == "rfid_001"
    assert task.classification_code == "005.7"
    assert task.shelf_id == "shelf_01"

    assert task.shelf_frame_id == "map"
    assert task.shelf_observation_pose["position"]["x"] == 3.0

    assert task.book_profile["profile_name"] == "standard"
    assert task.book_profile["thickness"] == pytest.approx(0.035)

    assert plan.home_pose["position"]["x"] == 0.0
    assert plan.return_station_pose["position"]["x"] == 1.0


def test_create_multiple_book_plan_preserves_order(
    job_planner: JobPlanner,
) -> None:
    """The plan should preserve the order of books in the tray."""
    plan = job_planner.create_plan(
        job_id="job_002",
        tray_id="tray_002",
        book_ids=[
            "book_001",
            "book_002",
        ],
        rfid_tags=[
            "rfid_001",
            "rfid_002",
        ],
        classification_codes=[
            "005.7",
            "813.6",
        ],
    )

    assert len(plan.tasks) == 2

    assert plan.tasks[0].book_id == "book_001"
    assert plan.tasks[0].shelf_id == "shelf_01"

    assert plan.tasks[1].book_id == "book_002"
    assert plan.tasks[1].shelf_id == "shelf_02"


def test_empty_book_list_is_rejected(
    job_planner: JobPlanner,
) -> None:
    """A tray job without books should be rejected."""
    with pytest.raises(
        InvalidTrayJobError,
        match="at least one book",
    ):
        job_planner.create_plan(
            job_id="job_003",
            tray_id="tray_003",
            book_ids=[],
            rfid_tags=[],
            classification_codes=[],
        )


@pytest.mark.parametrize(
    (
        "book_ids",
        "rfid_tags",
        "classification_codes",
    ),
    [
        (
            ["book_001"],
            [],
            ["005.7"],
        ),
        (
            ["book_001"],
            ["rfid_001"],
            [],
        ),
        (
            [
                "book_001",
                "book_002",
            ],
            ["rfid_001"],
            [
                "005.7",
                "813.6",
            ],
        ),
    ],
)
def test_mismatched_book_data_lengths_are_rejected(
    job_planner: JobPlanner,
    book_ids: list[str],
    rfid_tags: list[str],
    classification_codes: list[str],
) -> None:
    """Book, RFID, and classification lists must have equal lengths."""
    with pytest.raises(
        InvalidTrayJobError,
        match="same length",
    ):
        job_planner.create_plan(
            job_id="job_004",
            tray_id="tray_004",
            book_ids=book_ids,
            rfid_tags=rfid_tags,
            classification_codes=classification_codes,
        )


def test_duplicate_book_ids_are_rejected(
    job_planner: JobPlanner,
) -> None:
    """A tray job must not contain duplicate book IDs."""
    with pytest.raises(
        InvalidTrayJobError,
        match="duplicate book IDs",
    ):
        job_planner.create_plan(
            job_id="job_005",
            tray_id="tray_005",
            book_ids=[
                "book_001",
                "book_001",
            ],
            rfid_tags=[
                "rfid_001",
                "rfid_002",
            ],
            classification_codes=[
                "005.7",
                "813.6",
            ],
        )


@pytest.mark.parametrize(
    (
        "job_id",
        "tray_id",
    ),
    [
        (
            "",
            "tray_001",
        ),
        (
            "job_001",
            "",
        ),
        (
            "   ",
            "tray_001",
        ),
        (
            "job_001",
            "   ",
        ),
    ],
)
def test_empty_job_or_tray_id_is_rejected(
    job_planner: JobPlanner,
    job_id: str,
    tray_id: str,
) -> None:
    """Job and tray identifiers must not be empty."""
    with pytest.raises(InvalidTrayJobError):
        job_planner.create_plan(
            job_id=job_id,
            tray_id=tray_id,
            book_ids=["book_001"],
            rfid_tags=["rfid_001"],
            classification_codes=["005.7"],
        )


def test_unknown_classification_fails_planning(
    job_planner: JobPlanner,
) -> None:
    """A book without a target shelf should fail job planning."""
    with pytest.raises(
        JobPlanningError,
        match="book_001",
    ):
        job_planner.create_plan(
            job_id="job_006",
            tray_id="tray_006",
            book_ids=["book_001"],
            rfid_tags=["rfid_001"],
            classification_codes=["999.9"],
        )