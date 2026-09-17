"""Job planning logic for the book shelving system."""

from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Sequence

from shelving_system.yaml_data_manager import (
    ShelfNotFoundError,
    YamlDataManager,
)


class InvalidTrayJobError(ValueError):
    """Raised when received tray-job data is invalid."""


class JobPlanningError(RuntimeError):
    """Raised when a valid tray job cannot be planned."""


@dataclass(frozen=True)
class BookTaskPlan:
    """Plan for placing one book from a tray."""

    job_id: str
    tray_id: str
    book_id: str
    rfid_tag: str
    classification_code: str

    shelf_id: str
    shelf_frame_id: str
    shelf_observation_pose: dict[str, Any]

    book_profile: dict[str, Any]


@dataclass(frozen=True)
class JobPlan:
    """Complete plan for processing one returned-book tray."""

    job_id: str
    tray_id: str

    tasks: tuple[BookTaskPlan, ...]

    home_pose: dict[str, Any]
    return_station_pose: dict[str, Any]


class JobPlanner:
    """Create per-book tasks from one returned tray job."""

    def __init__(
        self,
        data_manager: YamlDataManager,
    ) -> None:
        """Initialize the planner with a YAML data manager."""
        self._data_manager = data_manager

    def create_plan(
        self,
        job_id: str,
        tray_id: str,
        book_ids: Sequence[str],
        rfid_tags: Sequence[str],
        classification_codes: Sequence[str],
    ) -> JobPlan:
        """Validate tray data and create a complete job plan."""
        normalized_job_id = self._validate_identifier(
            value=job_id,
            field_name="job_id",
        )
        normalized_tray_id = self._validate_identifier(
            value=tray_id,
            field_name="tray_id",
        )

        normalized_book_ids = self._validate_string_sequence(
            values=book_ids,
            field_name="book_ids",
        )
        normalized_rfid_tags = self._validate_string_sequence(
            values=rfid_tags,
            field_name="rfid_tags",
        )
        normalized_classification_codes = (
            self._validate_string_sequence(
                values=classification_codes,
                field_name="classification_codes",
            )
        )

        self._validate_list_lengths(
            book_ids=normalized_book_ids,
            rfid_tags=normalized_rfid_tags,
            classification_codes=(
                normalized_classification_codes
            ),
        )

        self._validate_unique_book_ids(
            normalized_book_ids
        )

        home_pose = self._data_manager.get_location_pose(
            "home"
        )
        return_station_pose = (
            self._data_manager.get_location_pose(
                "return_station"
            )
        )

        book_profile = self._data_manager.get_book_profile()

        tasks: list[BookTaskPlan] = []

        for (
            book_id,
            rfid_tag,
            classification_code,
        ) in zip(
            normalized_book_ids,
            normalized_rfid_tags,
            normalized_classification_codes,
            strict=True,
        ):
            task = self._create_book_task(
                job_id=normalized_job_id,
                tray_id=normalized_tray_id,
                book_id=book_id,
                rfid_tag=rfid_tag,
                classification_code=classification_code,
                book_profile=book_profile,
            )

            tasks.append(task)

        return JobPlan(
            job_id=normalized_job_id,
            tray_id=normalized_tray_id,
            tasks=tuple(tasks),
            home_pose=deepcopy(home_pose),
            return_station_pose=deepcopy(
                return_station_pose
            ),
        )

    def _create_book_task(
        self,
        job_id: str,
        tray_id: str,
        book_id: str,
        rfid_tag: str,
        classification_code: str,
        book_profile: dict[str, Any],
    ) -> BookTaskPlan:
        """Create a placement task for one book."""
        try:
            shelf = (
                self._data_manager
                .get_shelf_for_classification(
                    classification_code
                )
            )
        except ShelfNotFoundError as error:
            raise JobPlanningError(
                f"Failed to plan book '{book_id}': "
                f"no shelf for classification "
                f"'{classification_code}'."
            ) from error

        shelf_id = shelf.get("shelf_id")
        shelf_frame_id = shelf.get("frame_id")
        observation_pose = shelf.get(
            "observation_pose"
        )

        if (
            not isinstance(shelf_id, str)
            or not shelf_id.strip()
        ):
            raise JobPlanningError(
                f"Failed to plan book '{book_id}': "
                "shelf_id is missing."
            )

        if (
            not isinstance(shelf_frame_id, str)
            or not shelf_frame_id.strip()
        ):
            raise JobPlanningError(
                f"Failed to plan book '{book_id}': "
                "shelf frame_id is missing."
            )

        if not isinstance(observation_pose, dict):
            raise JobPlanningError(
                f"Failed to plan book '{book_id}': "
                "shelf observation_pose is missing."
            )

        return BookTaskPlan(
            job_id=job_id,
            tray_id=tray_id,
            book_id=book_id,
            rfid_tag=rfid_tag,
            classification_code=(
                classification_code
            ),
            shelf_id=shelf_id,
            shelf_frame_id=shelf_frame_id,
            shelf_observation_pose=deepcopy(
                observation_pose
            ),
            book_profile=deepcopy(book_profile),
        )

    @staticmethod
    def _validate_identifier(
        value: str,
        field_name: str,
    ) -> str:
        """Validate and normalize a required identifier."""
        if not isinstance(value, str):
            raise InvalidTrayJobError(
                f"{field_name} must be a string."
            )

        normalized_value = value.strip()

        if not normalized_value:
            raise InvalidTrayJobError(
                f"{field_name} must not be empty."
            )

        return normalized_value

    @classmethod
    def _validate_string_sequence(
        cls,
        values: Sequence[str],
        field_name: str,
    ) -> list[str]:
        """Validate a sequence containing required strings."""
        if isinstance(values, (str, bytes)):
            raise InvalidTrayJobError(
                f"{field_name} must be a sequence "
                "of strings."
            )

        if not isinstance(values, Sequence):
            raise InvalidTrayJobError(
                f"{field_name} must be a sequence."
            )

        normalized_values = [
            cls._validate_identifier(
                value=value,
                field_name=f"{field_name}[{index}]",
            )
            for index, value in enumerate(values)
        ]

        return normalized_values

    @staticmethod
    def _validate_list_lengths(
        book_ids: Sequence[str],
        rfid_tags: Sequence[str],
        classification_codes: Sequence[str],
    ) -> None:
        """Validate tray-list lengths and non-empty content."""
        if not book_ids:
            raise InvalidTrayJobError(
                "A tray job must contain at least one book."
            )

        if not (
            len(book_ids)
            == len(rfid_tags)
            == len(classification_codes)
        ):
            raise InvalidTrayJobError(
                "book_ids, rfid_tags, and "
                "classification_codes must have "
                "the same length."
            )

    @staticmethod
    def _validate_unique_book_ids(
        book_ids: Sequence[str],
    ) -> None:
        """Reject duplicate book identifiers."""
        if len(set(book_ids)) != len(book_ids):
            raise InvalidTrayJobError(
                "A tray job contains duplicate book IDs."
            )
