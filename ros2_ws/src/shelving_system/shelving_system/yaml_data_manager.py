"""YAML configuration loader for the book shelving system."""

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


class ConfigurationError(ValueError):
    """Raised when a YAML configuration is missing or invalid."""


class ShelfNotFoundError(LookupError):
    """Raised when no shelf matches a classification code."""


class BookProfileNotFoundError(LookupError):
    """Raised when a requested book profile does not exist."""


class YamlDataManager:
    """Load and provide access to shelving system YAML data."""

    _REQUIRED_BOOK_PROFILE_FIELDS = {
        "width",
        "height",
        "thickness",
        "safety_margin",
    }

    def __init__(
        self,
        shelf_map_path: str | Path,
        book_profiles_path: str | Path,
        system_config_path: str | Path,
    ) -> None:
        """Load and validate all required YAML configuration files."""
        self._shelf_map_path = Path(shelf_map_path)
        self._book_profiles_path = Path(book_profiles_path)
        self._system_config_path = Path(system_config_path)

        self._shelf_map = self._load_yaml(
            self._shelf_map_path,
            config_name="shelf map",
        )
        self._book_profiles = self._load_yaml(
            self._book_profiles_path,
            config_name="book profiles",
        )
        self._system_config = self._load_yaml(
            self._system_config_path,
            config_name="system configuration",
        )

        self._validate_shelf_map()
        self._validate_book_profiles()
        self._validate_system_config()

    @staticmethod
    def _load_yaml(
        path: Path,
        config_name: str,
    ) -> dict[str, Any]:
        """Read one YAML file and return its root mapping."""
        if not path.exists():
            raise ConfigurationError(
                f"{config_name} file does not exist: {path}"
            )

        if not path.is_file():
            raise ConfigurationError(
                f"{config_name} path is not a file: {path}"
            )

        try:
            with path.open(
                mode="r",
                encoding="utf-8",
            ) as yaml_file:
                loaded_data = yaml.safe_load(yaml_file)
        except OSError as error:
            raise ConfigurationError(
                f"Failed to read {config_name}: {path}"
            ) from error
        except yaml.YAMLError as error:
            raise ConfigurationError(
                f"Invalid YAML in {config_name}: {path}"
            ) from error

        if not isinstance(loaded_data, dict):
            raise ConfigurationError(
                f"{config_name} must contain a YAML mapping."
            )

        return loaded_data

    def _validate_shelf_map(self) -> None:
        """Validate the shelf map configuration."""
        frame_id = self._shelf_map.get("frame_id")

        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ConfigurationError(
                "shelf_map.yaml requires a non-empty frame_id."
            )

        locations = self._shelf_map.get("locations")

        if not isinstance(locations, dict):
            raise ConfigurationError(
                "shelf_map.yaml requires a locations mapping."
            )

        shelves = self._shelf_map.get("shelves")

        if not isinstance(shelves, dict) or not shelves:
            raise ConfigurationError(
                "shelf_map.yaml requires a non-empty shelves mapping."
            )

        for shelf_id, shelf_data in shelves.items():
            if not isinstance(shelf_data, dict):
                raise ConfigurationError(
                    f"Shelf '{shelf_id}' must contain a mapping."
                )

            prefixes = shelf_data.get("classification_prefixes")

            if not isinstance(prefixes, list) or not prefixes:
                raise ConfigurationError(
                    f"Shelf '{shelf_id}' requires "
                    "classification_prefixes."
                )

            for prefix in prefixes:
                if not isinstance(prefix, str) or not prefix.strip():
                    raise ConfigurationError(
                        f"Shelf '{shelf_id}' contains an invalid "
                        "classification prefix."
                    )

            observation_pose = shelf_data.get("observation_pose")

            if not isinstance(observation_pose, dict):
                raise ConfigurationError(
                    f"Shelf '{shelf_id}' requires observation_pose."
                )

            self._validate_pose(
                observation_pose,
                pose_name=f"shelf '{shelf_id}' observation pose",
            )

        for location_name, location_pose in locations.items():
            if not isinstance(location_pose, dict):
                raise ConfigurationError(
                    f"Location '{location_name}' must contain a pose."
                )

            self._validate_pose(
                location_pose,
                pose_name=f"location '{location_name}'",
            )

    def _validate_book_profiles(self) -> None:
        """Validate book profile configuration."""
        default_profile = self._book_profiles.get("default_profile")
        profiles = self._book_profiles.get("profiles")

        if not isinstance(default_profile, str):
            raise ConfigurationError(
                "book_profiles.yaml requires default_profile."
            )

        if not isinstance(profiles, dict) or not profiles:
            raise ConfigurationError(
                "book_profiles.yaml requires a non-empty profiles mapping."
            )

        if default_profile not in profiles:
            raise ConfigurationError(
                f"Default book profile '{default_profile}' does not exist."
            )

        for profile_name, profile_data in profiles.items():
            if not isinstance(profile_data, dict):
                raise ConfigurationError(
                    f"Book profile '{profile_name}' must be a mapping."
                )

            missing_fields = (
                self._REQUIRED_BOOK_PROFILE_FIELDS
                - profile_data.keys()
            )

            if missing_fields:
                missing_fields_text = ", ".join(
                    sorted(missing_fields)
                )

                raise ConfigurationError(
                    f"Book profile '{profile_name}' is missing: "
                    f"{missing_fields_text}"
                )

            for field_name in self._REQUIRED_BOOK_PROFILE_FIELDS:
                value = profile_data[field_name]

                if not self._is_positive_number(value):
                    raise ConfigurationError(
                        f"Book profile '{profile_name}' field "
                        f"'{field_name}' must be a positive number."
                    )

    def _validate_system_config(self) -> None:
        """Validate timeout and retry settings."""
        timeouts = self._system_config.get("timeouts")
        retry_limits = self._system_config.get("retry_limits")

        if not isinstance(timeouts, dict) or not timeouts:
            raise ConfigurationError(
                "system.yaml requires a non-empty timeouts mapping."
            )

        if not isinstance(retry_limits, dict) or not retry_limits:
            raise ConfigurationError(
                "system.yaml requires a non-empty retry_limits mapping."
            )

        for timeout_name, timeout_value in timeouts.items():
            if not self._is_positive_number(timeout_value):
                raise ConfigurationError(
                    f"Timeout '{timeout_name}' must be a positive number."
                )

        for retry_name, retry_value in retry_limits.items():
            if (
                isinstance(retry_value, bool)
                or not isinstance(retry_value, int)
                or retry_value < 0
            ):
                raise ConfigurationError(
                    f"Retry limit '{retry_name}' "
                    "must be a non-negative integer."
                )

    @staticmethod
    def _validate_pose(
        pose: dict[str, Any],
        pose_name: str,
    ) -> None:
        """Validate a position and orientation mapping."""
        position = pose.get("position")
        orientation = pose.get("orientation")

        if not isinstance(position, dict):
            raise ConfigurationError(
                f"{pose_name} requires position."
            )

        if not isinstance(orientation, dict):
            raise ConfigurationError(
                f"{pose_name} requires orientation."
            )

        for field_name in ("x", "y", "z"):
            value = position.get(field_name)

            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
            ):
                raise ConfigurationError(
                    f"{pose_name} position.{field_name} "
                    "must be a number."
                )

        for field_name in ("x", "y", "z", "w"):
            value = orientation.get(field_name)

            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
            ):
                raise ConfigurationError(
                    f"{pose_name} orientation.{field_name} "
                    "must be a number."
                )

    @staticmethod
    def _is_positive_number(value: Any) -> bool:
        """Return whether a value is a positive number."""
        return (
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and value > 0
        )

    def get_shelf_for_classification(
        self,
        classification_code: str,
    ) -> dict[str, Any]:
        """Return the shelf matching a classification-code prefix."""
        if not isinstance(classification_code, str):
            raise TypeError(
                "classification_code must be a string."
            )

        normalized_code = classification_code.strip()

        if not normalized_code:
            raise ShelfNotFoundError(
                "Classification code must not be empty."
            )

        best_match: tuple[
            int,
            str,
            dict[str, Any],
        ] | None = None

        shelves = self._shelf_map["shelves"]

        for shelf_id, shelf_data in shelves.items():
            prefixes = shelf_data["classification_prefixes"]

            for prefix in prefixes:
                if not normalized_code.startswith(prefix):
                    continue

                prefix_length = len(prefix)

                if (
                    best_match is None
                    or prefix_length > best_match[0]
                ):
                    best_match = (
                        prefix_length,
                        shelf_id,
                        shelf_data,
                    )

        if best_match is None:
            raise ShelfNotFoundError(
                "No shelf mapping found for classification code "
                f"'{classification_code}'."
            )

        _, shelf_id, shelf_data = best_match

        result = deepcopy(shelf_data)
        result["shelf_id"] = shelf_id
        result["frame_id"] = self._shelf_map["frame_id"]

        return result

    def get_location_pose(
        self,
        location_name: str,
    ) -> dict[str, Any]:
        """Return a named location pose in the map frame."""
        locations = self._shelf_map["locations"]

        if location_name not in locations:
            raise ConfigurationError(
                f"Unknown location: '{location_name}'."
            )

        pose = deepcopy(locations[location_name])
        pose["frame_id"] = self._shelf_map["frame_id"]

        return pose

    def get_book_profile(
        self,
        profile_name: str | None = None,
    ) -> dict[str, Any]:
        """Return the requested or default book profile."""
        selected_name = profile_name

        if selected_name is None:
            selected_name = self._book_profiles["default_profile"]

        profiles = self._book_profiles["profiles"]

        if selected_name not in profiles:
            raise BookProfileNotFoundError(
                f"Unknown book profile: '{selected_name}'."
            )

        profile = deepcopy(profiles[selected_name])
        profile["profile_name"] = selected_name

        return profile

    def get_timeout(self, component: str) -> float:
        """Return the timeout value for a component."""
        timeouts = self._system_config["timeouts"]

        if component not in timeouts:
            raise ConfigurationError(
                f"Unknown timeout setting: '{component}'."
            )

        return float(timeouts[component])

    def get_retry_limit(self, component: str) -> int:
        """Return the retry limit for a component."""
        retry_limits = self._system_config["retry_limits"]

        if component not in retry_limits:
            raise ConfigurationError(
                f"Unknown retry-limit setting: '{component}'."
            )

        return int(retry_limits[component])
