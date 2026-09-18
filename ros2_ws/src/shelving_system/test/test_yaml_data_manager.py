"""Tests for the YAML data manager."""

from pathlib import Path

import pytest
import yaml

from shelving_system.yaml_data_manager import (
    BookProfileNotFoundError,
    ConfigurationError,
    ShelfNotFoundError,
    YamlDataManager,
)


@pytest.fixture
def config_paths(tmp_path: Path) -> dict[str, Path]:
    """Create temporary YAML configuration files for each test."""
    shelf_map_data = {
        "frame_id": "map",
        "locations": {
            "home": {
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
                "position": {
                    "x": 1.0,
                    "y": 2.0,
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
        
        "shelves": {
            "shelf_01": {
                "classification_prefixes": [
                    "0",
                    "1",
                ],
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
            "shelf_02": {
                "classification_prefixes": [
                    "2",
                    "3",
                ],
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
        },
    }

    book_profiles_data = {
        "default_profile": "standard",
        "profiles": {
            "standard": {
                "width": 0.18,
                "height": 0.24,
                "thickness": 0.035,
                "safety_margin": 0.01,
            },
            "large": {
                "width": 0.22,
                "height": 0.30,
                "thickness": 0.05,
                "safety_margin": 0.015,
            },
        },
    }

    system_config_data = {
        "timeouts": {
            "navigation": 120.0,
            "perception": 10.0,
            "manipulation": 60.0,
        },
        "retry_limits": {
            "navigation": 2,
            "perception": 1,
            "manipulation": 1,
        },
    }

    shelf_map_path = tmp_path / "shelf_map.yaml"
    book_profiles_path = tmp_path / "book_profiles.yaml"
    system_config_path = tmp_path / "system.yaml"

    shelf_map_path.write_text(
        yaml.safe_dump(
            shelf_map_data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    book_profiles_path.write_text(
        yaml.safe_dump(
            book_profiles_data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    system_config_path.write_text(
        yaml.safe_dump(
            system_config_data,
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    return {
        "shelf_map": shelf_map_path,
        "book_profiles": book_profiles_path,
        "system": system_config_path,
    }


@pytest.fixture
def data_manager(
    config_paths: dict[str, Path],
) -> YamlDataManager:
    """Create a YAML data manager using temporary configuration files."""
    return YamlDataManager(
        shelf_map_path=config_paths["shelf_map"],
        book_profiles_path=config_paths["book_profiles"],
        system_config_path=config_paths["system"],
    )


def test_find_shelf_from_classification_prefix(
    data_manager: YamlDataManager,
) -> None:
    """A classification code should resolve to the correct shelf."""
    shelf = data_manager.get_shelf_for_classification("005.7")

    assert shelf["shelf_id"] == "shelf_01"
    assert shelf["frame_id"] == "map"
    assert shelf["observation_pose"]["position"]["x"] == 3.0


def test_find_second_shelf_from_classification_prefix(
    data_manager: YamlDataManager,
) -> None:
    """A different classification prefix should select another shelf."""
    shelf = data_manager.get_shelf_for_classification("230.1")

    assert shelf["shelf_id"] == "shelf_02"
    assert shelf["observation_pose"]["position"]["x"] == 5.0


def test_unknown_classification_raises_error(
    data_manager: YamlDataManager,
) -> None:
    """An unmapped classification code should raise an error."""
    with pytest.raises(ShelfNotFoundError):
        data_manager.get_shelf_for_classification("900.0")


def test_get_named_location_pose(
    data_manager: YamlDataManager,
) -> None:
    """Named locations such as HOME should be retrievable."""
    home_pose = data_manager.get_location_pose("home")

    assert home_pose["frame_id"] == "map"
    assert home_pose["position"]["x"] == 0.0
    assert home_pose["orientation"]["w"] == 1.0


def test_get_default_book_profile(
    data_manager: YamlDataManager,
) -> None:
    """The default book profile should be returned when no name is given."""
    profile = data_manager.get_book_profile()

    assert profile["profile_name"] == "standard"
    assert profile["width"] == pytest.approx(0.18)
    assert profile["height"] == pytest.approx(0.24)
    assert profile["thickness"] == pytest.approx(0.035)
    assert profile["safety_margin"] == pytest.approx(0.01)


def test_get_book_profile_by_name(
    data_manager: YamlDataManager,
) -> None:
    """A named book profile should be retrievable."""
    profile = data_manager.get_book_profile("large")

    assert profile["profile_name"] == "large"
    assert profile["height"] == pytest.approx(0.30)
    assert profile["thickness"] == pytest.approx(0.05)


def test_unknown_book_profile_raises_error(
    data_manager: YamlDataManager,
) -> None:
    """Requesting an unknown book profile should raise an error."""
    with pytest.raises(BookProfileNotFoundError):
        data_manager.get_book_profile("unknown")


def test_get_timeout_and_retry_limit(
    data_manager: YamlDataManager,
) -> None:
    """System timeout and retry settings should be retrievable."""
    assert data_manager.get_timeout("navigation") == pytest.approx(120.0)
    assert data_manager.get_timeout("perception") == pytest.approx(10.0)

    assert data_manager.get_retry_limit("navigation") == 2
    assert data_manager.get_retry_limit("perception") == 1


def test_missing_yaml_file_raises_configuration_error(
    config_paths: dict[str, Path],
    tmp_path: Path,
) -> None:
    """A missing configuration file should be rejected."""
    missing_path = tmp_path / "missing_shelf_map.yaml"

    with pytest.raises(ConfigurationError):
        YamlDataManager(
            shelf_map_path=missing_path,
            book_profiles_path=config_paths["book_profiles"],
            system_config_path=config_paths["system"],
        )


def test_missing_required_shelves_section_raises_error(
    config_paths: dict[str, Path],
) -> None:
    """A shelf map without the shelves section should be rejected."""
    config_paths["shelf_map"].write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "locations": {},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ConfigurationError):
        YamlDataManager(
            shelf_map_path=config_paths["shelf_map"],
            book_profiles_path=config_paths["book_profiles"],
            system_config_path=config_paths["system"],
        )