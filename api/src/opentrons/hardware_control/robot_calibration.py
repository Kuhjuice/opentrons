from functools import lru_cache
import logging
import numpy as np
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, List, Any, cast

from opentrons import config

from opentrons.config.robot_configs import (
    get_legacy_gantry_calibration,
    default_deck_calibration,
)
from opentrons.config.types import OT3Config
from opentrons.calibration_storage import (
    types,
    save_robot_deck_attitude,
    get_robot_deck_attitude,
)
from opentrons.calibration_storage.ot2 import models as ot2_models
from opentrons.types import Point
from opentrons.util import linal

from .util import DeckTransformState

log = logging.getLogger(__name__)

# Loading deck calibrations for the OT-2 only


@dataclass
class DeckCalibration:
    attitude: types.AttitudeMatrix
    source: types.SourceType
    status: types.CalibrationStatus
    last_modified: Optional[datetime] = None
    pipette_calibrated_with: Optional[str] = None
    tiprack: Optional[str] = None


@dataclass
class RobotCalibration:
    deck_calibration: DeckCalibration


@dataclass
class OT3Transforms(RobotCalibration):
    carriage_offset: Point
    left_mount_offset: Point
    right_mount_offset: Point
    gripper_mount_offset: Point


def build_ot3_transforms(config: OT3Config) -> OT3Transforms:
    return OT3Transforms(
        deck_calibration=DeckCalibration(
            attitude=config.deck_transform,
            source=types.SourceType.default,
            status=types.CalibrationStatus(),
        ),
        carriage_offset=Point(*config.carriage_offset),
        left_mount_offset=Point(*config.left_mount_offset),
        right_mount_offset=Point(*config.right_mount_offset),
        gripper_mount_offset=Point(*config.gripper_mount_offset),
    )


def build_temporary_identity_calibration() -> RobotCalibration:
    """
    Get a temporary identity deck cal suitable for use during
    calibration
    """
    return RobotCalibration(
        deck_calibration=DeckCalibration(
            attitude=default_deck_calibration(),
            source=types.SourceType.default,
            status=types.CalibrationStatus(),
        )
    )


def validate_attitude_deck_calibration(
    deck_cal: DeckCalibration,
) -> DeckTransformState:
    """
    This function determines whether the deck calibration is valid
    or not based on the following use-cases:

    TODO(lc, 8/10/2020): Expand on this method, or create
    another method to diagnose bad instrument offset data
    """
    curr_cal = np.array(deck_cal.attitude)
    row, _ = curr_cal.shape
    rank: int = np.linalg.matrix_rank(curr_cal)  # type: ignore
    if row != rank:
        # Check that the matrix is non-singular
        return DeckTransformState.SINGULARITY
    elif not deck_cal.last_modified:
        # Check that the matrix is not an identity
        return DeckTransformState.IDENTITY
    else:
        # Transform as it stands is sufficient.
        return DeckTransformState.OK


def validate_gantry_calibration(gantry_cal: List[List[float]]) -> DeckTransformState:
    """
    This function determines whether the gantry calibration is valid
    or not based on the following use-cases:
    """
    curr_cal = np.array(gantry_cal)
    row, _ = curr_cal.shape

    rank: int = np.linalg.matrix_rank(curr_cal)  # type: ignore

    id_matrix = linal.identity_deck_transform()

    z = abs(curr_cal[2][-1])

    outofrange = z < 16 or z > 34
    if row != rank:
        # Check that the matrix is non-singular
        return DeckTransformState.SINGULARITY
    elif np.array_equal(curr_cal, id_matrix):
        # Check that the matrix is not an identity
        return DeckTransformState.IDENTITY
    elif outofrange:
        # Check that the matrix is not out of range.
        return DeckTransformState.BAD_CALIBRATION
    else:
        # Transform as it stands is sufficient.
        return DeckTransformState.OK


def migrate_affine_xy_to_attitude(
    gantry_cal: List[List[float]],
) -> types.AttitudeMatrix:
    masked_transform = np.array(
        [
            [True, True, True, False],
            [True, True, True, False],
            [False, False, False, False],
            [False, False, False, False],
        ]
    )
    masked_array: np.ma.MaskedArray[
        Any, np.dtype[np.float64]
    ] = np.ma.masked_array(  # type: ignore
        gantry_cal, ~masked_transform
    )
    attitude_array = np.zeros((3, 3))
    np.put(attitude_array, [0, 1, 2], masked_array[0].compressed())
    np.put(attitude_array, [3, 4, 5], masked_array[1].compressed())
    np.put(attitude_array, 8, 1)
    return cast(List[List[float]], attitude_array.tolist())


def save_attitude_matrix(
    expected: linal.SolvePoints,
    actual: linal.SolvePoints,
    pipette_id: str,
    tiprack_hash: str,
) -> None:
    attitude = linal.solve_attitude(expected, actual)
    save_robot_deck_attitude(
        attitude,
        pipette_id,
        lw_hash=tiprack_hash,
    )


def load_attitude_matrix() -> DeckCalibration:
    calibration_data = cast(Optional[ot2_models.v1.DeckCalibrationModel], get_robot_deck_attitude())
    gantry_cal = get_legacy_gantry_calibration()
    if not calibration_data and gantry_cal:
        if validate_gantry_calibration(gantry_cal) == DeckTransformState.OK:
            log.debug(
                "Attitude deck calibration matrix not found. Migrating "
                "existing affine deck calibration matrix to {}".format(
                    config.get_opentrons_path("robot_calibration_dir")
                )
            )
            attitude = migrate_affine_xy_to_attitude(gantry_cal)
            save_robot_deck_attitude(
                transform=attitude,
                pipette_id=None,
                lw_hash=None,
                source=types.SourceType.legacy,
            )
            calibration_data = get_robot_deck_attitude()

    if calibration_data:
        return DeckCalibration(
            attitude=calibration_data.attitude,
            source=calibration_data.source,
            status=types.CalibrationStatus(**calibration_data.status.dict()),
            last_modified=calibration_data.last_modified,
            pipette_calibrated_with=calibration_data.pipette_calibrated_with,
            tiprack=calibration_data.tiprack,
        )
    else:
        # load default if deck calibration data do not exist
        return DeckCalibration(
            attitude=default_deck_calibration(),
            source=types.SourceType.default,
            status=types.CalibrationStatus(),
        )


def load() -> RobotCalibration:
    return RobotCalibration(deck_calibration=load_attitude_matrix())


class RobotCalibrationProvider:
    def __init__(self) -> None:
        self._robot_calibration = load()

    @lru_cache(1)
    def _validate(self) -> DeckTransformState:
        return validate_attitude_deck_calibration(
            self._robot_calibration.deck_calibration
        )

    @property
    def robot_calibration(self) -> RobotCalibration:
        return self._robot_calibration

    def reset_robot_calibration(self) -> None:
        self._validate.cache_clear()
        self._robot_calibration = load()

    def set_robot_calibration(self, robot_calibration: RobotCalibration) -> None:
        self._validate.cache_clear()
        self._robot_calibration = robot_calibration

    def validate_calibration(self) -> DeckTransformState:
        """
        The lru cache decorator is currently not supported by the
        ThreadManager. To work around this, we need to wrap the
        actual function around a dummy outer function.

        Once decorators are more fully supported, we can remove this.
        """
        return self._validate()
