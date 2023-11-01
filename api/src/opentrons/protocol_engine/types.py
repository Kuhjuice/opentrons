"""Public protocol engine value types and models."""
from __future__ import annotations
import re
from datetime import datetime
from enum import Enum
from dataclasses import dataclass
from pydantic import BaseModel, Field, validator
from typing import Optional, Union, List, Dict, Any, NamedTuple
from typing_extensions import Literal, TypeGuard

from opentrons_shared_data.pipette.dev_types import PipetteNameType
from opentrons.types import MountType, DeckSlotName
from opentrons.hardware_control.modules import (
    ModuleType as ModuleType,
)

from opentrons_shared_data.pipette.dev_types import (  # noqa: F401
    # convenience re-export of LabwareUri type
    LabwareUri as LabwareUri,
)


class EngineStatus(str, Enum):
    """Current execution status of a ProtocolEngine."""

    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    BLOCKED_BY_OPEN_DOOR = "blocked-by-open-door"
    STOP_REQUESTED = "stop-requested"
    STOPPED = "stopped"
    FINISHING = "finishing"
    FAILED = "failed"
    SUCCEEDED = "succeeded"


class DeckSlotLocation(BaseModel):
    """The location of something placed in a single deck slot."""

    slotName: DeckSlotName = Field(
        ...,
        description=(
            # This description should be kept in sync with LabwareOffsetLocation.slotName.
            "A slot on the robot's deck."
            "\n\n"
            'The plain numbers like `"5"` are for the OT-2,'
            ' and the coordinates like `"C2"` are for the Flex.'
            "\n\n"
            "When you provide one of these values, you can use either style."
            " It will automatically be converted to match the robot."
            "\n\n"
            "When one of these values is returned, it will always match the robot."
        ),
    )


class ModuleLocation(BaseModel):
    """The location of something placed atop a hardware module."""

    moduleId: str = Field(
        ...,
        description="The ID of a loaded module from a prior `loadModule` command.",
    )


class OnLabwareLocation(BaseModel):
    """The location of something placed atop another labware."""

    labwareId: str = Field(
        ...,
        description="The ID of a loaded Labware from a prior `loadLabware` command.",
    )


_OffDeckLocationType = Literal["offDeck"]
OFF_DECK_LOCATION: _OffDeckLocationType = "offDeck"

LabwareLocation = Union[
    DeckSlotLocation, ModuleLocation, OnLabwareLocation, _OffDeckLocationType
]
"""Union of all locations where it's legal to keep a labware."""

OnDeckLabwareLocation = Union[DeckSlotLocation, ModuleLocation, OnLabwareLocation]

NonStackedLocation = Union[DeckSlotLocation, ModuleLocation, _OffDeckLocationType]
"""Union of all locations where it's legal to keep a labware that can't be stacked on another labware"""


class WellOrigin(str, Enum):
    """Origin of WellLocation offset.

    Props:
        TOP: the top-center of the well
        BOTTOM: the bottom-center of the well
        CENTER: the middle-center of the well
    """

    TOP = "top"
    BOTTOM = "bottom"
    CENTER = "center"


class DropTipWellOrigin(str, Enum):
    """The origin of a DropTipWellLocation offset.

    Props:
        TOP: the top-center of the well
        BOTTOM: the bottom-center of the well
        CENTER: the middle-center of the well
        DEFAULT: the default drop-tip location of the well,
            based on pipette configuration and length of the tip.
    """

    TOP = "top"
    BOTTOM = "bottom"
    CENTER = "center"
    DEFAULT = "default"


# This is deliberately a separate type from Vec3f to let components default to 0.
class WellOffset(BaseModel):
    """An offset vector in (x, y, z)."""

    x: float = 0
    y: float = 0
    z: float = 0


class WellLocation(BaseModel):
    """A relative location in reference to a well's location."""

    origin: WellOrigin = WellOrigin.TOP
    offset: WellOffset = Field(default_factory=WellOffset)


class DropTipWellLocation(BaseModel):
    """Like WellLocation, but for dropping tips.

    Unlike a typical WellLocation, the location for a drop tip
    defaults to location based on the tip length rather than the well's top.
    """

    origin: DropTipWellOrigin = DropTipWellOrigin.DEFAULT
    offset: WellOffset = Field(default_factory=WellOffset)


@dataclass(frozen=True)
class Dimensions:
    """Dimensions of an object in deck-space."""

    x: float
    y: float
    z: float


# TODO(mm, 2022-11-07): Deduplicate with Vec3f.
class DeckPoint(BaseModel):
    """Coordinates of a point in deck space."""

    x: float
    y: float
    z: float


# TODO(mm, 2023-05-10): Deduplicate with constants in
# opentrons.protocols.api_support.deck_type
# and consider moving to shared-data.
class DeckType(str, Enum):
    """Types of deck available."""

    OT2_STANDARD = "ot2_standard"
    OT2_SHORT_TRASH = "ot2_short_trash"
    OT3_STANDARD = "ot3_standard"


class LoadedPipette(BaseModel):
    """A pipette that has been loaded."""

    id: str
    pipetteName: PipetteNameType
    mount: MountType


@dataclass
class FlowRates:
    """Default and current flow rates for a pipette."""

    default_blow_out: Dict[str, float]
    default_aspirate: Dict[str, float]
    default_dispense: Dict[str, float]


@dataclass(frozen=True)
class CurrentWell:
    """The latest well that the robot has accessed."""

    pipette_id: str
    labware_id: str
    well_name: str


@dataclass(frozen=True)
class TipGeometry:
    """Tip geometry data.

    Props:
        length: The effective length (total length minus overlap) of a tip in mm.
        diameter: Tip diameter in mm.
        volume: Maximum volume in µL.
    """

    length: float
    diameter: float
    volume: float


class MovementAxis(str, Enum):
    """Axis on which to issue a relative movement."""

    X = "x"
    Y = "y"
    Z = "z"


class MotorAxis(str, Enum):
    """Motor axis on which to issue a home command."""

    X = "x"
    Y = "y"
    LEFT_Z = "leftZ"
    RIGHT_Z = "rightZ"
    LEFT_PLUNGER = "leftPlunger"
    RIGHT_PLUNGER = "rightPlunger"
    EXTENSION_Z = "extensionZ"
    EXTENSION_JAW = "extensionJaw"


# TODO(mc, 2022-01-18): use opentrons_shared_data.module.dev_types.ModuleModel
class ModuleModel(str, Enum):
    """All available modules' models."""

    TEMPERATURE_MODULE_V1 = "temperatureModuleV1"
    TEMPERATURE_MODULE_V2 = "temperatureModuleV2"
    MAGNETIC_MODULE_V1 = "magneticModuleV1"
    MAGNETIC_MODULE_V2 = "magneticModuleV2"
    THERMOCYCLER_MODULE_V1 = "thermocyclerModuleV1"
    THERMOCYCLER_MODULE_V2 = "thermocyclerModuleV2"
    HEATER_SHAKER_MODULE_V1 = "heaterShakerModuleV1"
    MAGNETIC_BLOCK_V1 = "magneticBlockV1"

    def as_type(self) -> ModuleType:
        """Get the ModuleType of this model."""
        if ModuleModel.is_temperature_module_model(self):
            return ModuleType.TEMPERATURE
        elif ModuleModel.is_magnetic_module_model(self):
            return ModuleType.MAGNETIC
        elif ModuleModel.is_thermocycler_module_model(self):
            return ModuleType.THERMOCYCLER
        elif ModuleModel.is_heater_shaker_module_model(self):
            return ModuleType.HEATER_SHAKER
        elif ModuleModel.is_magnetic_block(self):
            return ModuleType.MAGNETIC_BLOCK

        assert False, f"Invalid ModuleModel {self}"

    @classmethod
    def is_temperature_module_model(
        cls, model: ModuleModel
    ) -> TypeGuard[TemperatureModuleModel]:
        """Whether a given model is a Temperature Module."""
        return model in [cls.TEMPERATURE_MODULE_V1, cls.TEMPERATURE_MODULE_V2]

    @classmethod
    def is_magnetic_module_model(
        cls, model: ModuleModel
    ) -> TypeGuard[MagneticModuleModel]:
        """Whether a given model is a Magnetic Module."""
        return model in [cls.MAGNETIC_MODULE_V1, cls.MAGNETIC_MODULE_V2]

    @classmethod
    def is_thermocycler_module_model(
        cls, model: ModuleModel
    ) -> TypeGuard[ThermocyclerModuleModel]:
        """Whether a given model is a Thermocycler Module."""
        return model in [cls.THERMOCYCLER_MODULE_V1, cls.THERMOCYCLER_MODULE_V2]

    @classmethod
    def is_heater_shaker_module_model(
        cls, model: ModuleModel
    ) -> TypeGuard[HeaterShakerModuleModel]:
        """Whether a given model is a Heater-Shaker Module."""
        return model == cls.HEATER_SHAKER_MODULE_V1

    @classmethod
    def is_magnetic_block(cls, model: ModuleModel) -> TypeGuard[MagneticBlockModel]:
        """Whether a given model is a Magnetic block."""
        return model == cls.MAGNETIC_BLOCK_V1


TemperatureModuleModel = Literal[
    ModuleModel.TEMPERATURE_MODULE_V1, ModuleModel.TEMPERATURE_MODULE_V2
]
MagneticModuleModel = Literal[
    ModuleModel.MAGNETIC_MODULE_V1, ModuleModel.MAGNETIC_MODULE_V2
]
ThermocyclerModuleModel = Literal[
    ModuleModel.THERMOCYCLER_MODULE_V1, ModuleModel.THERMOCYCLER_MODULE_V2
]
HeaterShakerModuleModel = Literal[ModuleModel.HEATER_SHAKER_MODULE_V1]
MagneticBlockModel = Literal[ModuleModel.MAGNETIC_BLOCK_V1]


class ModuleDimensions(BaseModel):
    """Dimension type for modules."""

    bareOverallHeight: float
    overLabwareHeight: float
    lidHeight: Optional[float]


class Vec3f(BaseModel):
    """A 3D vector of floats."""

    x: float
    y: float
    z: float


# TODO(mm, 2022-11-07): Deduplicate with Vec3f.
class ModuleCalibrationPoint(BaseModel):
    """Calibration Point type for module definition."""

    x: float
    y: float
    z: float


# TODO(mm, 2022-11-07): Deduplicate with Vec3f.
class LabwareOffsetVector(BaseModel):
    """Offset, in deck coordinates from nominal to actual position."""

    x: float
    y: float
    z: float

    def __add__(self, other: Any) -> LabwareOffsetVector:
        """Adds two vectors together."""
        if not isinstance(other, LabwareOffsetVector):
            return NotImplemented
        return LabwareOffsetVector(
            x=self.x + other.x, y=self.y + other.y, z=self.z + other.z
        )

    def __sub__(self, other: Any) -> LabwareOffsetVector:
        """Subtracts two vectors."""
        if not isinstance(other, LabwareOffsetVector):
            return NotImplemented
        return LabwareOffsetVector(
            x=self.x - other.x, y=self.y - other.y, z=self.z - other.z
        )


# TODO(mm, 2022-11-07): Deduplicate with Vec3f.
class InstrumentOffsetVector(BaseModel):
    """Instrument Offset from home position to robot deck."""

    x: float
    y: float
    z: float


# TODO(mm, 2022-11-07): Deduplicate with Vec3f.
class ModuleOffsetVector(BaseModel):
    """Offset, in deck coordinates, from nominal to actual position of labware on a module."""

    x: float
    y: float
    z: float


class OverlapOffset(Vec3f):
    """Offset representing overlap space of one labware on top of another labware or module."""


class LabwareMovementOffsetData(BaseModel):
    """Offsets to be used during labware movement."""

    pickUpOffset: LabwareOffsetVector
    dropOffset: LabwareOffsetVector


# TODO(mm, 2023-04-13): Move to shared-data, so this binding can be maintained alongside the JSON
# schema that it's sourced from. We already do that for labware definitions and JSON protocols.
class ModuleDefinition(BaseModel):
    """A module definition conforming to module definition schema v3."""

    # Note: This field is misleading.
    #
    # This class only models v3 definitions ("module/schemas/3"), not v2 ("module/schemas/2").
    # labwareOffset is required to have a z-component, for example.
    #
    # When parsing from a schema v3 JSON definition into this model,
    # the definition's `"$otSharedSchema": "module/schemas/3"` field will be thrown away
    # because it has a dollar sign, which doesn't match this field.
    # Then, this field will default to "module/schemas/2", because no value was provided.
    #
    # We should fix this field once Jira RSS-221 is resolved. RSS-221 makes it difficult to fix
    # because robot-server has been storing and loading these bad fields in its database.
    otSharedSchema: str = Field("module/schemas/2", description="The current schema.")

    moduleType: ModuleType = Field(
        ...,
        description="Module type (Temperature/Magnetic/Thermocycler)",
    )

    model: ModuleModel = Field(..., description="Model name of the module")

    labwareOffset: LabwareOffsetVector = Field(
        ...,
        description="Labware offset in x, y, z.",
    )

    dimensions: ModuleDimensions = Field(..., description="Module dimension")

    calibrationPoint: ModuleCalibrationPoint = Field(
        ...,
        description="Calibration point of module.",
    )

    displayName: str = Field(..., description="Display name.")

    quirks: List[str] = Field(..., description="Module quirks")

    # In releases prior to https://github.com/Opentrons/opentrons/pull/11873 (v6.3.0),
    # the matrices in slotTransforms were 3x3.
    # After, they are 4x4, even though there was no schema version bump.
    #
    # Because old objects of this class, with the 3x3 matrices, were stored in robot-server's
    # database, this field needs to stay typed loosely enough to support both sizes.
    # We can fix this once Jira RSS-221 is resolved.
    slotTransforms: Dict[str, Any] = Field(
        ...,
        description="Dictionary of transforms for each slot.",
    )

    compatibleWith: List[ModuleModel] = Field(
        ...,
        description="List of module models this model is compatible with.",
    )
    gripperOffsets: Optional[Dict[str, LabwareMovementOffsetData]] = Field(
        default_factory=dict,
        description="Offsets to use for labware movement using gripper",
    )


class LoadedModule(BaseModel):
    """A module that has been loaded."""

    id: str
    model: ModuleModel
    location: Optional[DeckSlotLocation]
    serialNumber: Optional[str]


class LabwareOffsetLocation(BaseModel):
    """Parameters describing when a given offset may apply to a given labware load."""

    slotName: DeckSlotName = Field(
        ...,
        description=(
            "The deck slot where the protocol will load the labware."
            " Or, if the protocol will load the labware on a module,"
            " the deck slot where the protocol will load that module."
            "\n\n"
            # This description should be kept in sync with DeckSlotLocation.slotName.
            'The plain numbers like `"5"` are for the OT-2,'
            ' and the coordinates like `"C2"` are for the Flex.'
            "\n\n"
            "When you provide one of these values, you can use either style."
            " It will automatically be converted to match the robot."
            "\n\n"
            "When one of these values is returned, it will always match the robot."
        ),
    )
    moduleModel: Optional[ModuleModel] = Field(
        None,
        description=(
            "The model of the module that the labware will be loaded onto,"
            " if applicable."
            "\n\n"
            "Because of module compatibility, the model that the protocol requests"
            " may not be exactly the same"
            " as what it will find physically connected during execution."
            " For this labware offset to apply,"
            " this field must be the *requested* model, not the connected one."
            " You can retrieve this from a `loadModule` command's `params.model`"
            " in the protocol's analysis."
        ),
    )
    definitionUri: Optional[str] = Field(
        None,
        description=(
            "The definition URI of a labware that a labware can be loaded onto,"
            " if applicable."
            "\n\n"
            "This can be combined with moduleModel if the labware is loaded on top of"
            " an adapter that is loaded on a module."
        ),
    )


class LabwareOffset(BaseModel):
    """An offset that the robot adds to a pipette's position when it moves to a labware.

    During the run, if a labware is loaded whose definition URI and location
    both match what's found here, the given offset will be added to all
    pipette movements that use that labware as a reference point.
    """

    id: str = Field(..., description="Unique labware offset record identifier.")
    createdAt: datetime = Field(..., description="When this labware offset was added.")
    definitionUri: str = Field(..., description="The URI for the labware's definition.")
    location: LabwareOffsetLocation = Field(
        ...,
        description="Where the labware is located on the robot.",
    )
    vector: LabwareOffsetVector = Field(
        ...,
        description="The offset applied to matching labware.",
    )


class LabwareOffsetCreate(BaseModel):
    """Create request data for a labware offset."""

    definitionUri: str = Field(..., description="The URI for the labware's definition.")
    location: LabwareOffsetLocation = Field(
        ...,
        description="Where the labware is located on the robot.",
    )
    vector: LabwareOffsetVector = Field(
        ...,
        description="The offset applied to matching labware.",
    )


class LoadedLabware(BaseModel):
    """A labware that has been loaded."""

    id: str
    loadName: str
    definitionUri: str
    location: LabwareLocation = Field(
        ..., description="The labware's current location."
    )
    offsetId: Optional[str] = Field(
        None,
        description=(
            "An ID referencing the labware offset"
            " that applies to this labware placement."
            " Null or undefined means no offset was provided for this load,"
            " so the default of (0, 0, 0) will be used."
        ),
    )
    displayName: Optional[str] = Field(
        None,
        description="A user-specified display name for this labware, if provided.",
    )


class HexColor(BaseModel):
    """Hex color representation."""

    __root__: str

    @validator("__root__")
    def _color_is_a_valid_hex(cls, v: str) -> str:
        match = re.search(r"^#(?:[0-9a-fA-F]{3,4}){1,2}$", v)
        if not match:
            raise ValueError("Color is not a valid hex color.")
        return v


class Liquid(BaseModel):
    """Payload required to create a liquid."""

    id: str
    displayName: str
    description: str
    displayColor: Optional[HexColor]


class SpeedRange(NamedTuple):
    """Minimum and maximum allowed speeds for a shaking module."""

    min: int
    max: int


class TemperatureRange(NamedTuple):
    """Minimum and maximum allowed temperatures for a heating module."""

    min: float
    max: float


class HeaterShakerLatchStatus(Enum):
    """Heater-Shaker latch status for determining pipette and labware movement errors."""

    CLOSED = "closed"
    OPEN = "open"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class HeaterShakerMovementRestrictors:
    """Shaking status, latch status and slot location for determining movement restrictions."""

    plate_shaking: bool
    latch_status: HeaterShakerLatchStatus
    deck_slot: int


class LabwareMovementStrategy(str, Enum):
    """Strategy to use for labware movement."""

    USING_GRIPPER = "usingGripper"
    MANUAL_MOVE_WITH_PAUSE = "manualMoveWithPause"
    MANUAL_MOVE_WITHOUT_PAUSE = "manualMoveWithoutPause"


class PostRunHardwareState(Enum):
    """State of robot gantry & motors after a stop is performed and the hardware API is reset.

    HOME_AND_STAY_ENGAGED: home the gantry and keep all motors engaged. This allows the
        robot to continue performing movement actions without re-homing
    HOME_THEN_DISENGAGE: home the gantry and then disengage motors.
        Reduces current consumption of the motors and prevents coil heating.
        Re-homing is required to re-engage the motors and resume robot movement.
    STAY_ENGAGED_IN_PLACE: do not home after the stop and keep the motors engaged.
        Keeps gantry in the same position as prior to `stop()` execution
        and allows the robot to execute movement commands without requiring to re-home first.
    DISENGAGE_IN_PLACE: disengage motors and do not home the robot
    Probable states for pipette:
        - for 1- or 8-channel:
            - HOME_AND_STAY_ENGAGED after protocol runs
            - STAY_ENGAGED_IN_PLACE after maintenance runs
        - for 96-channel:
            - HOME_THEN_DISENGAGE after protocol runs
            - DISENGAGE_IN_PLACE after maintenance runs
    """

    HOME_AND_STAY_ENGAGED = "homeAndStayEngaged"
    HOME_THEN_DISENGAGE = "homeThenDisengage"
    STAY_ENGAGED_IN_PLACE = "stayEngagedInPlace"
    DISENGAGE_IN_PLACE = "disengageInPlace"


NOZZLE_NAME_REGEX = "[A-Z][0-100]"
ALLOWED_PRIMARY_NOZZLES = ["A1", "H1", "A12", "H12"]
PRIMARY_NOZZLE_LITERAL = Literal["A1", "H1", "A12", "H12"]


class EmptyNozzleLayoutConfiguration(BaseModel):
    """Empty basemodel to represent a reset to the nozzle configuration. Sending no parameters resets to default."""

    style: Literal["EMPTY"] = "EMPTY"


class SingleNozzleLayoutConfiguration(BaseModel):
    """Minimum information required for a new nozzle configuration."""

    style: Literal["SINGLE"] = "SINGLE"
    primary_nozzle: PRIMARY_NOZZLE_LITERAL = Field(
        ...,
        description="The primary nozzle to use in the layout configuration. This nozzle will update the critical point of the current pipette. For now, this is also the back left corner of your rectangle.",
    )


class RowNozzleLayoutConfiguration(BaseModel):
    """Minimum information required for a new nozzle configuration."""

    style: Literal["ROW"] = "ROW"
    primary_nozzle: PRIMARY_NOZZLE_LITERAL = Field(
        ...,
        description="The primary nozzle to use in the layout configuration. This nozzle will update the critical point of the current pipette. For now, this is also the back left corner of your rectangle.",
    )


class ColumnNozzleLayoutConfiguration(BaseModel):
    """Information required for nozzle configurations of type ROW and COLUMN."""

    style: Literal["COLUMN"] = "COLUMN"
    primary_nozzle: PRIMARY_NOZZLE_LITERAL = Field(
        ...,
        description="The primary nozzle to use in the layout configuration. This nozzle will update the critical point of the current pipette. For now, this is also the back left corner of your rectangle.",
    )


class QuadrantNozzleLayoutConfiguration(BaseModel):
    """Information required for nozzle configurations of type QUADRANT."""

    style: Literal["QUADRANT"] = "QUADRANT"
    primary_nozzle: PRIMARY_NOZZLE_LITERAL = Field(
        ...,
        description="The primary nozzle to use in the layout configuration. This nozzle will update the critical point of the current pipette. For now, this is also the back left corner of your rectangle.",
    )
    front_right_nozzle: str = Field(
        ...,
        regex=NOZZLE_NAME_REGEX,
        description="The front right nozzle in your configuration.",
    )


NozzleLayoutConfigurationType = Union[
    EmptyNozzleLayoutConfiguration,
    SingleNozzleLayoutConfiguration,
    ColumnNozzleLayoutConfiguration,
    RowNozzleLayoutConfiguration,
    QuadrantNozzleLayoutConfiguration,
]
