from enum import Enum
from dataclasses import dataclass, asdict, fields
from typing import Dict, Tuple, TypeVar, Generic, List, cast, Optional, TYPE_CHECKING
from typing_extensions import TypedDict, Literal

if TYPE_CHECKING:
    # Work around circular import.
    from opentrons.hardware_control.types import OT3AxisKind, InstrumentProbeType


class AxisDict(TypedDict):
    X: float
    Y: float
    Z: float
    A: float
    B: float
    C: float


Vt = TypeVar("Vt")


class GantryLoad(Enum):
    HIGH_THROUGHPUT = "high_throughput"
    LOW_THROUGHPUT = "low_throughput"


@dataclass
class ByGantryLoad(Generic[Vt]):
    high_throughput: Vt
    low_throughput: Vt

    def __getitem__(self, key: GantryLoad) -> Vt:
        return cast(Vt, asdict(self)[key.value])


PerPipetteAxisSettings = ByGantryLoad[Dict["OT3AxisKind", float]]


class CurrentDictDefault(TypedDict):
    default: AxisDict


CurrentDictModelEntries = TypedDict(
    "CurrentDictModelEntries",
    {"2.1": AxisDict, "A": AxisDict, "B": AxisDict, "C": AxisDict},
    total=False,
)


class CurrentDict(CurrentDictDefault, CurrentDictModelEntries):
    pass


Offset = Tuple[float, float, float]


@dataclass
class RobotConfig:
    model: Literal["OT-2 Standard"]
    name: str
    version: int
    gantry_steps_per_mm: Dict[str, float]
    acceleration: Dict[str, float]
    serial_speed: int
    default_pipette_configs: Dict[str, float]
    default_current: CurrentDict
    low_current: CurrentDict
    high_current: CurrentDict
    default_max_speed: AxisDict
    log_level: str
    z_retract_distance: float
    left_mount_offset: Offset


OT3Transform = List[List[float]]


@dataclass(frozen=True)
class OT3MotionSettings:
    default_max_speed: PerPipetteAxisSettings
    acceleration: PerPipetteAxisSettings
    max_speed_discontinuity: PerPipetteAxisSettings
    direction_change_speed_discontinuity: PerPipetteAxisSettings

    def by_gantry_load(
        self, gantry_load: GantryLoad
    ) -> Dict[str, Dict["OT3AxisKind", float]]:
        return dict(
            (field.name, getattr(self, field.name)[gantry_load])
            for field in fields(self)
        )


@dataclass(frozen=True)
class OT3CurrentSettings:
    hold_current: PerPipetteAxisSettings
    run_current: PerPipetteAxisSettings

    def by_gantry_load(
        self, gantry_load: GantryLoad
    ) -> Dict[str, Dict["OT3AxisKind", float]]:
        return dict(
            (field.name, getattr(self, field.name)[gantry_load])
            for field in fields(self)
        )


@dataclass(frozen=True)
class CapacitivePassSettings:
    prep_distance_mm: float
    max_overrun_distance_mm: float
    speed_mm_per_s: float
    sensor_threshold_pf: float


@dataclass(frozen=True)
class ZSenseSettings:
    pass_settings: CapacitivePassSettings


# str enum so it can be json serializable
class OutputOptions(int, Enum):
    """Specifies where we should report sensor data to during a sensor pass."""

    stream_to_csv = 0x1
    sync_buffer_to_csv = 0x2
    can_bus_only = 0x4
    sync_only = 0x8


@dataclass
class LiquidProbeSettings:
    max_z_distance: float
    min_z_distance: float
    mount_speed: float
    plunger_speed: float
    sensor_threshold_pascals: float
    expected_liquid_height: float
    output_option: OutputOptions
    aspirate_while_sensing: bool
    auto_zero_sensor: bool
    num_baseline_reads: int
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    data_files: Optional[Dict[InstrumentProbeType, str]]
=======
    data_file: Optional[str]
>>>>>>> 7995d78c39 (refactor(hardware): give options for sensor data output during probe (#14673))
=======
    data_files: Optional[Dict[InstrumentProbeType, str]]
>>>>>>> b3b65dfc27 (feat(hardware-testing): enable multi sensor processing in liquid probe (#14883))
=======
    data_files: Optional[Dict["InstrumentProbeType", str]]
>>>>>>> bc32d40b51 (refactor(api): Move opentrons.initialize() to its own file (#15191))


@dataclass(frozen=True)
class EdgeSenseSettings:
    overrun_tolerance_mm: float
    early_sense_tolerance_mm: float
    pass_settings: CapacitivePassSettings
    search_initial_tolerance_mm: float
    search_iteration_limit: int

    def __init__(
        self,
        overrun_tolerance_mm: float,
        early_sense_tolerance_mm: float,
        pass_settings: CapacitivePassSettings,
        search_initial_tolerance_mm: float,
        search_iteration_limit: int,
    ) -> None:
        if overrun_tolerance_mm > pass_settings.max_overrun_distance_mm:
            raise ValueError("Overrun tolerance and pass setting distance do not match")
        object.__setattr__(self, "overrun_tolerance_mm", overrun_tolerance_mm)
        object.__setattr__(self, "early_sense_tolerance_mm", early_sense_tolerance_mm)
        object.__setattr__(self, "pass_settings", pass_settings)
        object.__setattr__(
            self, "search_initial_tolerance_mm", search_initial_tolerance_mm
        )
        object.__setattr__(self, "search_iteration_limit", search_iteration_limit)


@dataclass(frozen=True)
class OT3CalibrationSettings:
    z_offset: ZSenseSettings
    edge_sense: EdgeSenseSettings
    probe_length: float


@dataclass
class OT3Config:
    model: Literal["OT-3 Standard"]
    name: str
    version: int
    log_level: str
    motion_settings: OT3MotionSettings
    current_settings: OT3CurrentSettings
    safe_home_distance: float
    deck_transform: OT3Transform
    carriage_offset: Offset
    left_mount_offset: Offset
    right_mount_offset: Offset
    gripper_mount_offset: Offset
    calibration: OT3CalibrationSettings
    liquid_sense: LiquidProbeSettings
