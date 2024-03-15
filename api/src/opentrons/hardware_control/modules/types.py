from __future__ import annotations
from enum import Enum
from dataclasses import dataclass
from typing import (
    Dict,
    NamedTuple,
    Callable,
    Any,
    Tuple,
    Awaitable,
    Union,
    Set,
    cast,
    TYPE_CHECKING,
)
from typing_extensions import TypedDict
from pathlib import Path

from opentrons.drivers.rpi_drivers.types import USBPort

if TYPE_CHECKING:
    from opentrons_shared_data.module.dev_types import (
        ThermocyclerModuleType,
        MagneticModuleType,
        TemperatureModuleType,
        HeaterShakerModuleType,
        MagneticBlockType,
    )


class ThermocyclerStepBase(TypedDict):
    temperature: float


class ThermocyclerStep(ThermocyclerStepBase, total=False):
    hold_time_seconds: float
    hold_time_minutes: float


UploadFunction = Callable[[str, str, Dict[str, Any]], Awaitable[Tuple[bool, str]]]


class LiveData(TypedDict):
    status: str
    data: Dict[str, Union[float, str, bool, None]]


class ModuleType(str, Enum):
    THERMOCYCLER: ThermocyclerModuleType = "thermocyclerModuleType"
    TEMPERATURE: TemperatureModuleType = "temperatureModuleType"
    MAGNETIC: MagneticModuleType = "magneticModuleType"
    HEATER_SHAKER: HeaterShakerModuleType = "heaterShakerModuleType"
    MAGNETIC_BLOCK: MagneticBlockType = "magneticBlockType"

    @classmethod
    def from_model(cls, model: ModuleModel) -> ModuleType:
        if isinstance(model, MagneticModuleModel):
            return cls.MAGNETIC
        if isinstance(model, TemperatureModuleModel):
            return cls.TEMPERATURE
        if isinstance(model, ThermocyclerModuleModel):
            return cls.THERMOCYCLER
        if isinstance(model, HeaterShakerModuleModel):
            return cls.HEATER_SHAKER
        if isinstance(model, MagneticBlockModel):
            return cls.MAGNETIC_BLOCK

    @classmethod
    def to_module_fixture_id(cls, module_type: ModuleType) -> str:
        if module_type == ModuleType.THERMOCYCLER:
            # Thermocyclers are "loaded" in B1 only
            return "thermocyclerModuleFront"
        if module_type == ModuleType.TEMPERATURE:
            return "temperatureModule"
        if module_type == ModuleType.HEATER_SHAKER:
            return "heaterShakerModule"
        if module_type == ModuleType.MAGNETIC_BLOCK:
            return "magneticBlockModule"
        else:
            raise ValueError(
                f"Module Type {module_type} does not have a related fixture ID."
            )


class MagneticModuleModel(str, Enum):
    MAGNETIC_V1: str = "magneticModuleV1"
    MAGNETIC_V2: str = "magneticModuleV2"


class TemperatureModuleModel(str, Enum):
    TEMPERATURE_V1: str = "temperatureModuleV1"
    TEMPERATURE_V2: str = "temperatureModuleV2"


class ThermocyclerModuleModel(str, Enum):
    THERMOCYCLER_V1: str = "thermocyclerModuleV1"
    THERMOCYCLER_V2: str = "thermocyclerModuleV2"


class HeaterShakerModuleModel(str, Enum):
    HEATER_SHAKER_V1: str = "heaterShakerModuleV1"


class MagneticBlockModel(str, Enum):
    MAGNETIC_BLOCK_V1: str = "magneticBlockV1"


def module_model_from_string(model_string: str) -> ModuleModel:
    for model_enum in {
        MagneticModuleModel,
        TemperatureModuleModel,
        ThermocyclerModuleModel,
        HeaterShakerModuleModel,
        MagneticBlockModel,
    }:
        try:
            return cast(ModuleModel, model_enum(model_string))
        except ValueError:
            pass
    raise ValueError(f"No such module model {model_string}")


@dataclass
class ModuleAtPort:
    port: str
    name: str
    usb_port: USBPort = USBPort(name="", port_number=0)


class BundledFirmware(NamedTuple):
    """Represents a versioned firmware file, generally bundled into the fs"""

    version: str
    path: Path

    def __repr__(self) -> str:
        return f"<BundledFirmware {self.version}, path={self.path}>"


class UpdateError(RuntimeError):
    pass


class ModuleInfo(NamedTuple):
    model: str  # A module model such as "magneticModuleV2"
    fw_version: str  # The version of the firmware
    hw_revision: str  # the revision of the hardware
    serial: str  # the serial number


# TODO(mc, 2022-01-18): replace with enum
ModuleModel = Union[
    MagneticModuleModel,
    TemperatureModuleModel,
    ThermocyclerModuleModel,
    HeaterShakerModuleModel,
    MagneticBlockModel,
]


class MagneticStatus(str, Enum):
    ENGAGED = "engaged"
    DISENGAGED = "disengaged"


class TemperatureStatus(str, Enum):
    HOLDING = "holding at target"
    COOLING = "cooling"
    HEATING = "heating"
    IDLE = "idle"
    ERROR = "error"


class SpeedStatus(str, Enum):
    HOLDING = "holding at target"
    ACCELERATING = "speeding up"
    DECELERATING = "slowing down"
    IDLE = "idle"
    ERROR = "error"


class HeaterShakerStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    ERROR = "error"
