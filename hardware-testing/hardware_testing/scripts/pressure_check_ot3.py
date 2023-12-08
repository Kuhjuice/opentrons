"""Pressure-Check OT3."""
import argparse
import asyncio
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from time import time
from typing import List, Tuple, Any

from opentrons_hardware.firmware_bindings.constants import SensorId
from opentrons_hardware.scripts.sensor_utils import hms

from opentrons_shared_data.errors.exceptions import PipetteOverpressureError

from opentrons.hardware_control.ot3api import OT3API
from opentrons.hardware_control.instruments.ot3.pipette import Pipette

from hardware_testing import data as test_data
from hardware_testing.opentrons_api import types
from hardware_testing.opentrons_api.types import Point
from hardware_testing.opentrons_api import helpers_ot3

TEST_NAME = "pressure-check-ot3"

SLOTS_TIP_RACK = [2, 5, 6, 7, 8, 9, 10, 11]
SLOT_RESERVOIR = 3
SLOT_TRASH = 12

TRASH_HEIGHT_MM = 40

DEFAULT_SUBMERGE_MM = -1.5  # aspirate depth below meniscus
RUN_ID, START_TIME = test_data.create_run_id_and_start_time()

test_running = True
test_tag = ""
well_top_to_meniscus_mm = 0.0


def _seconds_elapsed() -> float:
    return round(time() - START_TIME, 2)


def _tip_position(slot: int, well: str) -> Point:
    _rack_a1 = helpers_ot3.get_theoretical_a1_position(
        slot, f"opentrons_flex_96_tiprack_50ul"  # all volumes are same size
    )
    x = 9 * (int(well[1:]) - 1)
    y = -9 * "ABCDEFGH".index(well[0])
    return _rack_a1 + Point(x=x, y=y, z=0)


_available_tips: List[Point] = [
    _tip_position(slot, f"{row}{col}")
    for slot in SLOTS_TIP_RACK
    for col in range(1, 13)
    for row in "ABCDEFGH"
]

trash_nominal = helpers_ot3.get_slot_calibration_square_position_ot3(
    SLOT_TRASH
) + Point(z=TRASH_HEIGHT_MM)
reservoir_a1 = helpers_ot3.get_theoretical_a1_position(
    SLOT_RESERVOIR, f"nest_1_reservoir_195ml"
)

FLOW_RATE_SAFE = {
    1: {  # 1ch pipette
        50: {50: 5},  # P50  # 50ul tip
        1000: {50: 5, 200: 5, 1000: 5},  # P1000  # 50ul tip  # 200ul tip  # 1000ul tip
    },
    8: {  # 8ch pipette
        50: {50: 5},  # P50  # 50ul tip
        1000: {50: 5, 200: 5, 1000: 5},  # P1000  # 50ul tip  # 200ul tip  # 1000ul tip
    },
}

TEST_ASPIRATE_VOLUME = {
    1: {  # 1ch pipette
        50: {50: [1, 5, 10, 20, 50]},  # P50  # 50ul tip
        1000: {  # P1000
            50: [5, 10, 20, 50],  # 50ul tip
            200: [5, 20, 100, 200],  # 200ul tip
            1000: [10, 100, 500, 1000],  # 1000ul tip
        },
    },
    8: {  # 8ch pipette
        50: {50: [1, 5, 10, 20, 50]},  # P50  # 50ul tip
        1000: {  # P1000
            50: [5, 10, 20, 50],  # 50ul tip
            200: [5, 20, 100, 200],  # 200ul tip
            1000: [10, 100, 500, 1000],  # 1000ul tip
        },
    },
}

TEST_FLOW_RATE_ASPIRATE = {
    1: {  # 1ch pipette
        50: {50: []},  # P50  # 50ul tip
        1000: {  # P1000
            50: [],  # 50ul tip
            200: [],  # 200ul tip
            1000: [],  # 1000ul tip
        },
    },
    8: {  # 8ch pipette
        50: {50: []},  # P50  # 50ul tip
        1000: {  # P1000
            50: [1, 5, 10, 15, 20],  # 50ul tip
            200: [],  # 200ul tip
            1000: [],  # 1000ul tip
        },
    },
}

TEST_FLOW_RATE_DISPENSE = {
    1: {  # 1ch pipette
        50: {50: []},  # P50  # 50ul tip
        1000: {  # P1000
            50: [1, 5, 10, 15, 20],  # 50ul tip
            200: [],  # 200ul tip
            1000: [],  # 1000ul tip
        },
    },
    8: {  # 8ch pipette
        50: {50: []},  # P50  # 50ul tip
        1000: {  # P1000
            50: [1, 5, 10, 15, 20],  # 50ul tip
            200: [],  # 200ul tip
            1000: [],  # 1000ul tip
        },
    },
}


@dataclass
class PipetteSettings:
    hw_pipette: Pipette
    mount: types.OT3Mount
    channels: int
    volume: int
    tip: int
    sensor_ids: List[SensorId]
    center_offset: Point
    offset_tip_rack: Point
    offset_reservoir: Point

    @classmethod
    def build(
        cls, hwapi: OT3API, tip: int, offset_tip_rack: Point, offset_reservoir: Point
    ) -> "PipetteSettings":
        mount = types.OT3Mount.LEFT
        pip = hwapi.hardware_pipettes[mount.to_mount()]
        ch = pip.channels.value
        if ch == 1:
            sids = [SensorId.S0]
        else:
            sids = [s for s in SensorId]
        return cls(
            hw_pipette=pip,
            mount=types.OT3Mount.LEFT,
            channels=pip.channels.value,
            volume=1000 if "1000" in pip.name else 50,
            tip=tip,
            sensor_ids=sids,
            center_offset=Point(
                x=0 if ch <= 8 else 9 * 11 * 0.5, y=9 * min(ch, 8) * 0.5, z=0
            ),
            offset_tip_rack=offset_tip_rack,
            offset_reservoir=offset_reservoir,
        )


@dataclass
class TrialSettings:
    pipette: PipetteSettings
    aspirate_volume: float
    submerge: float
    flow_rate_aspirate: float
    flow_rate_dispense: float

    def __str__(self) -> str:
        return (
            f"channels-{self.pipette.channels}_"
            f"pipette-{self.pipette.volume}_"
            f"tip-{self.pipette.tip}_"
            f"aspirate-volume-{self.aspirate_volume}_"
            f"flow-rate-aspirate-{self.flow_rate_aspirate}_"
            f"flow-rate-dispense-{self.flow_rate_dispense}"
        )


def _flow_rate_all(api: OT3API, pipette: PipetteSettings, _fr: float) -> None:
    api.set_flow_rate(pipette.mount, aspirate=_fr, dispense=_fr, blow_out=_fr)


async def _pick_up_tip(api: OT3API, pipette: PipetteSettings, tip_vol: int) -> None:
    global _available_tips
    assert len(_available_tips), "ran out of tip"
    await api.retract(pipette.mount)
    tip_length = helpers_ot3.get_default_tip_length(tip_vol)
    tip_pos = _available_tips[0]
    await helpers_ot3.move_to_arched_ot3(
        api, pipette.mount, tip_pos + pipette.offset_tip_rack
    )
    await api.pick_up_tip(pipette.mount, tip_length=tip_length)
    if not api.is_simulator:
        tip_state = await api.get_tip_presence_status(pipette.mount)
        assert tip_state == types.TipStateType.PRESENT, "tip not detected"
    for _ in range(pipette.channels):
        _available_tips.pop(0)
    await api.retract(pipette.mount)


async def _drop_tip(
    api: OT3API, pipette: PipetteSettings, after_fail: bool = False
) -> None:
    async def _drop() -> None:
        await api.retract(pipette.mount)
        loc = trash_nominal + pipette.center_offset
        await helpers_ot3.move_to_arched_ot3(api, pipette.mount, loc)
        await api.drop_tip(pipette.mount)
        _tip_state = await api.get_tip_presence_status(pipette.mount)
        assert _tip_state == types.TipStateType.ABSENT, "tip still detected"

    attempt_count = 0
    while True:
        attempt_count += 1
        print(f"dropping tip (#{attempt_count})")
        if after_fail:
            # slow down to some known safe speed
            _flow_rate_all(
                api,
                pipette,
                FLOW_RATE_SAFE[pipette.channels][pipette.volume][pipette.tip],
            )
        try:
            await _drop()
            break
        except PipetteOverpressureError as e:
            print(e)
            print("\ntrying again (or just pull the tip off...)")
            after_fail = True
            # gantry says it needs to reset position
            await api.home(list(types.Axis.gantry_axes()))
    await api.home_plunger(pipette.mount)
    await api.retract(pipette.mount)


async def _move_to_meniscus(api: OT3API, pip_settings: PipetteSettings) -> None:
    global well_top_to_meniscus_mm
    await api.retract(pip_settings.mount)
    # offset Y axis in 12-row reservoir
    # also include submerge depth (below top of labware)
    loc = reservoir_a1 + pip_settings.center_offset._replace(z=well_top_to_meniscus_mm)
    await helpers_ot3.move_to_arched_ot3(
        api, pip_settings.mount, loc + pip_settings.offset_reservoir
    )
    if not well_top_to_meniscus_mm:
        while True:
            _inp = _input(api, '"J" to jog down 1mm, or "stop" at meniscus: ').strip()
            if _inp.lower() == "j":
                await api.move_rel(pip_settings.mount, Point(z=-1.0))
                well_top_to_meniscus_mm -= 1.0
            elif _inp.lower() == "stop" or api.is_simulator:
                break
            else:
                continue


def _input(api: OT3API, msg: str) -> str:
    if api.is_simulator:
        print(msg)
        return ""
    return input(msg)


def _input_number(api: OT3API, msg: str) -> float:
    if api.is_simulator:
        print(msg + "50")
        return 50
    try:
        return float(_input(api, msg))
    except ValueError:
        return _input_number(api, msg)


async def _run_and_get_pressure(coro: Any, file: Path, simulate: bool) -> List[Tuple[datetime, float]]:
    if simulate:
        start = datetime.strptime("12:34:56.0", hms)
    else:
        start = datetime.now()
    await coro
    if simulate:
        end = datetime.strptime("12:34:58.0", hms)
    else:
        end = datetime.now()
    return _read_lines(file, start, end)


async def _delay(seconds: int, simulate: bool) -> None:
    for i in range(seconds):
        if i % 5 == 0:
            print(f"\t{i}/{seconds} seconds")
        await asyncio.sleep(0.01 if simulate else 1.0)


async def _run_trial(api: OT3API, trial: TrialSettings, pressure_file: Path) -> List[List[Tuple[datetime, float]]]:
    print("\n\n\n\n----------")
    await _pick_up_tip(api, trial.pipette, int(trial.pipette.tip))
    await _move_to_meniscus(api, trial.pipette)
    await api.move_rel(trial.pipette.mount, Point(z=-abs(trial.submerge)))
    api.set_flow_rate(
        trial.pipette.mount,
        aspirate=trial.flow_rate_aspirate,
        dispense=trial.flow_rate_dispense,
        blow_out=trial.flow_rate_dispense,
    )
    press_asp = []
    press_asp_del = []
    press_disp = []
    press_disp_del = []
    try:
        print(
            f"aspirating {trial.aspirate_volume} uL at {trial.flow_rate_aspirate} ul/sec"
        )
        press_asp = await _run_and_get_pressure(
            api.aspirate(trial.pipette.mount, volume=trial.aspirate_volume),
            pressure_file,
            api.is_simulator
        )
        print("delaying after aspirate...")
        press_asp_del = await _run_and_get_pressure(
            _delay(60 * 2, api.is_simulator),
            pressure_file,
            api.is_simulator
        )
        await api.move_rel(trial.pipette.mount, Point(z=abs(well_top_to_meniscus_mm)))
        print(
            f"dispensing {trial.aspirate_volume} uL at {trial.flow_rate_dispense} ul/sec"
        )
        press_disp = await _run_and_get_pressure(
            api.blow_out(trial.pipette.mount),
            pressure_file,
            api.is_simulator
        )
        print("delaying after dispensing...")
        press_disp_del = await _run_and_get_pressure(
            _delay(60 * 2, api.is_simulator),
            pressure_file,
            api.is_simulator
        )
        await api.retract(trial.pipette.mount)
        await _drop_tip(api, trial.pipette)
    except PipetteOverpressureError as e:
        print(e)
        await api.home(list(types.Axis.gantry_axes()))
        await _drop_tip(api, trial.pipette, after_fail=True)
    return [press_asp, press_asp_del, press_disp ,press_disp_del]


async def _reset_hardware(api: OT3API, pipette: PipetteSettings) -> None:
    await api.home([ax for ax in types.Axis.gantry_axes()])
    tip_state = await api.get_tip_presence_status(pipette.mount)
    if tip_state == types.TipStateType.PRESENT or api.is_simulator:
        await api.add_tip(
            pipette.mount, helpers_ot3.get_default_tip_length(pipette.tip)
        )
        await _drop_tip(api, pipette, after_fail=True)
    await api.home_plunger(pipette.mount)


def _build_default_trial(pipette: PipetteSettings) -> TrialSettings:
    return TrialSettings(
        pipette=pipette,
        aspirate_volume=0,
        submerge=DEFAULT_SUBMERGE_MM,
        flow_rate_aspirate=FLOW_RATE_SAFE[pipette.channels][pipette.volume][
            pipette.tip
        ],
        flow_rate_dispense=FLOW_RATE_SAFE[pipette.channels][pipette.volume][
            pipette.tip
        ],
    )


def _read_lines(file: Path, start: datetime, end: datetime) -> List[Tuple[datetime, float]]:
    with open(file, "r") as f:
        lines = f.readlines()
    ret: List[Tuple[datetime, float]] = []
    for line in lines:
        elements = [el.strip() for el in line.strip().split(",") if el]
        try:
            assert len(elements) == 2
            dtime = datetime.strptime(elements[0], hms)
            pressure = float(elements[1])
            if start <= dtime <= end:
                ret.append((dtime, pressure,))
        except Exception as e:
            print(e)
            continue
    print(ret)
    return ret


async def _run_test(
    api: OT3API, pipette: PipetteSettings, file: test_data.File, pressure_file: Path
) -> None:
    file.append("header,here")
    aspirate_volumes = TEST_ASPIRATE_VOLUME[pipette.channels][pipette.volume][
        pipette.tip
    ]
    flow_rates_aspirate = TEST_FLOW_RATE_ASPIRATE[pipette.channels][pipette.volume][
        pipette.tip
    ]
    flow_rates_dispense = TEST_FLOW_RATE_DISPENSE[pipette.channels][pipette.volume][
        pipette.tip
    ]
    print("aspirate_volumes")
    print(aspirate_volumes)
    print("flow_rates_aspirate")
    print(flow_rates_aspirate)
    print("flow_rates_dispense")
    print(flow_rates_dispense)
    # test aspirate flow-rates (per volume)
    trial = _build_default_trial(pipette)
    for flow_rate in flow_rates_aspirate:
        for volume in aspirate_volumes:
            trial.flow_rate_aspirate = flow_rate
            trial.aspirate_volume = volume
            trial_data = await _run_trial(api, trial, pressure_file)
            print(trial_data)
    # test dispense flow-rates (per volume)
    trial = _build_default_trial(pipette)
    for flow_rate in flow_rates_dispense:
        for volume in aspirate_volumes:
            trial.flow_rate_dispense = flow_rate
            trial.aspirate_volume = volume
            trial_data = await _run_trial(api, trial, pressure_file)
            print(trial_data)


async def _main(
    pressure_file: Path,
    is_simulating: bool,
    tip: int,
    offset_tip_rack: Point,
    offset_reservoir: Point,
) -> None:
    api = await helpers_ot3.build_async_ot3_hardware_api(
        is_simulating=is_simulating, pipette_left="p1000_multi_v3.5"
    )
    pipette = PipetteSettings.build(api, tip, offset_tip_rack, offset_reservoir)
    await _reset_hardware(api, pipette)

    pip_serial = helpers_ot3.get_pipette_serial_ot3(pipette.hw_pipette)
    file_results = test_data.create_file(
        test_name=TEST_NAME, tag=f"{pip_serial}-results", run_id=RUN_ID
    )
    await _run_test(api, pipette, file_results, pressure_file)


def _find_pressure_file() -> Path:
    # NOTE: this function relies on you already having started the
    found_paths: List[Path] = []
    for p in Path().resolve().iterdir():
        if p.is_file and "pressure_test_" in p.name and ".csv" in p.name:
            found_paths.append(p)
    if not len(found_paths):
        raise RuntimeError("unable to find pressure_test CSV")
    found_paths.sort(key=lambda path: path.stat().st_mtime)
    file = found_paths[-1]  # sorting by time, so biggest (newest) is at end
    print(f"reading from pressure file: {file.name}")
    return file


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--tip", type=int, required=True)
    parser.add_argument("--offset-tip-rack", nargs="+", type=float, default=[0, 0, 0])
    parser.add_argument("--offset-reservoir", nargs="+", type=float, default=[0, 0, 0])
    args = parser.parse_args()
    assert len(args.offset_tip_rack) == 3
    assert len(args.offset_reservoir) == 3
    asyncio.run(
        _main(
            _find_pressure_file(),
            args.simulate,
            args.tip,
            Point(*args.offset_tip_rack),
            Point(*args.offset_reservoir),
        )
    )
