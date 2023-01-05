"""Pipette Assembly QC Test."""
import argparse
import asyncio
from dataclasses import dataclass, fields
import os
from time import time
from typing import Optional, Callable, List, Any, Tuple
from typing_extensions import Final

from opentrons.hardware_control.ot3api import OT3API

from hardware_testing import data
from hardware_testing.drivers import list_ports_and_select
from hardware_testing.drivers.pressure_fixture import (
    PressureFixture,
    SimPressureFixture,
)
from hardware_testing.opentrons_api import helpers_ot3
from hardware_testing.opentrons_api.types import OT3Mount, Point

TEST_NAME: Final = "pipette-assembly-qc"
CSV_HEADER_FIXTURE: Final = "time,p1,p2,p3,p4,p5,p6,p7,p8,phase"

TRASH_HEIGHT_MM: Final = 45
LEAK_HOVER_ABOVE_LIQUID_MM: Final = 50

FIXTURE_LOCATION_A1_LEFT = Point(x=14.4, y=74.5, z=71.2)
FIXTURE_LOCATION_A1_RIGHT = FIXTURE_LOCATION_A1_LEFT._replace(x=128 - 14.4)
FIXTURE_TIP_VOLUME = 50
FIXTURE_ASPIRATE_VOLUME = FIXTURE_TIP_VOLUME
FIXTURE_PAUSE_AFTER_MOVEMENT_SECONDS = 1

FIXTURE_EVENT_TAG_PRE = "pressure-pre"
FIXTURE_EVENT_TAG_INSERT = "pressure-insert"
FIXTURE_EVENT_TAG_ASPIRATE = "pressure-aspirate"
FIXTURE_EVENT_TAG_DISPENSE = "pressure-dispense"
FIXTURE_EVENT_TAG_POST = "pressure-post"

FIXTURE_EVENT_STABILITY_THRESHOLD = 0.05
FIXTURE_EVENT_THRESHOLDS = {
    FIXTURE_EVENT_TAG_PRE: (
        -0.01,
        0.01,
    ),
    FIXTURE_EVENT_TAG_INSERT: (
        0.2,
        0.5,
    ),
    FIXTURE_EVENT_TAG_ASPIRATE: (
        -3.5,
        -2.5,
    ),
    FIXTURE_EVENT_TAG_DISPENSE: (
        0.2,
        0.5,
    ),
    FIXTURE_EVENT_TAG_POST: (
        -0.01,
        0.01,
    ),
}

SAFE_HEIGHT_TRAVEL = 10
SAFE_HEIGHT_CALIBRATE = 10


@dataclass
class TestConfig:
    """Test Configurations."""

    operator_name: str
    skip_liquid: bool
    skip_fixture: bool
    skip_diagnostics: bool
    fixture_port: str
    fixture_depth: int
    fixture_side: str
    fixture_sample_count: int
    fixture_sample_delay: float
    slot_tip_rack_liquid: int
    slot_tip_rack_fixture: int
    slot_reservoir: int
    slot_fixture: int
    slot_trash: int
    num_trials: int
    wait_seconds: int
    simulate: bool


@dataclass
class LabwareLocations:
    """Test Labware Locations."""

    trash: Optional[Point]
    tip_rack_liquid: Optional[Point]
    tip_rack_fixture: Optional[Point]
    reservoir: Optional[Point]
    fixture: Optional[Point]


# start with dummy values, these will be immediately overwritten
# we start with actual values here to pass linting
IDEAL_LABWARE_LOCATIONS: LabwareLocations = LabwareLocations(
    trash=None,
    tip_rack_liquid=None,
    tip_rack_fixture=None,
    reservoir=None,
    fixture=None,
)
CALIBRATED_LABWARE_LOCATIONS: LabwareLocations = LabwareLocations(
    trash=None,
    tip_rack_liquid=None,
    tip_rack_fixture=None,
    reservoir=None,
    fixture=None,
)


def _get_operator_answer_to_question(question: str) -> bool:
    user_inp = ""
    print("\n------------------------------------------------")
    while not user_inp or user_inp not in ["y", "n"]:
        user_inp = input(f"QUESTION: {question} (y/n): ").strip()
    print(f"ANSWER: {user_inp}")
    print("------------------------------------------------\n")
    return "y" in user_inp


def _get_ideal_labware_locations(
    test_config: TestConfig, pipette_volume: int
) -> LabwareLocations:
    tip_rack_liquid_loc_ideal = helpers_ot3.get_theoretical_a1_position(
        test_config.slot_tip_rack_liquid,
        f"opentrons_ot3_96_tiprack_{pipette_volume}ul",
    )
    tip_rack_fixture_loc_ideal = helpers_ot3.get_theoretical_a1_position(
        test_config.slot_tip_rack_fixture,
        f"opentrons_ot3_96_tiprack_{FIXTURE_TIP_VOLUME}ul",
    )
    reservoir_loc_ideal = helpers_ot3.get_theoretical_a1_position(
        test_config.slot_reservoir, "nest_1_reservoir_195ml"
    )
    # trash
    trash_loc_ideal = helpers_ot3.get_slot_calibration_square_position_ot3(
        test_config.slot_trash
    )
    trash_loc_ideal += Point(z=TRASH_HEIGHT_MM)
    # pressure fixture
    fixture_slot_pos = helpers_ot3.get_slot_bottom_left_position_ot3(
        test_config.slot_fixture
    )
    if test_config.fixture_side == "left":
        fixture_loc_ideal = fixture_slot_pos + FIXTURE_LOCATION_A1_LEFT
    else:
        fixture_loc_ideal = fixture_slot_pos + FIXTURE_LOCATION_A1_RIGHT
    return LabwareLocations(
        tip_rack_liquid=tip_rack_liquid_loc_ideal,
        tip_rack_fixture=tip_rack_fixture_loc_ideal,
        reservoir=reservoir_loc_ideal,
        trash=trash_loc_ideal,
        fixture=fixture_loc_ideal,
    )


def _tip_name_to_xy_offset(tip: str) -> Point:
    tip_rack_rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
    tip_row = tip_rack_rows.index(tip[0])
    tip_column = int(tip[1]) - 1
    return Point(x=tip_column * 9, y=tip_row * 9)


async def _move_to_or_calibrate(
    api: OT3API, mount: OT3Mount, expected: Optional[Point], actual: Optional[Point]
) -> Point:
    current_pos = await api.gantry_position(mount)
    if not actual:
        assert expected
        safe_expected = expected + Point(z=SAFE_HEIGHT_CALIBRATE)
        safe_height = max(safe_expected.z, current_pos.z) + SAFE_HEIGHT_TRAVEL
        await helpers_ot3.move_to_arched_ot3(
            api, mount, safe_expected, safe_height=safe_height
        )
        await helpers_ot3.jog_mount_ot3(api, mount, display=False)
        actual = await api.gantry_position(mount)
    else:
        safe_height = max(actual.z, current_pos.z) + SAFE_HEIGHT_TRAVEL
        await helpers_ot3.move_to_arched_ot3(
            api, mount, actual, safe_height=safe_height
        )
    return actual


async def _pick_up_tip(
    api: OT3API,
    mount: OT3Mount,
    tip: str,
    expected: Optional[Point],
    actual: Optional[Point],
    tip_volume: Optional[float] = None,
) -> Point:
    actual = await _move_to_or_calibrate(api, mount, expected, actual)
    tip_offset = _tip_name_to_xy_offset(tip)
    tip_pos = actual + tip_offset
    await helpers_ot3.move_to_arched_ot3(
        api, mount, tip_pos, safe_height=tip_pos.z + SAFE_HEIGHT_TRAVEL
    )
    if not tip_volume:
        tip_volume = api.hardware_pipettes[mount.to_mount()].working_volume
    tip_length = helpers_ot3.get_default_tip_length(tip_volume)
    await api.pick_up_tip(mount, tip_length=tip_length)
    return actual


async def _pick_up_tip_for_liquid(api: OT3API, mount: OT3Mount, tip: str) -> None:
    CALIBRATED_LABWARE_LOCATIONS.tip_rack_liquid = await _pick_up_tip(
        api,
        mount,
        tip,
        IDEAL_LABWARE_LOCATIONS.tip_rack_liquid,
        CALIBRATED_LABWARE_LOCATIONS.tip_rack_liquid,
    )


async def _pick_up_tip_for_fixture(api: OT3API, mount: OT3Mount, tip: str) -> None:
    CALIBRATED_LABWARE_LOCATIONS.tip_rack_fixture = await _pick_up_tip(
        api,
        mount,
        tip,
        IDEAL_LABWARE_LOCATIONS.tip_rack_fixture,
        CALIBRATED_LABWARE_LOCATIONS.tip_rack_fixture,
        tip_volume=FIXTURE_TIP_VOLUME,
    )


async def _move_to_liquid(api: OT3API, mount: OT3Mount) -> None:
    CALIBRATED_LABWARE_LOCATIONS.reservoir = await _move_to_or_calibrate(
        api,
        mount,
        IDEAL_LABWARE_LOCATIONS.reservoir,
        CALIBRATED_LABWARE_LOCATIONS.reservoir,
    )


async def _move_to_fixture(api: OT3API, mount: OT3Mount) -> None:
    CALIBRATED_LABWARE_LOCATIONS.fixture = await _move_to_or_calibrate(
        api,
        mount,
        IDEAL_LABWARE_LOCATIONS.fixture,
        CALIBRATED_LABWARE_LOCATIONS.fixture,
    )


async def _drop_tip_in_trash(api: OT3API, mount: OT3Mount) -> None:
    # assume the ideal is accurate enough
    ideal = IDEAL_LABWARE_LOCATIONS.trash
    current_pos = await api.gantry_position(mount)
    safe_height = max(ideal.z, current_pos.z) + SAFE_HEIGHT_TRAVEL
    await helpers_ot3.move_to_arched_ot3(api, mount, ideal, safe_height=safe_height)
    await api.drop_tip(mount, home_after=False)


async def _aspirate_and_look_for_droplets(
    api: OT3API, mount: OT3Mount, test_config: TestConfig
) -> bool:
    pipette_volume = api.hardware_pipettes[mount.to_mount()].working_volume
    print(f"aspirating {pipette_volume} microliters")
    await api.aspirate(mount, pipette_volume)
    await api.move_rel(mount, Point(z=LEAK_HOVER_ABOVE_LIQUID_MM))
    for t in range(test_config.wait_seconds):
        print(f"waiting for leaking tips ({t + 1}/{test_config.wait_seconds})")
        if not api.is_simulator:
            await asyncio.sleep(1)
    if api.is_simulator:
        leak_test_passed = True
    else:
        leak_test_passed = _get_operator_answer_to_question("did it pass? no leaking?")
    # TODO: save pass/fail to CSV
    print("dispensing back into reservoir")
    await api.move_rel(mount, Point(z=-LEAK_HOVER_ABOVE_LIQUID_MM))
    await api.dispense(mount, pipette_volume)
    await api.blow_out(mount)
    return leak_test_passed


async def _read_pressure_and_check_results(
    api: OT3API,
    test_config: TestConfig,
    fixture: PressureFixture,
    tag: str,
    write_cb: Callable,
) -> bool:
    if not api.is_simulator:
        # TODO: remove once firmware has been fixed so values update quicker
        await asyncio.sleep(FIXTURE_PAUSE_AFTER_MOVEMENT_SECONDS)
    _samples = []
    for i in range(test_config.fixture_sample_count):
        _samples.append(fixture.read_all_pressure_channel())
        _sample_as_strings = [str(round(p, 2)) for p in _samples[-1]]
        csv_data_sample = [tag] + _sample_as_strings
        print(csv_data_sample)
        write_cb(csv_data_sample)
        if not api.is_simulator and i < test_config.fixture_sample_count - 1:
            await asyncio.sleep(test_config.fixture_sample_delay)
    _samples_channel_1 = [s[0] for s in _samples]
    _samples_channel_1.sort()
    _samples_clipped = _samples_channel_1[1:-1]
    _samples_min = min(_samples_clipped)
    _samples_max = max(_samples_clipped)
    if _samples_max - _samples_min > FIXTURE_EVENT_STABILITY_THRESHOLD:
        test_pass_stability = False
    else:
        test_pass_stability = True
    csv_data_stability = [tag + "-stability", "PASS" if test_pass_stability else "FAIL"]
    print(csv_data_stability)
    write_cb(csv_data_stability)
    thresholds = FIXTURE_EVENT_THRESHOLDS[tag]
    if _samples_min < thresholds[0] or _samples_max > thresholds[1]:
        test_pass_accuracy = False
    else:
        test_pass_accuracy = True
    csv_data_accuracy = [tag + "-accuracy", "PASS" if test_pass_accuracy else "FAIL"]
    print(csv_data_accuracy)
    write_cb(csv_data_accuracy)
    return test_pass_stability and test_pass_accuracy


async def _fixture_check_pressure(
    api: OT3API,
    mount: OT3Mount,
    test_config: TestConfig,
    write_cb: Callable,
    fixture: PressureFixture,
) -> bool:
    results = []
    # above the fixture
    r = await _read_pressure_and_check_results(
        api, test_config, fixture, FIXTURE_EVENT_TAG_PRE, write_cb
    )
    results.append(r)
    # insert into the fixture
    await api.move_rel(mount, Point(z=-test_config.fixture_depth))
    r = await _read_pressure_and_check_results(
        api, test_config, fixture, FIXTURE_EVENT_TAG_INSERT, write_cb
    )
    results.append(r)
    # aspirate 50uL
    await api.aspirate(mount, FIXTURE_ASPIRATE_VOLUME)
    r = await _read_pressure_and_check_results(
        api, test_config, fixture, FIXTURE_EVENT_TAG_ASPIRATE, write_cb
    )
    results.append(r)
    # dispense
    await api.dispense(mount, FIXTURE_ASPIRATE_VOLUME)
    r = await _read_pressure_and_check_results(
        api, test_config, fixture, FIXTURE_EVENT_TAG_DISPENSE, write_cb
    )
    results.append(r)
    # retract out of fixture
    await api.move_rel(mount, Point(z=test_config.fixture_depth))
    r = await _read_pressure_and_check_results(
        api, test_config, fixture, FIXTURE_EVENT_TAG_POST, write_cb
    )
    results.append(r)
    return False not in results


async def _test_for_leak(
    api: OT3API,
    mount: OT3Mount,
    test_config: TestConfig,
    tip: str,
    fixture: Optional[PressureFixture],
    write_cb: Optional[Callable],
) -> bool:
    if fixture:
        await _pick_up_tip_for_fixture(api, mount, tip)
        assert write_cb, "pressure fixture requires recording data to disk"
        await _move_to_fixture(api, mount)
        test_passed = await _fixture_check_pressure(
            api, mount, test_config, write_cb=write_cb, fixture=fixture
        )
    else:
        await _pick_up_tip_for_liquid(api, mount, tip)
        await _move_to_liquid(api, mount)
        test_passed = await _aspirate_and_look_for_droplets(api, mount, test_config)
    await _drop_tip_in_trash(api, mount)
    pass_msg = "PASS" if test_passed else "FAIL"
    print(f"tip {tip}: {pass_msg}")
    return test_passed


async def _test_for_leak_by_eye(
    api: OT3API, mount: OT3Mount, test_config: TestConfig, tip: str
) -> bool:
    return await _test_for_leak(api, mount, test_config, tip, None, None)


def _connect_to_fixture(test_config: TestConfig) -> PressureFixture:
    if not test_config.simulate and not test_config.skip_fixture:
        if not test_config.fixture_port:
            _port = list_ports_and_select("pressure-fixture")
        else:
            _port = ""
        fixture = PressureFixture.create(port=_port, slot_side=test_config.fixture_side)
    else:
        fixture = SimPressureFixture()  # type: ignore[assignment]
    fixture.connect()
    return fixture


async def _main(test_config: TestConfig) -> None:
    global IDEAL_LABWARE_LOCATIONS
    global CALIBRATED_LABWARE_LOCATIONS

    # connect to the pressure fixture (or simulate one)
    fixture = _connect_to_fixture(test_config)

    # create API instance, and get Pipette serial number
    api = await helpers_ot3.build_async_ot3_hardware_api(
        is_simulating=test_config.simulate,
        pipette_left="p1000_single_v3.3",
        pipette_right="p1000_single_v3.3",
    )
    pips = {OT3Mount.from_mount(m): p for m, p in api.hardware_pipettes.items() if p}
    assert pips, "no pipettes attached"
    for mount, pipette in pips.items():
        pipette_sn = helpers_ot3.get_pipette_serial_ot3(pipette)
        print(f"Pipette: {pipette_sn} on the {mount.name} mount")
        if not api.is_simulator and not _get_operator_answer_to_question(
            "qc this pipette?"
        ):
            continue

        # setup our labware locations
        pipette_volume = int(pipette.working_volume)
        IDEAL_LABWARE_LOCATIONS = _get_ideal_labware_locations(
            test_config, pipette_volume
        )
        CALIBRATED_LABWARE_LOCATIONS = LabwareLocations(
            trash=None,
            tip_rack_liquid=None,
            tip_rack_fixture=None,
            reservoir=None,
            fixture=None,
        )

        # create the CSV file, using the Pipette serial number as the tag
        run_id = data.create_run_id()
        folder_path = data.create_folder_for_test_data(TEST_NAME)
        file_name = data.create_file_name(TEST_NAME, run_id, pipette_sn)
        print(f"CSV: {os.path.join(folder_path, file_name)}")
        start_time = time()

        # callback function for writing new data to CSV file
        def _append_csv_data(data_list: List[Any]) -> None:
            # prepend the elapsed seconds, so the time is always in the first column
            elapsed_seconds = round(time() - start_time, 2)
            data_list_with_time = [elapsed_seconds] + data_list
            data_str = ",".join([str(d) for d in data_list_with_time])
            data.append_data_to_file(TEST_NAME, file_name, data_str + "\n")

        # add metadata to CSV
        _append_csv_data(["--------"])
        _append_csv_data(["METADATA"])
        _append_csv_data(["--------"])
        if test_config.simulate:
            _append_csv_data(["simulating"])
        _append_csv_data(["test-name", TEST_NAME])
        _append_csv_data(["operator-name", test_config.operator_name])
        _append_csv_data(["run-id", run_id])  # includes a date/time string
        _append_csv_data(["pipette", pipette_sn])
        # add test configurations to CSV
        _append_csv_data(["--------------"])
        _append_csv_data(["CONFIGURATIONS"])
        _append_csv_data(["--------------"])
        for f in fields(test_config):
            _append_csv_data([f.name, getattr(test_config, f.name)])
        # add pressure thresholds to CSV
        _append_csv_data(["----------"])
        _append_csv_data(["THRESHOLDS"])
        _append_csv_data(["----------"])
        _append_csv_data(["pressure-stability", FIXTURE_EVENT_STABILITY_THRESHOLD])
        for tag, threshold in FIXTURE_EVENT_THRESHOLDS.items():
            _append_csv_data([tag, threshold[0], threshold[1]])

        # run the test
        _append_csv_data(["-----"])
        _append_csv_data(["BEGIN"])
        _append_csv_data(["-----"])
        print("homing")
        await api.home()
        if not test_config.skip_liquid:
            tips_liquid = [f"A{i + 1}" for i in range(test_config.num_trials)]
            for tip in tips_liquid:
                test_passed = await _test_for_leak_by_eye(api, mount, test_config, tip)
                _append_csv_data(
                    ["droplet-test", tip, "PASS" if test_passed else "FAIL"]
                )
        if not test_config.skip_fixture:
            tips_fixture = [f"A{i + 1}" for i in range(test_config.num_trials)]
            for tip in tips_fixture:
                test_passed = await _test_for_leak(
                    api,
                    mount,
                    test_config,
                    tip,
                    fixture=fixture,
                    write_cb=_append_csv_data,
                )
                _append_csv_data(
                    ["pressure-test", tip, "PASS" if test_passed else "FAIL"]
                )

        _append_csv_data(["---"])
        _append_csv_data(["END"])
        _append_csv_data(["---"])
        print("test complete")
        print(f"CSV: {os.path.join(folder_path, file_name)}")
        print("homing")
        await api.home()


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="OT-3 Pipette Assembly QC Test")
    arg_parser.add_argument("--operator", type=str, required=True)
    arg_parser.add_argument("--skip-liquid", action="store_true")
    arg_parser.add_argument("--skip-fixture", action="store_true")
    arg_parser.add_argument("--skip-diagnostics", action="store_true")
    arg_parser.add_argument("--fixture-side", choices=["left", "right"], default="left")
    arg_parser.add_argument("--port", type=str, default="")
    arg_parser.add_argument("--num-trials", type=int, default=2)
    arg_parser.add_argument("--sample-count", type=float, default=10)
    arg_parser.add_argument("--sample-delay", type=float, default=0.25)
    arg_parser.add_argument("--wait", type=int, default=30)
    arg_parser.add_argument("--slot-tip-rack-liquid", type=int, default=7)
    arg_parser.add_argument("--slot-tip-rack-fixture", type=int, default=1)
    arg_parser.add_argument("--slot-reservoir", type=int, default=8)
    arg_parser.add_argument("--slot-fixture", type=int, default=2)
    arg_parser.add_argument("--slot-trash", type=int, default=12)
    arg_parser.add_argument("--insert-depth", type=int, default=14)
    arg_parser.add_argument("--simulate", action="store_true")
    args = arg_parser.parse_args()
    _cfg = TestConfig(
        operator_name=args.operator,
        skip_liquid=args.skip_liquid,
        skip_fixture=args.skip_fixture,
        skip_diagnostics=args.skip_diagnostics,
        fixture_port=args.port,
        fixture_depth=args.insert_depth,
        fixture_side=args.fixture_side,
        fixture_sample_count=args.sample_count,
        fixture_sample_delay=args.sample_delay,
        slot_tip_rack_liquid=args.slot_tip_rack_liquid,
        slot_tip_rack_fixture=args.slot_tip_rack_fixture,
        slot_reservoir=args.slot_reservoir,
        slot_fixture=args.slot_fixture,
        slot_trash=args.slot_trash,
        num_trials=args.num_trials,
        wait_seconds=args.wait,
        simulate=args.simulate,
    )
    asyncio.run(_main(_cfg))
