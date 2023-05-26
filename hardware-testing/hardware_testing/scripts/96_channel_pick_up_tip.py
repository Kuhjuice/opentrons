"""Demo OT3 Gantry Functionality."""
import argparse
import ast
import asyncio
import csv
import time
from typing import Tuple, Dict, Optional
from threading import Thread
import datetime
import os
import sys
import termios
import tty
import json

from opentrons.hardware_control.motion_utilities import target_position_from_plunger
from hardware_testing.opentrons_api.types import (
    OT3Mount,
    OT3Axis,
    Point,
    CriticalPoint,
)
from hardware_testing.opentrons_api.helpers_ot3 import (
    build_async_ot3_hardware_api,
    home_ot3,
    move_plunger_absolute_ot3,
    move_plunger_relative_ot3,
    get_plunger_positions_ot3,
    update_pick_up_current,
    update_pick_up_distance,
    _get_pipette_from_mount,
)

from opentrons.config.types import LiquidProbeSettings

from hardware_testing import data
from hardware_testing.drivers.mark10 import Mark10
from hardware_testing.drivers import mitutoyo_digimatic_indicator

aspirate_depth = 7
dispense_depth = 3
liquid_retract_dist = 12
liquid_retract_speed = 5
retract_dist = 100
retract_speed = 60

leak_test_time = 30


def dict_keys_to_line(dict):
    return str.join(",", list(dict.keys())) + "\n"


def file_setup(test_data, details):
    today = datetime.date.today()
    test_name = "{}-pick_up-up-test-{}Amps".format(
        details[0],  # Pipette model
        details[1],  # Motor Current
    )
    test_header = dict_keys_to_line(test_data)
    test_tag = "-{}".format(today.strftime("%b-%d-%Y"))
    test_id = data.create_run_id()
    test_path = data.create_folder_for_test_data(test_name)
    test_file = data.create_file_name(test_name, test_id, test_tag)
    data.append_data_to_file(test_name, test_file, test_header)
    print("FILE PATH = ", test_path)
    print("FILE NAME = ", test_file)
    return test_name, test_file


def dial_indicator_setup(port):
    gauge = mitutoyo_digimatic_indicator.Mitutoyo_Digimatic_Indicator(port=port)
    gauge.connect()
    return gauge


def getch():
    """
    fd: file descriptor stdout, stdin, stderr
    This functions gets a single input keyboard character from the user
    """

    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    return _getch()


async def jog(api, position, cp) -> Dict[OT3Axis, float]:
    step_size = [0.01, 0.05, 0.1, 0.5, 1, 10, 20, 50]
    step_length_index = 3
    step = step_size[step_length_index]
    xy_speed = 60
    za_speed = 65
    information_str = """
        Click  >>   i   << to move up
        Click  >>   k   << to move down
        Click  >>   a  << to move left
        Click  >>   d  << to move right
        Click  >>   w  << to move forward
        Click  >>   s  << to move back
        Click  >>   +   << to Increase the length of each step
        Click  >>   -   << to decrease the length of each step
        Click  >> Enter << to save position
        Click  >> q << to quit the test script
                    """
    print(information_str)
    while True:
        input = getch()
        if input == "a":
            # minus x direction
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(-step_size[step_length_index], 0, 0), speed=xy_speed
            )

        elif input == "d":
            # plus x direction
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(step_size[step_length_index], 0, 0), speed=xy_speed
            )

        elif input == "w":
            # minus y direction
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(0, step_size[step_length_index], 0), speed=xy_speed
            )

        elif input == "s":
            # plus y direction
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(0, -step_size[step_length_index], 0), speed=xy_speed
            )

        elif input == "i":
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(0, 0, step_size[step_length_index]), speed=za_speed
            )

        elif input == "k":
            sys.stdout.flush()
            await api.move_rel(
                mount, Point(0, 0, -step_size[step_length_index]), speed=za_speed
            )

        elif input == "q":
            sys.stdout.flush()
            print("TEST CANCELLED")
            quit()

        elif input == "+":
            sys.stdout.flush()
            step_length_index = step_length_index + 1
            if step_length_index >= 7:
                step_length_index = 7
            step = step_size[step_length_index]

        elif input == "-":
            sys.stdout.flush()
            step_length_index = step_length_index - 1
            if step_length_index <= 0:
                step_length_index = 0
            step = step_size[step_length_index]

        elif input == "\r":
            sys.stdout.flush()
            position = await api.current_position_ot3(
                mount, refresh=True, critical_point=cp
            )
            print("\r\n")
            return position
        position = await api.current_position_ot3(
            mount, refresh=True, critical_point=cp
        )

        print(
            "Coordinates: ",
            round(position[OT3Axis.X], 2),
            ",",
            round(position[OT3Axis.Y], 2),
            ",",
            round(position[OT3Axis.by_mount(mount)], 2),
            " Motor Step: ",
            step_size[step_length_index],
            end="",
        )
        print("\r", end="")


async def countdown(count_time: float):
    """
    This function loops through a countdown before checking the leak visually
    """
    time_suspend = 0
    while time_suspend < count_time:
        await asyncio.sleep(1)
        time_suspend += 1
        print(f"Remaining: {count_time-time_suspend} (s)", end="")
        print("\r", end="")
    print("")


async def update_pickup_tip_speed(api, mount, speed) -> None:
    """Update drop-tip current."""
    pipette = _get_pipette_from_mount(api, mount)
    config_model = pipette.pick_up_configurations
    config_model.speed = speed
    pipette.pick_up_configurations = config_model
    print(pipette.pick_up_configurations)

async def move_to_point(api, mount, point, cp):
    home_pos = api.get_instrument_max_height(mount, cp)
    pos = await api.current_position_ot3(mount, refresh=True, critical_point = cp)
    await api.move_to(mount,
                    Point(pos[OT3Axis.X],
                        pos[OT3Axis.Y],
                        home_pos))
    await api.move_to(mount,
                    Point(point.x,
                        point.y,
                        home_pos))
    await api.move_to(mount,
                    Point(point.x,
                        point.y,
                        point.z))

async def _main() -> None:
    today = datetime.date.today()
    tips_to_use = 12
    slot_loc = {
        "A1": (13.42, 394.92, 110),
        "A2": (177.32, 394.92, 110),
        "A3": (341.03, 394.0, 110),
        "B1": (13.42, 288.42, 110),
        "B2": (177.32, 288.92, 110),
        "B3": (341.03, 288.92, 110),
        "C1": (13.42, 181.92, 110),
        "C2": (177.32, 181.92, 110),
        "C3": (341.03, 181.92, 110),
        "D1": (13.42, 75.5, 110),
        "D2": (177.32, 75.5, 110),
        "D3": (341.03, 75.5, 110),
    }
    hw_api = await build_async_ot3_hardware_api(
        is_simulating=args.simulate, use_defaults=True
    )
    tip_length = {"T1K": 85.7, "T200": 48.35, "T50": 47.9}
    pipette_model = hw_api.get_all_attached_instr()[OT3Mount.LEFT]["pipette_id"]
    dial_data = {
        "Ch1": None, "Ch2": None, "Ch3": None, "Ch4": None, "Ch5": None, "Ch6": None,
        "Ch7": None, "Ch8": None, "Ch9": None, "Ch10": None, "Ch11": None, "Ch12": None,
        "Ch13": None, "Ch14": None, "Ch15": None, "Ch16": None, "Ch17": None, "Ch18": None,
        "Ch19": None, "Ch20": None, "Ch21": None, "Ch22": None, "Ch23": None, "Ch24": None,
        "Ch25": None, "Ch26": None, "Ch27": None, "Ch28": None, "Ch29": None, "Ch30": None,
        "Ch31": None, "Ch32": None, "Ch33": None, "Ch34": None, "Ch35": None, "Ch36": None,
        "Ch37": None, "Ch38": None, "Ch39": None, "Ch40": None, "Ch41": None, "Ch42": None,
        "Ch43": None, "Ch44": None, "Ch45": None, "Ch46": None, "Ch47": None, "Ch48": None,
        "Ch49": None, "Ch50": None, "Ch51": None, "Ch52": None, "Ch53": None, "Ch54": None,
        "Ch55": None, "Ch56": None, "Ch57": None, "Ch58": None, "Ch59": None, "Ch60": None,
        "Ch61": None, "Ch62": None, "Ch63": None, "Ch64": None, "Ch65": None, "Ch66": None,
        "Ch67": None, "Ch68": None, "Ch69": None, "Ch70": None, "Ch71": None, "Ch72": None,
        "Ch73": None, "Ch74": None, "Ch75": None, "Ch76": None, "Ch77": None, "Ch78": None,
        "Ch79": None, "Ch80": None, "Ch81": None, "Ch82": None, "Ch83": None, "Ch84": None,
        "Ch85": None, "Ch86": None, "Ch87": None, "Ch88": None, "Ch89": None, "Ch90": None,
        "Ch91": None, "Ch92": None, "Ch93": None, "Ch94": None, "Ch95": None, "Ch96": None,
        "Motor Current": None,
        "Trial": None,
        "pipette_model": None,
    }
    m_current = float(input("motor_current in amps: "))
    pick_up_speed = float(input("pick up tip speed in mm/s: "))
    details = [pipette_model, m_current]
    test_n, test_f = file_setup(dial_data, details)
    file_name = "/home/root/.opentrons/testing_data/enc_data/enc_pu_test_%s-%s.csv" % (
        m_current,
        datetime.datetime.now().strftime("%m-%d-%y_%H-%M"),
    )
    print(file_name)
    print(test_n)
    print(test_f)
    await home_ot3(hw_api, [OT3Axis.Z_L, OT3Axis.Z_R, OT3Axis.X, OT3Axis.Y])
    await hw_api.home_plunger(mount)
    await hw_api.set_lights(rails=True)
    plunger_pos = get_plunger_positions_ot3(hw_api, mount)
    home_position = await hw_api.current_position_ot3(mount)
    start_time = time.perf_counter()
    # if args.tip_size == "T1K":
    #     home_with_tip_position = 164.3  # T1K
    # if args.tip_size == "T200":
    #     home_with_tip_position = 201.64999999999998  # T1K
    # elif args.tip_size == "T50":
    #     home_with_tip_position = 192.1  # T50

    if args.tiprack:
        cp = CriticalPoint.NOZZLE
        tiprack_loc = Point(
                        slot_loc["B2"][0],
                        slot_loc["B2"][1],
                        home_position[OT3Axis.by_mount(mount)])
        print("Move to Tiprack")
        await move_to_point(hw_api, mount, tiprack_loc, cp)
        current_position = await hw_api.current_position_ot3(mount)
        tiprack_loc = await jog(hw_api, current_position, cp)
        tiprack_loc = Point(tiprack_loc[OT3Axis.X],
                            tiprack_loc[OT3Axis.Y],
                            tiprack_loc[OT3Axis.by_mount(mount)])
        # Start recording the encoder
        init_tip_loc = await hw_api.encoder_current_position_ot3(
            mount, CriticalPoint.NOZZLE
        )
        print(f"Start encoder: {init_tip_loc}")
        init_tip_loc = init_tip_loc[OT3Axis.by_mount(mount)]
        encoder_position = init_tip_loc
        init_tip_loc = await hw_api.encoder_current_position_ot3(
            mount, CriticalPoint.NOZZLE
        )
        await update_pick_up_current(hw_api, mount, m_current)
        await update_pickup_tip_speed(hw_api, mount, pick_up_speed)
        # Move pipette to Force Gauge press location
        final_tip_loc = await hw_api.pick_up_tip(
            mount, tip_length=tip_length[args.tip_size]
        )
        # move_to_point(hw_api, mount, tiprack_loc, cp)
        await home_ot3(hw_api, [OT3Axis.by_mount(mount)])
        home_with_tip_position = await hw_api.current_position_ot3(
            mount, critical_point=CriticalPoint.TIP
        )
        # await hw_api.move_to(
        #     mount,
        #     Point(
        #         current_position[OT3Axis.X],
        #         current_position[OT3Axis.Y],
        #         home_with_tip_position[OT3Axis.by_mount(mount)],
        #     ),
        #     critical_point=CriticalPoint.TIP,
        # )
        encoder_end = final_tip_loc[OT3Axis.by_mount(mount)]
        # final_tip_loc = await hw_api.encoder_current_position_ot3(mount, CriticalPoint.NOZZLE)
        print(f"End Encoder: {final_tip_loc}")
        final_tip_loc = final_tip_loc[OT3Axis.by_mount(mount)]

    # if args.dial_indicator:
    #     cp = CriticalPoint.TIP
    #     dial_loc = Point(
    #                     slot_loc["C2"][0],
    #                     slot_loc["C2"][1],
    #                     home_with_tip_position[OT3Axis.by_mount(mount)])
    #     print("Move to Tiprack")
    #     await move_to_point(hw_api, mount, dial_loc, cp)
    #     print("Move to Tiprack")
    #     current_position = await hw_api.current_position_ot3(mount)
    #     dial_loc = await jog(hw_api, current_position, cp)
    #     dial_loc = [
    #         dial_loc[OT3Axis.X],
    #         dial_loc[OT3Axis.Y],
    #         dial_loc[OT3Axis.by_mount(mount)],
    #     ]
    #     tip_offset = 0
    #     measurements = ""
    #     tips = 96
    #     trial = 1
    #     for tip in range(1, tips + 1):
    #         await hw_api.move_to(
    #             mount,
    #             Point(
    #                 dial_loc[0],
    #                 dial_loc[1] + tip_offset,
    #                 dial_loc[2],
    #             ),
    #         )
    #         await asyncio.sleep(1)
    #         tip_measurement = gauge.read()
    #         await asyncio.sleep(2)
    #         measurements += str(tip_measurement) + ","
    #
    #         await hw_api.move_to(
    #             mount,
    #             Point(
    #                 dial_loc[0],
    #                 dial_loc[1] + tip_offset,
    #                 home_with_tip_position[OT3Axis.by_mount(mount)],
    #             ),
    #         )
    #         tip_offset += 9
    #         await hw_api.move_to(
    #             mount,
    #             Point(
    #                 dial_loc[0],
    #                 dial_loc[1] + tip_offset,
    #                 home_with_tip_position[OT3Axis.by_mount(mount)],
    #             ),
    #         )
    #     d_str = f"{measurements} {pipette_model} , {trial}, {m_current} \n"
    #     data.append_data_to_file(test_n, test_f, d_str)

    if args.trough:
        await hw_api.move_to(
            mount,
            Point(
                slot_loc["B3"][0],
                slot_loc["B3"][1],
                home_with_tip_position[OT3Axis.by_mount(mount)],
            ),
        )
        cp = CriticalPoint.TIP
        print("Move  to Trough")
        current_position = await hw_api.current_position_ot3(mount)
        trough_loc = await jog(hw_api, current_position, cp)
        trough_loc = [
            trough_loc[OT3Axis.X],
            trough_loc[OT3Axis.Y],
            trough_loc[OT3Axis.by_mount(mount)],
        ]
        await hw_api.move_to(
            mount,
            Point(
                trough_loc[0],
                trough_loc[1],
                home_with_tip_position[OT3Axis.by_mount(mount)],
            ),
            critical_point=CriticalPoint.TIP,
        )
        # Move to trash slot
        await hw_api.move_to(
            mount,
            Point(
                slot_loc["A3"][0] + 50,
                slot_loc["A3"][1],
                home_with_tip_position[OT3Axis.by_mount(mount)],
            ),
            critical_point=CriticalPoint.TIP,
        )
        input("Feel the Tip!")
        await hw_api.drop_tip(mount)

    lp_file_name = "/var/pressure_sensor_data_P-{}_Z-{}-{}.csv".format(
        args.plunger_speed, args.mount_speed, today.strftime("%b-%d-%Y")
    )
    liquid_probe_settings = LiquidProbeSettings(
        max_z_distance=args.max_z_distance,
        min_z_distance=args.min_z_distance,
        mount_speed=args.mount_speed,
        plunger_speed=args.plunger_speed,
        sensor_threshold_pascals=args.sensor_threshold,
        expected_liquid_height=args.expected_liquid_height,
        log_pressure=args.log_pressure,
        aspirate_while_sensing=False,
        auto_zero_sensor=False,
        num_baseline_reads=10,
        data_file=lp_file_name,
    )
    tip_count = 0
    x_offset = 0
    y_offset = 0
    try:

        while True:
            # -----------------------Tiprack------------------------------------
            m_current = float(input("motor_current in amps: "))
            pick_up_speed = float(input("pick up tip speed in mm/s: "))
            cp = CriticalPoint.NOZZLE
            # Move over to the TipRack location and
            print("Move to Tiprack")
            await move_to_point(hw_api, mount, tiprack_loc, cp)

            # Start recording the encoder
            init_tip_loc = await hw_api.encoder_current_position_ot3(
                mount, CriticalPoint.NOZZLE
            )
            print(f"Start encoder: {init_tip_loc}")
            init_tip_loc = init_tip_loc[OT3Axis.by_mount(mount)]
            encoder_position = init_tip_loc
            # Press Pipette into the tip
            await update_pick_up_current(hw_api, mount, m_current)
            # Move pipette to Force Gauge press location
            final_tip_loc = await hw_api.pick_up_tip(
                mount, tip_length=tip_length[args.tip_size]
            )
            encoder_end = final_tip_loc[OT3Axis.by_mount(mount)]
            # final_tip_loc = await hw_api.encoder_current_position_ot3(mount, CriticalPoint.NOZZLE)
            print(f"End Encoder: {final_tip_loc}")
            final_tip_loc = final_tip_loc[OT3Axis.by_mount(mount)]
            test_details = [
                start_time,
                m_current,
                location,
                init_tip_loc,
                final_tip_loc,
                tip_count,
            ]
            enc_record(file_name, test_details)
            # Home Z
            await hw_api.home([OT3Axis.by_mount(mount)])
            cp = CriticalPoint.TIP
            current_position = await hw_api.current_position_ot3(mount, cp)
            # --------------------------Dial Indicator-----------------------
            # Move over to the dial indicator
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         dial_loc[0],
            #         dial_loc[1],
            #         home_with_tip_position[OT3Axis.by_mount(mount)],
            #     ),
            # )
            # tip_offset = 0
            # measurements = ""
            # tips = 8
            # for t in range(1, tips + 1):
            #     # Move over to the dial indicator
            #     await hw_api.move_to(
            #         mount,
            #         Point(dial_loc[0], dial_loc[1] + tip_offset, dial_loc[2]),
            #     )
            #     await asyncio.sleep(3)
            #     tip_measurement = gauge.read()
            #     measurements += str(tip_measurement) + ","
            #     await asyncio.sleep(1)
            #     # Move over to the dial indicator
            #     await hw_api.move_to(
            #         mount,
            #         Point(
            #             dial_loc[0],
            #             dial_loc[1] + tip_offset,
            #             home_with_tip_position[OT3Axis.by_mount(mount)],
            #         ),
            #     )
            #     tip_offset += 9
            #     await hw_api.move_to(
            #         mount,
            #         Point(
            #             dial_loc[0],
            #             dial_loc[1] + tip_offset,
            #             home_with_tip_position[OT3Axis.by_mount(mount)],
            #         ),
            #     )
            # d_str = f"{measurements} {pipette_model} , {trial}, {m_current} \n"
            # data.append_data_to_file(test_n, test_f, d_str)
            # # -----------------------Aspirate-----------------------------------
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         trough_loc[0],
            #         trough_loc[1],
            #         home_with_tip_position[OT3Axis.by_mount(mount)],
            #     ),
            # )
            # # Move to offset from trough
            # await hw_api.move_to(
            #     mount, Point(trough_loc[0], trough_loc[1], trough_loc[2])
            # )
            # # Move the plunger to the top position
            # await move_plunger_absolute_ot3(hw_api, mount, plunger_pos[0])
            # # Liquid Probe
            # liquid_height, enc_liquid_height = await hw_api.liquid_probe(
            #     mount, probe_settings=liquid_probe_settings
            # )
            #
            # liquid_height = await hw_api.current_position_ot3(
            #     mount, critical_point=CriticalPoint.TIP
            # )
            # # await move_plunger_relative_ot3(hw_api, mount, 1.5, None, speed = 2) # P50S
            # await move_plunger_relative_ot3(
            #     hw_api, mount, 0.25, None, speed=2
            # )  # P1KS
            # await hw_api.move_to(
            #     mount, Point(trough_loc[0], trough_loc[1], trough_loc[2])
            # )
            # # Prepare to aspirate before descending to trough well
            # await hw_api.prepare_for_aspirate(mount)
            # # Descend to aspirate depth
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         liquid_height[OT3Axis.X],
            #         liquid_height[OT3Axis.Y],
            #         liquid_height[OT3Axis.by_mount(mount)] - aspirate_depth,
            #     ),
            #     speed=5,
            #     critical_point=CriticalPoint.TIP,
            # )
            # # Aspirate
            # await hw_api.aspirate(mount, volume=50)
            # cur_pos = await hw_api.current_position_ot3(
            #     mount, critical_point=CriticalPoint.TIP
            # )
            # z_pos = cur_pos[OT3Axis.by_mount(mount)]
            # # Retract from liquid with retract speed
            # await hw_api.move_to(
            #     mount,
            #     Point(trough_loc[0], trough_loc[1], z_pos + liquid_retract_dist),
            #     speed=liquid_retract_speed,
            #     critical_point=CriticalPoint.TIP,
            # )
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         trough_loc[0],
            #         trough_loc[1],
            #         home_with_tip_position[OT3Axis.by_mount(mount)],
            #     ),
            #     critical_point=CriticalPoint.TIP,
            # )
            # await countdown(count_time=leak_test_time)
            # # input("Check to see if the pipette is leaking")
            # await hw_api.move_to(
            #     mount,
            #     Point(trough_loc[0], trough_loc[1], trough_loc[2]),
            #     critical_point=CriticalPoint.TIP,
            # )
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         trough_loc[0],
            #         trough_loc[1],
            #         liquid_height[OT3Axis.by_mount(mount)] - dispense_depth,
            #     ),
            #     speed=5,
            #     critical_point=CriticalPoint.TIP,
            # )
            # await hw_api.dispense(mount)
            # await hw_api.blow_out(mount)
            # --------------------Drop Tip--------------------------------------
            # current_position = await hw_api.current_position_ot3(
            #     mount, critical_point=CriticalPoint.TIP
            # )
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         trough_loc[0],
            #         trough_loc[1],
            #         home_with_tip_position[OT3Axis.by_mount(mount)],
            #     ),
            #     critical_point=CriticalPoint.TIP,
            # )
            # # Move to trash slot
            # await hw_api.move_to(
            #     mount,
            #     Point(
            #         slot_loc["A3"][0] + 50,
            #         slot_loc["A3"][1],
            #         home_with_tip_position[OT3Axis.by_mount(mount)],
            #     ),
            #     critical_point=CriticalPoint.TIP,
            # )
            # await hw_api.drop_tip(mount)

        await hw_api.disengage_axes([OT3Axis.X, OT3Axis.Y, OT3Axis.Z_L, OT3Axis.Z_R])
    except KeyboardInterrupt:
        await hw_api.disengage_axes([OT3Axis.X, OT3Axis.Y, OT3Axis.Z_L, OT3Axis.Z_R])
    finally:
        await hw_api.disengage_axes([OT3Axis.X, OT3Axis.Y, OT3Axis.Z_L, OT3Axis.Z_R])
        await hw_api.clean_up()

if __name__ == "__main__":
    slot_locs = [
        "A1",
        "A2",
        "A3",
        "B1",
        "B2",
        "B3:",
        "C1",
        "C2",
        "C3",
        "D1",
        "D2",
        "D3",
    ]
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--fg_jog", action="store_true")
    parser.add_argument("--trough", action="store_true")
    parser.add_argument("--tiprack", action="store_true")
    parser.add_argument("--mount", type=str, choices=["left", "right"], default="left")
    parser.add_argument("--tiprack_slot", type=str, choices=slot_locs, default="B2")
    parser.add_argument("--dial_slot", type=str, choices=slot_locs, default="C1")
    parser.add_argument("--trough_slot", type=str, choices=slot_locs, default="B3")
    parser.add_argument("--fg", action="store_true")
    parser.add_argument("--dial_indicator", action="store_true")
    parser.add_argument("--tip_size", type=str, default="T1K", help="Tip Size")
    parser.add_argument("--max_z_distance", type=float, default=40)
    parser.add_argument("--min_z_distance", type=float, default=5)
    parser.add_argument("--mount_speed", type=float, default=5)
    parser.add_argument("--plunger_speed", type=float, default=11)
    parser.add_argument(
        "--sensor_threshold", type=float, default=200, help="Threshold in Pascals"
    )
    parser.add_argument("--expected_liquid_height", type=int, default=0)
    parser.add_argument("--log_pressure", action="store_true")
    # parser.add_argument(
    #     "--fg_port", type=str, default="/dev/ttyUSB0", help="Force Gauge Port"
    # )
    parser.add_argument(
        "--dial_port", type=str, default="/dev/ttyUSB0", help="Dial indicator Port"
    )
    args = parser.parse_args()
    if args.mount == "left":
        mount = OT3Mount.LEFT
    else:
        mount = OT3Mount.RIGHT
    # if args.fg:
    #     fg = Mark10.create(port=args.fg_port)
    #     fg.connect()

    if args.dial_indicator:
        gauge = dial_indicator_setup(port=args.dial_port)
    asyncio.run(_main())
