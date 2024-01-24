import argparse
import asyncio
import time

import os
import sys
import termios, tty
import json

from hardware_testing.opentrons_api import types
from hardware_testing.opentrons_api import helpers_ot3
from hardware_testing import data

from hardware_testing.opentrons_api.types import Mount, Axis, Point, GantryLoad, OT3Mount

from opentrons.hardware_control.types import CriticalPoint

def convert(seconds):
    weeks, seconds = divmod(seconds, 7*24*60*60)
    days, seconds = divmod(seconds, 24*60*60)
    hours, seconds = divmod(seconds, 60*60)
    minutes, seconds = divmod(seconds, 60)

    return "%02d:%02d:%02d:%02d:%02d" % (weeks, days, hours, minutes, seconds)

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

async def jog(api, position, cp):
    step_size = [0.01, 0.05, 0.1, 0.5, 1, 10, 20, 50]
    step_length_index = 3
    step = step_size[step_length_index]
    xy_speed = 60
    za_speed = 65
    mount = Mount.LEFT
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
            round(position[Axis.X], 2),
            ",",
            round(position[Axis.Y], 2),
            ",",
            round(position[Axis.by_mount(mount)], 2),
            " Motor Step: ",
            step_size[step_length_index],
            end="",
        )
        print("\r", end="")

async def calibrate_tip_racks(api, mount, slot_loc, AXIS):
    print("Calibrate tip rack positions\n")
    calibrated_slot_loc = {}

    for key in slot_loc.keys():
        print(f"TIP RACK IN SLOT {key}\n")
        await api.move_to(mount, Point(slot_loc[key][0], slot_loc[key][1], 250.0))
        await api.move_to(mount, Point(slot_loc[key][0], slot_loc[key][1], slot_loc[key][2]))
        # tip_rack_position = await helpers_ot3.jog_mount_ot3(api, mount)
        cur_pos = await api.current_position_ot3(mount, critical_point=CriticalPoint.NOZZLE)
        tip_rack_position = await jog(api, cur_pos, CriticalPoint.NOZZLE)
        calibrated_slot_loc[key] = (tip_rack_position[Axis.X], tip_rack_position[Axis.Y], tip_rack_position[AXIS])
        await api.home([AXIS])

    json_object = json.dumps(calibrated_slot_loc, indent=0)
    # ("/home/root/calibrated_slot_locations.json", "w")
    with open("/data/testing_data/calibrated_slot_locations.json", "w") as outfile:
        outfile.write(json_object)
    return calibrated_slot_loc


async def _main(is_simulating: bool) -> None:
    path = '/data/testing_data/calibrated_slot_locations.json'
    mount = OT3Mount.LEFT
    api = await helpers_ot3.build_async_ot3_hardware_api(is_simulating=is_simulating)
    await api.home()
    await api.home_plunger(mount)

    test_tag = ""
    test_robot = "Tip Pick Up/Plunger Lifetime"
    if args.test_tag:
        test_tag = input("Enter test tag:\n\t>> ")
    if args.test_robot:
        test_robot = input("Enter robot ID:\n\t>> ")

    AXIS = Axis.Z_L

    # TIP_RACKS = args.tip_rack_num # default: 12
    PICKUPS_PER_TIP = args.pick_up_num # default: 20
    # COLUMNS = 12
    # ROWS = 8
    CYCLES = 1

    test_pip = api.get_attached_instrument(mount)

    print("mount.id:{}".format(test_pip["pipette_id"]))

    test_name = "tip-pick-up-lifetime-test"
    if args.restart_flag:
        if(os.path.exists(path)):
            with open(path, 'r') as openfile:
                complete_dict = json.load(openfile)
                file_name = complete_dict['csv_name']
        else:
            print("Slot locations calibration file not found.\n")
            calibrated_slot_loc = await calibrate_tip_racks(api, mount, slot_loc, AXIS)
    else:
        file_name = data.create_file_name(test_name=test_name, run_id=data.create_run_id(), tag=test_tag,pipid=test_pip["pipette_id"])
        header = ['Time (W:H:M:S)', 'Test Robot', 'Test Pipette', 'Tip Rack', 'Tip Number', 'Total Tip Pick Ups', 'Total Failures']
        header_str = data.convert_list_to_csv_line(header)
        data.append_data_to_file(test_name=test_name, file_name=file_name, data=header_str)

    print("test_pip",test_pip)
    if(len(test_pip) == 0):
        print(f"No pipette recognized on {mount.name} mount\n")
        sys.exit()

    print(f"\nTest pipette: {test_pip['name']}\n")

    load = api.gantry_load

    if load == GantryLoad.HIGH_THROUGHPUT:
        ### add 96 ch conditions here ###
        multi_pip = True ###
        COLUMNS = 1
        ROWS = 1
    else:
        print("Test requires a high-throughput load (96ch pipette)\n")
        print("Exiting script...\n")
        sys.exit()
        # if "single" in test_pip['name']:
        #     multi_pip = False
        #     ### remove ### check_tip_presence = True
        # else:
        #     multi_pip = True
        #     ROWS = 1
        #     CYCLES = 8
        #     ### remove ### check_tip_presence = True

    slot_loc = {
        "A1": (13.42, 394.92, 150),
        "A2": (177.32, 394.92, 150),
        "A3": (341.03, 394.92, 150),
        "B1": (13.42, 288.42, 150),
        "B2": (177.32, 288.92, 150),
        "B3": (341.03, 288.92, 150),
        "C1": (13.42, 181.92, 150),
        "C2": (177.32, 181.92, 150),
        "C3": (341.03, 181.92, 150),
        "D1": (13.42, 75.5, 150),
        "D2": (177.32, 75.5, 150),
        "D3": (341.03, 75.5, 150),
    }

    ### optional arg for tip rack calibration
    if(not args.load_cal):
        calibrated_slot_loc = await calibrate_tip_racks(api, mount, slot_loc, AXIS)
    else:
        ### import calibrated json file
        # path = '/home/root/.opentrons/testing_data/calibrated_slot_locations.json'
        print("Loading calibration data...\n")
        path = '/data/testing_data/calibrated_slot_locations.json'
        if(os.path.exists(path)):
            with open('/data/testing_data/calibrated_slot_locations.json', 'r') as openfile:
                calibrated_slot_loc = json.load(openfile)
        else:
            print("Slot locations calibration file not found.\n")
            calibrated_slot_loc = await calibrate_tip_racks(api, mount, slot_loc, AXIS)
        print("Calibration data successfully loaded!\n")

    # add cfg start slot
    start_slot = int(str(args.start_slot_row_col_totalTips_totalFailure).split(':')[0])
    start_row = int(str(args.start_slot_row_col_totalTips_totalFailure).split(':')[1])
    start_col = int(str(args.start_slot_row_col_totalTips_totalFailure).split(':')[2])
    total_tip_num = int(str(args.start_slot_row_col_totalTips_totalFailure).split(':')[3])
    total_fail_num = int(str(args.start_slot_row_col_totalTips_totalFailure).split(':')[4])

    start_time = time.perf_counter()
    elapsed_time = 0
    rack = start_slot - 1
    total_pick_ups = total_tip_num - 1
    total_failures = total_fail_num
    start_tip_nums = 1

    # load complete information
    if args.restart_flag:
        if(os.path.exists(path)):
            with open('/data/testing_data/calibrated_slot_locations.json', 'r') as openfile:
                print("load complete information...\n")
                load_complete_dict = json.load(openfile)
                CYCLES = CYCLES - (load_complete_dict["cycle"] -1)
                rack = (load_complete_dict["slot_num"] - 1)
                total_pick_ups = load_complete_dict["total_tip_pick_up"]
                total_failures = load_complete_dict["total_failure"]
                start_slot = rack + 1
                start_row = load_complete_dict["row"]
                start_col = load_complete_dict["col"]
                start_tip_nums = load_complete_dict["tip_num"] + 1
    else:
        print("Failed to load complete information.\n")

    for i in range(start_slot-1):
        del calibrated_slot_loc[list(calibrated_slot_loc)[0]]

    for i in range(CYCLES):
        print(f"\n=========== Cycle {i + 1}/{CYCLES} ===========\n")
        if i > 0:
            stop_time = time.perf_counter()
            print("Replace tips before continuing test.")
            input("\n\t>> Press \"Enter\" to continue.")
            resume_time = time.perf_counter()
            elapsed_time += resume_time - stop_time
            print(f"Elapsed time: {convert(resume_time-stop_time)}\n")
        for key_index, key in enumerate(calibrated_slot_loc.keys()):
            if (key_index >= 12):
                break
            rack += 1
            await api.home([AXIS])
            await api.move_to(mount, Point(calibrated_slot_loc[key][0],
                        calibrated_slot_loc[key][1], 250.0))
            await api.move_to(mount, Point(calibrated_slot_loc[key][0],
                        calibrated_slot_loc[key][1], calibrated_slot_loc[key][2]+5))
            for col in range(COLUMNS):
                if col < start_col - 1:
                    continue
                await api.move_to(mount, Point(calibrated_slot_loc[key][0]+9*col,
                            calibrated_slot_loc[key][1], calibrated_slot_loc[key][2]+5))
                for row in range(ROWS):
                    if col == start_col -1 and row < start_row -1:
                        continue
                    print("=================================\n")
                    print(f"Tip rack in slot {key}, Column: {col+1}, Row: {row+1}\n")
                    if 'p1000' in test_pip['name']:
                        if "1" in key:
                            tip_len = 95.6
                        elif "2" in key:
                            tip_len = 58.35
                        elif "3" in key:
                            tip_len = 57.9
                    else:
                        tip_len = 57.9
                    print(f"Tip length: {tip_len} mm\n")
                    if row > 0:
                        await api.move_rel(mount, delta=Point(y=-9))
                    await api.move_to(mount, Point(calibrated_slot_loc[key][0]+9*col,
                                calibrated_slot_loc[key][1]-9*row, calibrated_slot_loc[key][2]+5))
                    await api.move_to(mount, Point(calibrated_slot_loc[key][0]+9*col,
                                calibrated_slot_loc[key][1]-9*row, calibrated_slot_loc[key][2]))
                    start_pos = await api.gantry_position(mount)
                    for pick_up in range(PICKUPS_PER_TIP):
                        await api.move_to(mount, start_pos)
                        if col == start_col -1 and row == start_row -1 and pick_up < start_tip_nums -1:
                            continue
                        print("= = = = = = = = = = = = = = = = =\n")
                        print(f"Tip Pick Up #{pick_up+1}\n")
                        print("Picking up tip...\n")
                        await api.pick_up_tip(mount, tip_len)
                        total_pick_ups += 1

                        ### Tip presence implemented in sw, removed for 96ch testing ###
                        ### check tip presence after tip pick up
                        # if check_tip_presence:
                        #     tip_presence_pick_up = await api.get_tip_status(mount)
                        #     pick_up_keys = list(tip_presence_pick_up.keys())
                        #     if(tip_presence_pick_up[pick_up_keys[0]]):
                        #         print("\t>> Tip detected!\n")
                        #         tip_presence_pick_up_flag = True
                        #     else:
                        #         total_failures += 1
                        #         tip_presence_pick_up_flag = False
                        #         print(f"\t>> Tip not detected! Total failures: {total_failures}\n")
                        # else:
                        #     tip_presence_pick_up_flag = 'Tip Presence Not Checked'

                        ### move plunger from blowout to top, back to blow_out
                        # top_pos, bottom_pos, _, _ = helpers_ot3.get_plunger_positions_ot3(api, mount)

                        # print("Move to bottom plunger position\n")
                        # await helpers_ot3.move_plunger_absolute_ot3(api, mount, bottom_pos)
                        # print("Move to top plunger position\n")
                        # await helpers_ot3.move_plunger_absolute_ot3(api, mount, top_pos)
                        # print("Move to bottom plunger position\n")
                        # await helpers_ot3.move_plunger_absolute_ot3(api, mount, bottom_pos)

                        ### check tip presence after tip drop
                        print("Dropping tip...\n")
                        await api.drop_tip(mount)
                        ### Tip presence implemented in sw, removed for 96ch testing ###
                        # if check_tip_presence:
                        #     tip_presence_eject = await api.get_tip_status(mount)
                        #     drop_tip_keys = list(tip_presence_eject.keys())
                        #     if(tip_presence_eject[drop_tip_keys[0]]):
                        #         print("\t>> Tip detected after ejecting tip!\n")
                        #         print("\t>> Canceling script...\n")
                        #         total_failures += 1
                        #         tip_presence_eject_flag = True
                        #     else:
                        #         print("\t>> Tip not detected!\n")
                        #         tip_presence_eject_flag = False
                        # else:
                        #     tip_presence_eject_flag = 'Tip Presence Not Checked'

                        ### save test data and continue loop/exit based on tip eject success

                        cycle_data = [convert(time.perf_counter()-elapsed_time-start_time), test_robot, test_pip["pipette_id"], rack, pick_up+1,
                            total_pick_ups, total_failures]
                        cycle_data_str = data.convert_list_to_csv_line(cycle_data)
                        data.append_data_to_file(test_name=test_name, file_name=file_name, data=cycle_data_str)

                        ### save the last complate information

                        if(os.path.exists(path)):
                            with open('/data/testing_data/calibrated_slot_locations.json', 'r') as openfile:
                                print("Recording...\n")
                                calibrated_slot_loc = json.load(openfile)
                                complete_dict = {"cycle": i+1, "slot_num": rack, "tip_num":pick_up+1, "total_tip_pick_up": total_pick_ups,
                                                 "total_failure": total_failures, "col":col + 1, "row":row + 1, "csv_name":file_name}
                                calibrated_slot_loc.update(complete_dict)
                                with open('/data/testing_data/calibrated_slot_locations.json', 'w') as writefile:
                                    json.dump(calibrated_slot_loc, writefile)

                        else:
                            print("Slot locations calibration file not found.\n")
                            print("Failed to record complete information.\n")

                        ### Tip presence implemented in sw, removed for 96ch testing ###
                        # if tip_presence_eject_flag == True:
                        #     await api.home()
                        #     sys.exit()

                    ### adjust row increment
                    # print("Moving to next row...\n")
                    # await api.move_rel(mount, delta=Point(z=5))

                ### adjust column increment
                await api.move_to(mount, Point(calibrated_slot_loc[key][0]+9*col,
                            calibrated_slot_loc[key][1]-9*row, calibrated_slot_loc[key][2]+5))
                print("Moving to next column...\n")

            # release start
            start_col = 1
            start_row = 1
            start_tip_nums = 1

        print("=================================\n")
        print(f"\nCYCLE {i+1} COMPLETE\n")
        await api.home()
        await api.home_plunger(mount)

    print("=================================\n")
    print("1/4 LIFETIME TEST COMPLETE\n")
    await api.home()

if __name__ == "__main__":
    # mount_options = {
    #     "left": types.Mount.LEFT,
    #     "right": types.Mount.RIGHT,
    #     "gripper": types.Mount.GRIPPER,
    # }
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true")
    # parser.add_argument(
    #     "--mount", type=str, choices=list(mount_options.keys()), default="left"
    # )
    parser.add_argument("--pick_up_num", type=int, default=20)
    # parser.add_argument("--tip_rack_num", type=int, default=12)
    parser.add_argument("--load_cal", action="store_true")
    parser.add_argument("--test_tag", action="store_true")
    parser.add_argument("--test_robot", action="store_true")
    parser.add_argument("--restart_flag", action="store_true")
    parser.add_argument("--start_slot_row_col_totalTips_totalFailure", type=str, default="1:1:1:1:0")
    # parser.add_argument("--check_tip", action="store_true")
    args = parser.parse_args()
    # mount = mount_options[args.mount]

    asyncio.run(_main(args.simulate))
