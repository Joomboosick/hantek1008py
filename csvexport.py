#!/usr/bin/env python3
from hantek1008 import Hantek1008, CorrectionDataType, ZeroOffsetShiftCompensationFunctionType
from typing import Union, Optional, List, Dict, Any, IO
import logging as log
import argparse
import time
import datetime
import os
import lzma
import sys
import math
from usb.core import USBError
from time import sleep
from utils.csvwriter import ThreadedCsvWriter

assert sys.version_info >= (3, 6)


def main(csv_file_path: str,
         selected_channels: Optional[List[int]]=None,
         vertical_scale_factor: Optional[List[float]]=[1.0],
         calibrate_output_file_path: Optional[str]=None,
         calibrate_channels_at_once: Optional[int]=None,
         calibration_file_path: Optional[str]=None,
         zero_offset_shift_compensation_channel: Optional[int]=None,
         zero_offset_shift_compensation_function_file_path: Optional[str]=None,
         zero_offset_shift_compensation_function_time_offset_sec: int=0,
         raw_or_volt: str="volt",
         sampling_rate: float=440,
         timestamp_style: str="own_row",
         do_sampling_rate_measure: bool=True) -> None:

    if selected_channels is None or len(selected_channels) == 0:
        selected_channels = list(range(1, 9))

    assert len(set(selected_channels)) == len(selected_channels)
    assert all(1 <= c <= 8 for c in selected_channels)
    selected_channels = [i-1 for i in selected_channels]

    assert zero_offset_shift_compensation_channel is None or zero_offset_shift_compensation_function_file_path is None

    assert zero_offset_shift_compensation_channel is None or 1 <= zero_offset_shift_compensation_channel <= 8
    if zero_offset_shift_compensation_channel is not None:
        zero_offset_shift_compensation_channel -= 1

    assert zero_offset_shift_compensation_function_time_offset_sec >= 0

    assert vertical_scale_factor is None or isinstance(vertical_scale_factor, List)
    if vertical_scale_factor is None:
        vertical_scale_factor = [1.0] * 8
    elif len(vertical_scale_factor) == 1:
        vertical_scale_factor = [1.0 if i not in selected_channels
                                 else vertical_scale_factor[0]
                                 for i in range(8)]
    else:
        assert len(vertical_scale_factor) == len(selected_channels)
        # the vscale value of a channel is the value in vertical_scale_factor
        # on the same index as the channel in selected channel
        # or 1.0 if the channel is not in selected_channels
        vertical_scale_factor = [1.0 if i not in selected_channels
                                 else vertical_scale_factor[selected_channels.index(i)]
                                 for i in range(8)]

    timestamp_style = timestamp_style.lower()
    assert timestamp_style in ["first_column", "own_row"]

    correction_data: CorrectionDataType = [{} for _ in range(8)]  # list of dicts of dicts
    # use case: correction_data[channel_id][vscale][units] = correction_factor

    zero_offset_shift_compensation_function = None
    if zero_offset_shift_compensation_function_file_path:
        globals_dict: Dict[str, Any] = {}
        with check_and_open_file(zero_offset_shift_compensation_function_file_path) as f:
            exec(f.read(), globals_dict)
        zero_offset_shift_compensation_function = globals_dict["calc_zos"]
        assert callable(zero_offset_shift_compensation_function)

    if calibration_file_path:
        with check_and_open_file(calibration_file_path) as f:
            import json
            calibration_data = json.load(f)

        log.info(f"Using calibration data from file '{calibration_file_path}' to correct measured values")

        for channel_id, channel_cdata in sorted(calibration_data.items()):
            channel_id = int(channel_id)
            if len(channel_cdata) == 0:
                continue
            log.info(f"Channel {channel_id+1}:")
            for test in channel_cdata:
                vscale = test["vscale"]
                test_voltage = test["test_voltage"]
                units = test["measured_value"] - test["zero_offset"]
                correction_factor = test_voltage / (units * 0.01 * vscale)

                if test_voltage == 0:
                    continue
                assert 0.5 < correction_factor < 2.0, "Correction factor seems to be false"

                #log.info(f"    {test} -> {correction_factor}")
                log.info(f"{test_voltage:>6}V -> {correction_factor:0.5f}")

                if vscale not in correction_data[channel_id]:
                    correction_data[channel_id][vscale] = {}

                correction_data[channel_id][vscale][units] = correction_factor

        # log.info("\n".join(str(x) for x in correction_data))
        channels_without_cd = [i + 1 for i, x in enumerate(correction_data) if len(x) == 0]
        if len(channels_without_cd) > 0:
            log.warning(f"There is no calibration data for channel(s): {channels_without_cd}")

    device = connect(vertical_scale_factor, selected_channels, correction_data, zero_offset_shift_compensation_channel,
                     zero_offset_shift_compensation_function, zero_offset_shift_compensation_function_time_offset_sec)

    if calibrate_output_file_path:
        calibration_routine(device, calibrate_output_file_path, calibrate_channels_at_once)
        device.close()
        sys.exit()

    measured_sampling_rate = None
    if do_sampling_rate_measure:
        measurment_duration = 10
        log.info(f"Measure sample rate of device (takes about {measurment_duration} sec) ...")
        measured_sampling_rate = measure_sampling_rate(device, sampling_rate, measurment_duration)
        log.info(f"-> {measured_sampling_rate:.4f} Hz")
        # TODO Remove
        # sys.exit()

    csv_file_path_zero = csv_file_path

    # data collection is in loop because in case of an error it restarts the collection
    for i in range(1, 100):
        try:
            sample(device, raw_or_volt, selected_channels, sampling_rate, vertical_scale_factor,
                   csv_file_path, timestamp_style, measured_sampling_rate)
            # no error? -> finished by user interaction
            break
        except USBError as usb_error:
            # usb error bug occurred? try to close the device or reset it, sleep a sec and restart
            log.error(str(usb_error))
            try:
                device.close()
            except:
                try:
                    sleep(0.5)
                    device._dev.reset()
                except:
                    pass
            sleep(1.0)
            device = connect(vertical_scale_factor, selected_channels, correction_data,
                             zero_offset_shift_compensation_channel,
                             zero_offset_shift_compensation_function,
                             zero_offset_shift_compensation_function_time_offset_sec)
            if csv_file_path_zero != '-':
                csv_file_path = f"{csv_file_path_zero}.{i:02d}"

    log.info("Exporting data finished")
    device.close()


def connect(vertical_scale_factor: Union[float, List[float]],
            selected_channels: List[int],
            correction_data: Optional[CorrectionDataType] = None,
            zero_offset_shift_compensation_channel: Optional[int] = None,
            zero_offset_shift_compensation_function: Optional[ZeroOffsetShiftCompensationFunctionType] = None,
            zero_offset_shift_compensation_function_time_offset_sec: Optional[int] = 0):
    device = Hantek1008(vertical_scale_factor=vertical_scale_factor,
                        active_channels=selected_channels,
                        correction_data=correction_data,
                        zero_offset_shift_compensation_channel=zero_offset_shift_compensation_channel,
                        zero_offset_shift_compensation_function=zero_offset_shift_compensation_function,
                        zero_offset_shift_compensation_function_time_offset_sec
                        =zero_offset_shift_compensation_function_time_offset_sec)

    try:
        log.info("Connecting...")
        try:
            device.connect()
        except RuntimeError as e:
            log.error(str(e))
            sys.exit(1)
        log.info("Connection established")

        log.info("Initialising...")
        try:
            device.init()
        except RuntimeError as e:
            log.error(str(e))
            sys.exit(1)
        log.info("Initialisation completed")
    except KeyboardInterrupt:
        device.close()
        sys.exit(0)

    return device


def sample(device: Hantek1008, raw_or_volt: str, selected_channels: List[int], sampling_rate:float,
           vertical_scale_factor: List[float], csv_file_path: str, timestamp_style: str,
           measured_sampling_rate: float = None):
    log.info(f"Processing data of channel{'' if len(selected_channels) == 1 else 's'}:"
             f" {' '.join([str(i+1) for i in selected_channels])}")

    computed_actual_sampling_rate = Hantek1008.actual_sampling_rate_factor(len(selected_channels))
    log.warning(f"When not using all 8 channels, the actual sampling rate ({computed_actual_sampling_rate:.2f}) is "
                f"higher than the given sampling_rate ({sampling_rate})! "
                f"Best is to use the --measuresamplingrate flag.")

    if raw_or_volt == "volt+raw":  # add the coresponding raw values to the selected channel list
        selected_channels += [sc + Hantek1008.channel_count() for sc in selected_channels]

    try:
        # csv_file:  IO[str] = None
        # output_csv_filename = "channel_data.csv"
        if csv_file_path == '-':
            log.info("Exporting data to stdout...")
            csv_file: IO[str] = sys.stdout
        elif csv_file_path.endswith(".xz"):
            log.info(f"Exporting data lzma-compressed to file '{csv_file_path}'...")
            csv_file = lzma.open(csv_file_path, 'at', newline='')
        else:
            log.info(f"Exporting data to file '{csv_file_path}'...")
            csv_file = open(csv_file_path, 'at', newline='')

        csv_writer = ThreadedCsvWriter(csv_file, delimiter=',')

        csv_writer.write_comment("HEADER")

        now = datetime.datetime.now()
        # timestamps are by nature UTC
        csv_writer.write_comment(f"UNIX-Time: {now.timestamp()}")
        csv_writer.write_comment(f"UNIX-Time: {now.astimezone(datetime.timezone.utc).isoformat()} UTC")

        # channel >= 8 are the raw values of the corresponding channels < 8
        channel_titles = [f'ch_{i+1 if i < 8 else (str(i+1-8)+"_raw")}' for i in selected_channels]
        if timestamp_style == "first_column":
            channel_titles = ["time"] + channel_titles
        csv_writer.write_comment(f"{', '.join(channel_titles)}")

        csv_writer.write_comment(f"intended samplingrate: {sampling_rate} Hz")
        csv_writer.write_comment(f"samplingrate: {computed_actual_sampling_rate} Hz")
        if measured_sampling_rate:
            csv_writer.write_comment(f"measured samplingrate: {measured_sampling_rate} Hz")

        csv_writer.write_comment(f"vscale: {', '.join(str(f) for f in vertical_scale_factor)}")
        csv_writer.write_comment("# zero offset data:")
        for vscale, zero_offset in sorted(device.get_zero_offsets().items()):
            csv_writer.write_comment(f"zero_offset [{vscale:<4}]: {' '.join([str(round(v, 1)) for v in zero_offset])}")

        csv_writer.write_comment(f"zosc-method: {device.get_used_zero_offsets_shift_compensation_method()}")

        csv_writer.write_comment(f"DATA")

        # TODO: make these configurable
        roll_mode = True
        milli_volt_int_representation = False

        if roll_mode:
            last_timestamp = datetime.datetime.now().timestamp()
            for per_channel_data in device.request_samples_roll_mode(mode=raw_or_volt, sampling_rate=sampling_rate):
                now_timestamp = datetime.datetime.now().timestamp()

                def get_data_of_ch(ch: int):
                    index = sorted(selected_channels).index(ch)
                    # per_channel_data contains lists of values per channel, channels sorted ascending
                    return per_channel_data[index]

                # after this, channels are sorted the same way as in selected_channels
                per_channel_data = [get_data_of_ch(ch) for ch in selected_channels]

                if milli_volt_int_representation:
                    per_channel_data = [[f"{round(value*1000)}" for value in single_channel]
                                        for single_channel in per_channel_data]

                if timestamp_style == "first_column":
                    values_per_channel_count = len(per_channel_data[0])
                    deltatime_per_value = (now_timestamp-last_timestamp)/values_per_channel_count
                    timestamps_interpolated = [last_timestamp + i*deltatime_per_value
                                               for i in range(values_per_channel_count)]
                    csv_writer.write_rows(zip(timestamps_interpolated, *per_channel_data))
                else:  # timestamp_style == "own_row":
                    csv_writer.write_rows(zip(*per_channel_data))
                    # timestamps are by nature UTC
                    csv_writer.write_comment(f"UNIX-Time: {now_timestamp}")

                last_timestamp = now_timestamp
        else:  # burst mode
            # TODO currently not supported
            # TODO missing features:
            # * timestamp_style
            # * change arrangement of channels if selected_channels is nor sorted
            while True:
                channel_data2, channel_data3 = device.request_samples_burst_mode()

                print(len(channel_data2[0]), len(channel_data3[0]))
                # channel_data = [cd2 + cd3 for cd2, cd3 in zip(channel_data2, channel_data3)]
                # channel_data = [cd[70:] + cd[:70] for cd in channel_data]

                csv_writer.write_rows(zip(*channel_data2))
                csv_writer.write_rows(zip(*channel_data3))
                # timestamps are by nature UTC
                csv_writer.write_comment(f"UNIX-Time: { datetime.datetime.now().timestamp()}")
    except KeyboardInterrupt:
        log.info("Sample collection was stopped by user")
        pass

    if csv_writer:
        csv_writer.close()


def measure_sampling_rate(device: Hantek1008, used_sampling_rate: float, measurment_duration: float) -> float:
    required_samples = max(4, int(math.ceil(measurment_duration * used_sampling_rate)))
    counter = -1
    start_time: float = 0
    for per_channel_data in device.request_samples_roll_mode(sampling_rate=used_sampling_rate):
        if counter == -1:  # skip first samples to ignore the duration of initialisation
            start_time = time.perf_counter()
            counter = 0
        counter += len(per_channel_data[0])
        if counter >= required_samples:
            break

    duration = time.perf_counter() - start_time
    return counter/duration


def calibration_routine(device: Hantek1008, calibrate_file_path: str, channels_at_once: int):
    assert channels_at_once in [1, 2, 4, 8]

    print("This interactive routine will generate a calibration that can later be used "
          "to get more precise results. It works by connecting different well known "
          "voltages one after another to a channel. Once all calibration voltages are "
          "measured, the same is done for every other channel.")

    import json
    required_calibration_samples_nun = 512
    calibration_data: Dict[int, List[Dict[str, Any]]] = {}  # dictionary of lists
    device.pause()

    test_voltages = None
    while test_voltages is None:
        try:
            in_str = input("Calibration voltages (x, y, z, ...): ")
            test_voltages = [float(v) for v in in_str.split(',')]
            if len(test_voltages) < 1:
                print("Input must contain at least one voltage")
        except ValueError:
            print("Input must be comma separated floats")

    print(f"Calibration voltages are: {' '.join([ f'{v}V' for v in test_voltages])}")

    for channel_id in range(8):
        calibration_data[channel_id] = []

    for channel_id in range(0, 8, channels_at_once):

        for test_voltage in test_voltages:
            cmd = input(f"Do {test_voltage}V measurement on channel {channel_id+1}"
                        f"{(' to ' + str(channel_id+channels_at_once)) if channels_at_once>1 else ''} (Enter),"
                        f" skip voltage (s), skip channel (ss) or quit (q): ")
            if cmd == 'q':
                return
            elif cmd == 'ss':
                break
            elif cmd == 's':
                continue

            device.cancel_pause()

            print(f"Measure {required_calibration_samples_nun} values for {test_voltage}V...")
            data = []
            for _, row in zip(
                    range(required_calibration_samples_nun),
                    device.request_samples_roll_mode_single_row(mode="raw")):
                data.append(row)
                pass

            device.pause()

            channel_data = list(zip(*data))

            for calibrated_channel_id in range(channel_id, channel_id+channels_at_once):
                cd = channel_data[calibrated_channel_id]
                avg = sum(cd) / len(cd)

                calibration_data[calibrated_channel_id].append({
                    "test_voltage": test_voltage,
                    "measured_value": round(avg, 2),
                    "vscale": device.get_vscales()[calibrated_channel_id],
                    "zero_offset": round(device.get_zero_offset(channel_id=calibrated_channel_id), 2)
                })

    with open(calibrate_file_path, 'w') as calibration_file:
        calibration_file.write(json.dumps(calibration_data))


def check_and_open_file(file_path: str):
    if not os.path.exists(file_path):
        log.error(f"There is no file '{file_path}'.")
        sys.exit(1)
    if os.path.isdir(file_path):
        log.error(f"'{file_path}' is a directory.")
        sys.exit(1)
    return open(file_path)


if __name__ == "__main__":

    description = f"""\
Collect data from device 'Hantek 1008'. Usage examples:
    * Save data sampled with 22 Hz in file 'my_data.csv':
        {sys.argv[0]} my_data.csv --channels 1 2 --samplingrate 22
    * Create and fill calibration file 'my_cal.json':
        {sys.argv[0]} --calibrate my_cal.cd.json 1
"""

    def channel_type(value):
        ivalue = int(value)
        if 1 <= ivalue <= 8:
            return ivalue
        raise argparse.ArgumentTypeError(f"There is no channel {value}")

    str_to_log_level = {log.getLevelName(ll).lower(): ll for ll in [log.DEBUG, log.INFO, log.WARN]}

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=description)
    command_group = parser.add_mutually_exclusive_group(required=True)
    command_group.add_argument(metavar='csv_path', dest='csv_path', nargs='?',
                               type=str, default=None,
                               help='Exports measured data to the given file in CSV format.'
                                    " If filename ends with '.xz' the content is compress using lzma/xz."
                                    " This reduces the filesize to ~ 1/12 compered to the uncompressed format."
                                    " Those files can be decompressed using 'xz -dk <filename>")
    command_group.add_argument('--calibrate', metavar=('calibrationfile_path', 'channels_at_once'), nargs=2,
                               type=str, default=None,
                               help='If set, calibrate the device by measuring given voltages and write'
                                    ' calibration values to given file.'
                                    ' Multiple channels (1, 2, 4 or all 8) can get calibrated at the same time'
                                    ' if supplied with the same voltage. Ignores all other arguments.')
    parser.add_argument('-s', '--channels', metavar='channel', nargs='+',
                        type=channel_type, default=list(range(1, 9)),
                        help="Select channels that are of interest")
    parser.add_argument('-l', '--loglevel', dest='log_level', nargs='?',
                        type=str, default="info", choices=str_to_log_level.keys(),
                        help='Set the loglevel to debug')
    parser.add_argument('-v', '--vscale', metavar='scale', nargs="+",
                        type=float, default=[1.0], choices=Hantek1008.valid_vscale_factors(),
                        help='Set the pre scale in the hardware, must be 1, 0.125, or 0.02. If one value is given, all '
                             'selected channels will use that vscale, otherwise there must be one value per selected'
                             'channel')
    parser.add_argument('-c', '--calibrationfile', dest="calibration_file_path", metavar='calibrationfile_path',
                        type=str, default=None,
                        help="Use the content of the given calibration file to correct the measured samples")
    parser.add_argument('-r', '--raw', dest="raw_or_volt",
                        type=str, default="volt", const="raw", nargs='?', choices=["raw", "volt", "volt+raw"],
                        help="Specifies whether the sample values return from the device should be transformed"
                             " to volts (eventually using calibration data) or not. If flag is not set, it defaults"
                             " to 'volt'. If flag is set without a parameter 'raw' is used")
    parser.add_argument('-z', '--zoscompensation', dest="zos_compensation", metavar='x',
                        type=str, default=None, nargs='*',
                        help=
                        """Compensate the zero offset shift that obscures over longer timescales.
                        There are two possible ways of compensating that:
                        (A) Compute shift out of an unused channels: Needs at least one unused channel, make sure
                         that no voltage is applied to the given channel. 
                        (B) Compute shift with the help of a given function. Such a function computes a correction-factor
                         based on the time past since start.
                        Defaults to no compensation. If used without an argument method A is used on channel 8.
                        If one integer argument is given method A is used on that channel. Otherwise Method B is used.
                        It awaits a path to a file with a python function 
                        (calc_zos(ch: int, vscale: float, dtime: float)->float) in it 
                        and as a second argument a time offset (how long the device is already running in sec).
                        """)
    parser.add_argument('-f', '--samplingrate', dest='sampling_rate',
                        type=float, default=440, choices=Hantek1008.valid_roll_sampling_rates(),
                        help='Set the sampling rate (in Hz) the device should use (default:440). If not all channels '
                             'are used the actual sampling rate is higher. The factors are: '
                             f'{[Hantek1008.actual_sampling_rate_factor(ch) for ch in  range(1, 9)]}. '
                             'E.g. if only two channels are used the actual sampling rate is 3.03 higher '
                             'than the given value. A free channel that is used for the zos-compensation will reduce '
                             'the actual sampling the same way as if the channel is normally used.')
    parser.add_argument('-m', '--measuresamplingrate', dest='do_sampling_rate_measure', action="store_const",
                        default=False, const=True,
                        help='Measure the exact samplingrate the device achieves by using the computer internal clock.'
                             'Increases startup duration by ~10 sec')
    parser.add_argument('-t', '--timestampstyle', dest="timestamp_style",
                        type=str, default="own_row", nargs='?', choices=["own_row", "first_column"],
                        help="Specifies the style the timestamps of the values are included in the CSV output. There"
                             " are two options: When the 'own_row' style is used, every time the device sends a bunch"
                             " of measured samples, these are writen to the CSV output followed by one row with the"
                             " timestamp."
                             " Use the 'first_column' option to let the first column have interpolated timestamps."
                             " Default is 'own_row'.")

    args = parser.parse_args()

    args.log_level = str_to_log_level[args.log_level]

    def arg_assert(ok, fail_message):
        if not ok:
            parser.error(fail_message)


    if args.calibrate is not None:
        calibrate_channels_at_once = args.calibrate[1]
        arg_assert(calibrate_channels_at_once.isdigit() and int(calibrate_channels_at_once) in [1, 2, 4, 8],
                   "The second argument must be 1, 2, 4 or 8.")

    arg_assert(len(args.vscale) == 1 or len(args.vscale) == len(args.channels),
               "There must be one vscale factor or as many as selected channels")
    arg_assert(len(set(args.channels)) == len(args.channels),
               "Selected channels list is not a set (multiple occurrences of the same channel id")
    # arg_assert(args.calibration_file_path is None or not args.raw_or_volt.contains("volt"),
    #            "--calibrationfile can not be used together with the '--raw volt' flag")
    # arg_assert(args.zos_compensation is None or not args.raw_or_volt.contains("volt"),
    #            "--zoscompensation can not be used together with the '--raw volt' flag")

    if args.zos_compensation is not None:
        arg_assert(len(args.zos_compensation) <= 2, "'--zoscompensation' only awaits 0, 1 or 2 parameters")
        if len(args.zos_compensation) == 0:
            # defaults to channel 8
            args.zos_compensation = [8]
        if len(args.zos_compensation) == 1:  # if compensation via unused channel is used
            args.zos_compensation[0] = channel_type(args.zos_compensation[0])
            arg_assert(len(args.channels) < 8,
                       "Zero-offset-shift-compensation is only possible if there is at least one unused channel")
            arg_assert(args.zos_compensation[0] not in args.channels,
                       f"The channel {args.zos_compensation[0]} is used for Zero-offset-shift-compensation,"
                       f" but it is also a selected channel")
        if len(args.zos_compensation) == 2:  # if compensation via function is used
            arg_assert(args.zos_compensation[1].isdigit(), "The second argument must be an int")
            args.zos_compensation[1] = int(args.zos_compensation[1])

    log.basicConfig(level=args.log_level, format='%(levelname)-7s: %(message)s')

    main(selected_channels=args.channels,
         vertical_scale_factor=args.vscale,
         csv_file_path=args.csv_path,
         calibrate_output_file_path=args.calibrate[0] if args.calibrate else None,
         calibrate_channels_at_once=int(args.calibrate[1]) if args.calibrate else None,
         calibration_file_path=args.calibration_file_path,
         raw_or_volt=args.raw_or_volt,
         zero_offset_shift_compensation_channel=
         args.zos_compensation[0]
         if args.zos_compensation is not None and len(args.zos_compensation) == 1
         else None,
         zero_offset_shift_compensation_function_file_path=
         args.zos_compensation[0]
         if args.zos_compensation is not None and len(args.zos_compensation) == 2
         else None,
         zero_offset_shift_compensation_function_time_offset_sec=
         args.zos_compensation[1]
         if args.zos_compensation is not None and len(args.zos_compensation) == 2
         else 0,
         sampling_rate=args.sampling_rate,
         timestamp_style=args.timestamp_style,
         do_sampling_rate_measure=args.do_sampling_rate_measure)
