"""Repeatable DriveBase turn calibration.

Run this through the normal program menu.  Results are printed as one
comma-separated line per test so they can be copied into a spreadsheet or
parsed from the Pybricks console.
"""

from pybricks.parameters import Stop
from pybricks.tools import StopWatch


# Edit these three lists to focus on the angles and motion profiles used in a
# competition run.  The default matrix covers short/medium turns, both
# directions, and slow through fast turn rates.
CALIBRATION_ANGLES = (85, 135, -85, -135)
CALIBRATION_TURN_RATES = (80, 120, 180, 240)
CALIBRATION_TURN_ACCELERATIONS = (150, 300, 600)

CALIBRATION_SETTLE_MS = 250
CALIBRATION_PAUSE_MS = 150


def _yaw_rate(hub):
    try:
        # PrimeHub is mounted flat in this robot, so the third component is
        # the yaw rate.  Keep this helper defensive for older firmware.
        return hub.imu.angular_velocity()[2]
    except Exception:
        return 0


def _run_turn(robot, target):
    """Prefer absolute turns, with a compatibility fallback for old firmware."""
    try:
        robot.turn(target, then=Stop.HOLD, precise=False, absolute=True)
        return "absolute"
    except TypeError:
        robot.turn(target, then=Stop.HOLD, precise=False)
        return "relative_fallback"


def run(robot):
    robot.set_gyro_use(True)
    robot.reset_drivebase_settings()

    print("CAL_BEGIN")
    print("CAL_FIELDS,mode,target_deg,turn_rate,turn_accel,raw_delta_deg,raw_error_deg,settled_delta_deg,settled_error_deg,duration_ms,final_yaw_rate")

    for turn_acceleration in CALIBRATION_TURN_ACCELERATIONS:
        for turn_rate in CALIBRATION_TURN_RATES:
            robot.set_drivebase_settings(
                turn_rate=turn_rate,
                turn_acceleration=turn_acceleration,
            )

            for target in CALIBRATION_ANGLES:
                robot.check_abort()
                robot.reset_heading(0)
                robot.wait(CALIBRATION_PAUSE_MS)

                start_angle = robot.drive_base.angle()
                watch = StopWatch()
                mode = _run_turn(robot, target)
                raw_angle = robot.drive_base.angle()
                raw_delta = raw_angle - start_angle
                raw_error = target - raw_delta
                duration = watch.time()

                # This delay is only for measurement.  Production turns do
                # not pay this cost; it shows whether HOLD continues to move
                # the robot after Pybricks first reports done().
                robot.wait(CALIBRATION_SETTLE_MS)
                settled_angle = robot.drive_base.angle()
                settled_delta = settled_angle - start_angle
                settled_error = target - settled_delta
                final_yaw_rate = _yaw_rate(robot.hub)

                print(
                    "CAL_RESULT,{},{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{},{}".format(
                        mode,
                        target,
                        turn_rate,
                        turn_acceleration,
                        raw_delta,
                        raw_error,
                        settled_delta,
                        settled_error,
                        duration,
                        final_yaw_rate,
                    )
                )

    robot.stop_drive()
    robot.reset_drivebase_settings()
    print("CAL_END")
