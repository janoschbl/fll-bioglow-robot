from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase

import robot_config as config
from robot import MotionTimeout, ProgramAborted, Robot
from telemetry import Telemetry


def main():
    hub = PrimeHub()
    left_motor = Motor(config.LEFT_ATTACHMENT_PORT)
    right_motor = Motor(config.RIGHT_ATTACHMENT_PORT)
    left_drive_motor = Motor(
        config.LEFT_DRIVE_PORT,
        config.LEFT_DRIVE_DIRECTION,
    )
    right_drive_motor = Motor(
        config.RIGHT_DRIVE_PORT,
        config.RIGHT_DRIVE_DIRECTION,
    )

    drive_base = DriveBase(
        left_drive_motor,
        right_drive_motor,
        wheel_diameter=config.WHEEL_DIAMETER_MM,
        axle_track=config.AXLE_TRACK_MM,
    )
    drive_base.use_gyro(True)

    robot = Robot(
        hub,
        drive_base,
        left_motor,
        right_motor,
        left_drive_motor,
        right_drive_motor,
    )
    robot.reset_drivebase_settings()
    telemetry = Telemetry(
        hub,
        drive_base,
        (left_motor, right_motor, left_drive_motor, right_drive_motor),
        None,
    )
    robot.set_telemetry(telemetry)
    ergebnis = "success"
    robot.begin_program("Live-Debugfahrt")
    try:
        print("DEBUG_START")
        robot.wait(500)
        robot.straight(150)
        robot.wait(500)
        robot.turn(45)
        robot.wait(300)
        robot.turn(-45)
        robot.wait(500)
        print("DEBUG_SUCCESS")
    except ProgramAborted:
        ergebnis = "aborted"
        print("DEBUG_ABORTED")
    except MotionTimeout as error:
        ergebnis = "timeout"
        print("DEBUG_TIMEOUT", str(error))
    except Exception as error:
        ergebnis = "error"
        print("DEBUG_ERROR", str(error))
    finally:
        robot.emergency_stop()
        robot.end_program(ergebnis)


if __name__ == "__main__":
    main()
