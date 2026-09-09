from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.pupdevices import ForceSensor, Motor
from pybricks.robotics import DriveBase

import programs
import robot_config as config
from menu import Menu
from robot import Robot
from telemetry import Telemetry


def create_force_sensor():
    try:
        return ForceSensor(config.FORCE_SENSOR_PORT)
    except OSError as error:
        print("FORCE_SENSOR_UNAVAILABLE", str(error))
        return None


def main():
    hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)

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

    force_sensor = create_force_sensor()
    telemetry = Telemetry(
        hub,
        drive_base,
        (left_motor, right_motor, left_drive_motor, right_drive_motor),
        force_sensor,
    )
    robot.set_telemetry(telemetry)
    menu = Menu(hub, robot, force_sensor)
    programs.load(menu.program, robot)

    menu.register_motor_debug(left_motor, direction=1, index=1, name="Left Motor")
    menu.register_motor_debug(right_motor, direction=1, index=2, name="Right Motor")
    menu.register_motor_debug(left_drive_motor, direction=1, index=3, name="Left Drive")
    menu.register_motor_debug(right_drive_motor, direction=1, index=4, name="Right Drive")

    menu.run()


if __name__ == "__main__":
    main()
