"""Vergleicht Ein-Pass-Drehungen direkt auf dem Hub und der Matte."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

import robot_config as config
from robot import Robot


def main():
    config.SMOOTH_TURN_DEBUG = True
    hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)
    left = Motor(config.LEFT_DRIVE_PORT, config.LEFT_DRIVE_DIRECTION)
    right = Motor(config.RIGHT_DRIVE_PORT, config.RIGHT_DRIVE_DIRECTION)
    base = DriveBase(left, right, config.WHEEL_DIAMETER_MM, config.AXLE_TRACK_MM)
    base.use_gyro(True)
    robot = Robot(hub, base, None, None, left, right)
    robot.reset_drivebase_settings()

    while not hub.imu.ready():
        wait(100)
    base.reset(distance=0, angle=0)
    robot.begin_program("smooth_turn_bench")
    try:
        for attempt in range(1, 4):
            for heading in (180, 0):
                print("BENCH", attempt, "target", heading)
                robot.turn_to(heading)
                print("BENCH_RESULT", attempt, heading, base.angle())
                robot.wait(300)
    finally:
        base.brake()


if __name__ == "__main__":
    main()
