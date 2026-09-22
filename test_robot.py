import sys
import types
import unittest


class FakeStopWatch:
    now = 0

    def __init__(self):
        self.started = self.now

    def time(self):
        return self.now - self.started


def fake_wait(milliseconds):
    FakeStopWatch.now += milliseconds


parameters = types.ModuleType("pybricks.parameters")
parameters.Button = types.SimpleNamespace(CENTER="center")
parameters.Direction = types.SimpleNamespace(COUNTERCLOCKWISE="counterclockwise", CLOCKWISE="clockwise")
parameters.Port = types.SimpleNamespace(A="A", B="B", C="C", D="D", F="F")
parameters.Stop = types.SimpleNamespace(HOLD="hold", BRAKE="brake", COAST="coast")
tools = types.ModuleType("pybricks.tools")
tools.StopWatch = FakeStopWatch
tools.wait = fake_wait
sys.modules["pybricks"] = types.ModuleType("pybricks")
sys.modules["pybricks.parameters"] = parameters
sys.modules["pybricks.tools"] = tools

import robot_config as config
from robot import MotionTimeout, Robot


class FakeButtons:
    def pressed(self):
        return []


class FakeHub:
    buttons = FakeButtons()


class FakeImu:
    def angular_velocity(self):
        return (0, 0, 0)


FakeHub.imu = FakeImu()


class FakeMotion:
    def __init__(self, done_after=2):
        self.done_after = done_after
        self.done_checks = 0
        self.started = None
        self.stopped = False
        self.current_angle = 0

    def done(self):
        self.done_checks += 1
        return self.done_checks >= self.done_after

    def stop(self):
        self.stopped = True

    def brake(self):
        self.stopped = True

    def angle(self):
        return self.current_angle


class FakeDriveBase(FakeMotion):
    def __init__(self, done_after=2):
        super().__init__(done_after)
        self.current_distance = 123
        self.reset_values = None
        self.drive_settings = None
        self.heading_control = FakeControl()

    def settings(self, *values):
        if values:
            self.drive_settings = values
        return self.drive_settings

    def straight(self, distance, then, wait):
        self.started = (distance, then, wait)

    def turn(self, angle, then, wait, absolute=False):
        self.started = (angle, then, wait, absolute)
        if absolute:
            self.current_angle = angle
        else:
            self.current_angle += angle

    def drive(self, speed, turn_rate):
        self.current_angle += turn_rate * 0.01

    def distance(self):
        return self.current_distance

    def reset(self, distance, angle):
        self.reset_values = (distance, angle)


class StaleDoneDriveBase(FakeDriveBase):
    def __init__(self):
        super().__init__()
        self.done_values = [True, False, False, True, True]

    def done(self):
        self.done_checks += 1
        return self.done_values.pop(0)


class FakeMotor(FakeMotion):
    def run_angle(self, speed, angle, then, wait):
        self.started = (speed, angle, then, wait)


class FakeControl:
    def __init__(self):
        self.tolerances = (10, 8)

    def target_tolerances(self, *values):
        if values:
            self.tolerances = values
        return self.tolerances


def make_robot(drive_base=None, attachment=None):
    drive_base = drive_base or FakeDriveBase()
    attachment = attachment or FakeMotor()
    robot = Robot(
        FakeHub(),
        drive_base,
        attachment,
        FakeMotor(),
        FakeMotor(),
        FakeMotor(),
    )
    return robot, drive_base, attachment


class RobotTest(unittest.TestCase):
    def setUp(self):
        FakeStopWatch.now = 0

    def test_reset_heading_preserves_distance(self):
        robot, drive_base, _ = make_robot()

        robot.reset_heading(42)

        self.assertEqual(drive_base.reset_values, (123, 42))

    def test_multitask_runs_straight_and_motor_together(self):
        robot, drive_base, motor = make_robot()

        result = robot.multitask(
            robot.straight_task(500),
            robot.motor_angle_task(motor, 300, -90),
        )

        self.assertTrue(result)
        self.assertEqual(drive_base.started, (500, "hold", False))
        self.assertEqual(motor.started, (300, -90, "hold", False))

    def test_straight_ignores_stale_done_from_previous_command(self):
        drive_base = StaleDoneDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.straight(600)

        self.assertEqual(drive_base.done_checks, 4)
        self.assertEqual(drive_base.reset_values, (123, 0))

    def test_straight_brakes_at_measured_distance_without_done(self):
        class CrossingDriveBase(FakeDriveBase):
            def __init__(self):
                super().__init__(done_after=1000)
                self.command_target = self.current_distance
                self.drive_settings = (450, 700, 100, 300)

            def straight(self, distance, then, wait):
                self.started = (distance, then, wait)
                self.command_target = self.current_distance + distance

            def done(self):
                self.done_checks += 1
                self.current_distance = min(
                    self.command_target,
                    self.current_distance + 5,
                )
                return False

        drive_base = CrossingDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.straight(100)

        self.assertLessEqual(abs(drive_base.distance() - 223), 15)
        self.assertTrue(drive_base.stopped)

    def test_turn_can_use_absolute_heading(self):
        robot, drive_base, _ = make_robot()

        robot.turn(85, precise=False, absolute=True)

        self.assertEqual(drive_base.started, (85, "hold", False, True))
        self.assertEqual(drive_base.angle(), 85)

    def test_turn_compensates_an_inaccurate_completed_motion(self):
        class InaccurateDriveBase(FakeDriveBase):
            def turn(self, angle, then, wait, absolute=False):
                self.started = (angle, then, wait, absolute)
                self.current_angle = angle - 5

        drive_base = InaccurateDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.turn(85, absolute=True)

        self.assertEqual(drive_base.started, (91, "brake", False, True))
        self.assertLess(abs(85 - drive_base.angle()), 2)
        self.assertLess(FakeStopWatch.now, 1500)

    def test_turn_brakes_during_single_motion_at_measured_target(self):
        class CrossingDriveBase(FakeDriveBase):
            def __init__(self):
                super().__init__(done_after=1000)
                self.turn_calls = []
                self.command_target = 0

            def turn(self, angle, then, wait, absolute=False):
                self.turn_calls.append(angle)
                self.command_target = angle

            def done(self):
                self.done_checks += 1
                self.current_angle = min(self.command_target, self.current_angle + 0.5)
                return False

        drive_base = CrossingDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.turn(85)

        self.assertEqual(drive_base.turn_calls, [91])
        self.assertLessEqual(abs(drive_base.angle() - 85), 1)
        self.assertTrue(drive_base.stopped)

    def test_negative_turn_uses_same_early_brake(self):
        class CrossingDriveBase(FakeDriveBase):
            def __init__(self):
                super().__init__(done_after=1000)
                self.command_target = 0

            def turn(self, angle, then, wait, absolute=False):
                self.command_target = angle

            def done(self):
                self.done_checks += 1
                self.current_angle = max(self.command_target, self.current_angle - 0.5)
                return False

        drive_base = CrossingDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.turn(-85)

        self.assertLessEqual(abs(drive_base.angle() + 85), 1)
        self.assertTrue(drive_base.stopped)

    def test_large_turn_finishes_at_measured_target_before_done(self):
        class SlowLargeTurnDriveBase(FakeDriveBase):
            def done(self):
                self.done_checks += 1
                return FakeStopWatch.now >= 2000

        drive_base = SlowLargeTurnDriveBase()
        drive_base.drive_settings = (450, 700, 100, 300)
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.turn(180, precise=False)

        self.assertEqual(FakeStopWatch.now, 0)
        self.assertLess(FakeStopWatch.now, 2000)

    def test_precise_180_turn_finishes_at_stable_gyro_target(self):
        class StuckDoneAtTargetDriveBase(FakeDriveBase):
            def done(self):
                self.done_checks += 1
                return False

            def turn(self, angle, then, wait, absolute=False):
                self.started = (angle, then, wait, absolute)
                self.current_angle = 180

        drive_base = StuckDoneAtTargetDriveBase()
        robot, _, _ = make_robot(drive_base=drive_base)

        robot.turn(180)

        self.assertEqual(drive_base.started, (186, "brake", False, False))
        self.assertEqual(drive_base.angle(), 180)
        self.assertTrue(drive_base.stopped)
        self.assertLess(FakeStopWatch.now, 1000)

    def test_reset_settings_tightens_heading_position_tolerance(self):
        robot, drive_base, _ = make_robot()

        robot.reset_drivebase_settings()

        self.assertEqual(drive_base.heading_control.tolerances, (10, 1))

    def test_multitask_stops_everything_on_timeout(self):
        drive_base = FakeDriveBase(done_after=1000)
        motor = FakeMotor(done_after=1000)
        robot, _, _ = make_robot(drive_base, motor)

        with self.assertRaises(MotionTimeout):
            robot.multitask(
                robot.straight_task(500, timeout_ms=20),
                robot.motor_angle_task(motor, 300, -90, timeout_ms=20),
            )

        self.assertTrue(drive_base.stopped)
        self.assertTrue(motor.stopped)


if __name__ == "__main__":
    unittest.main()
