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

from robot import MotionTimeout, Robot


class FakeButtons:
    def pressed(self):
        return []


class FakeHub:
    buttons = FakeButtons()


class FakeMotion:
    def __init__(self, done_after=2):
        self.done_after = done_after
        self.done_checks = 0
        self.started = None
        self.stopped = False

    def done(self):
        self.done_checks += 1
        return self.done_checks >= self.done_after

    def stop(self):
        self.stopped = True


class FakeControl:
    def __init__(self):
        self.tolerances = (12, 8)

    def target_tolerances(self, *values):
        if values:
            self.tolerances = values
        return self.tolerances


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

    def distance(self):
        return self.current_distance

    def reset(self, distance, angle):
        self.reset_values = (distance, angle)


class FakeMotor(FakeMotion):
    def run_angle(self, speed, angle, then, wait):
        self.started = (speed, angle, then, wait)


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

    def test_reset_settings_tightens_only_heading_position_tolerance(self):
        robot, drive_base, _ = make_robot()

        robot.reset_drivebase_settings()

        self.assertEqual(drive_base.heading_control.tolerances, (12, 1))

    def test_multitask_runs_straight_and_motor_together(self):
        robot, drive_base, motor = make_robot()

        result = robot.multitask(
            robot.straight_task(500),
            robot.motor_angle_task(motor, 300, -90),
        )

        self.assertTrue(result)
        self.assertEqual(drive_base.started, (500, "hold", False))
        self.assertEqual(motor.started, (300, -90, "hold", False))

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
