from pybricks.parameters import Button, Stop
from pybricks.tools import StopWatch, wait

import robot_config as config


class ProgramAborted(Exception):
    pass


class MotionTimeout(Exception):
    pass


class Robot:
    def __init__(
        self,
        hub,
        drive_base,
        left_motor,
        right_motor,
        left_drive_motor,
        right_drive_motor,
    ):
        self.hub = hub
        self.drive_base = drive_base
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_drive_motor = left_drive_motor
        self.right_drive_motor = right_drive_motor

        self.attachment_motors = (left_motor, right_motor)
        self.drive_motors = (left_drive_motor, right_drive_motor)

        self.requested_stop = False
        self.previous_buttons = set()
        self.telemetry = None

    def set_telemetry(self, telemetry):
        self.telemetry = telemetry

    def begin_program(self, name="run"):
        self.requested_stop = False
        self.previous_buttons = set(self.hub.buttons.pressed())
        if self.telemetry:
            self.telemetry.begin(name)

    def end_program(self, outcome="success"):
        if self.telemetry:
            self.telemetry.finish_and_send(outcome)
        self.requested_stop = False
        self.previous_buttons = set(self.hub.buttons.pressed())

    def _telemetry_tick(self):
        if self.telemetry:
            self.telemetry.tick()

    def _telemetry_event(self, name, phase, value1=0, value2=0):
        if self.telemetry:
            self.telemetry.event(name, phase, value1, value2)

    def should_stop(self):
        return self.requested_stop

    def check_abort(self):
        buttons = set(self.hub.buttons.pressed())
        new_buttons = buttons - self.previous_buttons
        self.previous_buttons = buttons

        if Button.CENTER in new_buttons:
            self.requested_stop = True

        if self.requested_stop:
            raise ProgramAborted()

    def wait(self, milliseconds, step=config.MOTION_POLL_MS):
        self._telemetry_event("wait", 0, milliseconds)
        elapsed = 0

        while elapsed < milliseconds:
            self.check_abort()
            self._telemetry_tick()
            part = min(step, milliseconds - elapsed)
            wait(part)
            elapsed += part
        self._telemetry_event("wait", 1, milliseconds)

    def stop_drive(self):
        try:
            self.drive_base.stop()
        except Exception as error:
            print("STOP_ERROR drive_base", str(error))

        for motor in self.drive_motors:
            try:
                motor.stop()
            except Exception as error:
                print("STOP_ERROR drive_motor", str(error))

    def stop_attachment(self, motor):
        try:
            motor.stop()
        except Exception as error:
            print("STOP_ERROR attachment", str(error))

    def stop_attachments(self):
        for motor in self.attachment_motors:
            self.stop_attachment(motor)

    def emergency_stop(self):
        self.stop_drive()
        self.stop_attachments()

    def set_drivebase_settings(
        self,
        straight_speed=None,
        straight_acceleration=None,
        turn_rate=None,
        turn_acceleration=None,
    ):
        current = self.drive_base.settings()

        self.drive_base.settings(
            current[0] if straight_speed is None else straight_speed,
            current[1] if straight_acceleration is None else straight_acceleration,
            current[2] if turn_rate is None else turn_rate,
            current[3] if turn_acceleration is None else turn_acceleration,
        )
        self._telemetry_event("settings", 1)

    def reset_drivebase_settings(self):
        self.drive_base.settings(*config.DEFAULT_DRIVEBASE_SETTINGS)

    def set_gyro_use(self, value):
        self.drive_base.use_gyro(value)
        self._telemetry_event("gyro", 1, 1 if value else 0)

    def reset_heading(self, angle=0):
        """Setzt nur die Fahrtrichtung neu, nicht die gefahrene Strecke."""
        self.check_abort()
        distance = self.drive_base.distance()
        self.drive_base.reset(distance=distance, angle=angle)
        self._telemetry_event("reset_heading", 1, angle)

    def straight_task(self, distance, then=Stop.HOLD, timeout_ms=None):
        """Beschreibt eine Geradeausfahrt für ``multitask``."""
        return ("straight", distance, then, timeout_ms)

    def motor_angle_task(
        self,
        motor,
        speed,
        angle,
        then=Stop.HOLD,
        timeout_ms=None,
    ):
        """Beschreibt eine Anbaubewegung für ``multitask``."""
        return ("motor_angle", motor, speed, angle, then, timeout_ms)

    def multitask(self, *tasks):
        """Führt Geradeausfahrt und Anbaumotor-Bewegungen gleichzeitig aus."""
        if not tasks:
            raise ValueError("multitask needs at least one task")

        active = []
        drive_task_seen = False
        attachment_motors = []

        self.check_abort()

        try:
            for task in tasks:
                if not isinstance(task, tuple) or not task:
                    raise ValueError("invalid multitask task")

                task_type = task[0]
                timer = StopWatch()

                if task_type == "straight":
                    if len(task) != 4:
                        raise ValueError("invalid straight task")
                    if drive_task_seen:
                        raise ValueError("multitask accepts only one drive task")

                    distance, then, timeout_ms = task[1:]
                    timeout_ms = (
                        config.DRIVE_TIMEOUT_MS
                        if timeout_ms is None
                        else timeout_ms
                    )
                    self._telemetry_event("straight", 0, distance)
                    self.stop_drive()
                    self.drive_base.straight(distance, then=then, wait=False)
                    active.append((
                        self.drive_base.done,
                        timer,
                        timeout_ms,
                        "straight",
                        distance,
                        self.stop_drive,
                    ))
                    drive_task_seen = True
                elif task_type == "motor_angle":
                    if len(task) != 6:
                        raise ValueError("invalid motor_angle task")

                    motor, speed, angle, then, timeout_ms = task[1:]
                    if motor in attachment_motors:
                        raise ValueError("a motor can only have one task")

                    timeout_ms = (
                        config.MOTOR_TIMEOUT_MS
                        if timeout_ms is None
                        else timeout_ms
                    )
                    self._telemetry_event("motor_angle", 0, speed, angle)
                    motor.run_angle(speed, angle, then=then, wait=False)
                    active.append((
                        motor.done,
                        timer,
                        timeout_ms,
                        "motor_angle",
                        (speed, angle),
                        lambda motor=motor: self.stop_attachment(motor),
                    ))
                    attachment_motors.append(motor)
                else:
                    raise ValueError("unsupported multitask task: " + str(task_type))

            while active:
                self.check_abort()
                self._telemetry_tick()

                for action in active[:]:
                    done, timer, timeout_ms, name, values, stop = action

                    if done():
                        if name == "straight":
                            self._telemetry_event(name, 1, values)
                        else:
                            self._telemetry_event(name, 1, values[0], values[1])
                        active.remove(action)
                    elif timer.time() >= timeout_ms:
                        raise MotionTimeout(name + " timed out")

                if active:
                    wait(config.MOTION_POLL_MS)
        except Exception:
            for action in active:
                action[5]()
            raise

        return True

    def _wait_until_done(self, done, timeout_ms, action_name):
        timer = StopWatch()

        while not done():
            self.check_abort()
            self._telemetry_tick()

            if timer.time() >= timeout_ms:
                raise MotionTimeout(action_name + " timed out")

            wait(config.MOTION_POLL_MS)

    def straight(self, distance, then=Stop.HOLD, timeout_ms=None):
        self.check_abort()
        self._telemetry_event("straight", 0, distance)
        self.stop_drive()
        self.drive_base.straight(distance, then=then, wait=False)

        try:
            self._wait_until_done(
                self.drive_base.done,
                config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms,
                "straight",
            )
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("straight", 2, distance)
            raise

        self._telemetry_event("straight", 1, distance)
        return True

    def turn(self, angle, then=Stop.HOLD, timeout_ms=None):
        self.check_abort()
        self._telemetry_event("turn", 0, angle)
        self.stop_drive()
        self.drive_base.turn(angle, then=then, wait=False)

        try:
            self._wait_until_done(
                self.drive_base.done,
                config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms,
                "turn",
            )
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("turn", 2, angle)
            raise

        self._telemetry_event("turn", 1, angle)
        return True

    def arc(
        self,
        radius,
        angle=None,
        distance=None,
        then=Stop.HOLD,
        timeout_ms=None,
    ):
        if angle is None and distance is None:
            raise ValueError("arc needs angle or distance")
        if angle is not None and distance is not None:
            raise ValueError("arc accepts angle or distance, not both")

        self.check_abort()
        self._telemetry_event("arc", 0, radius, angle if angle is not None else distance)
        self.stop_drive()

        if angle is not None:
            self.drive_base.arc(radius, angle=angle, then=then, wait=False)
        else:
            self.drive_base.arc(radius, distance=distance, then=then, wait=False)

        try:
            self._wait_until_done(
                self.drive_base.done,
                config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms,
                "arc",
            )
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("arc", 2, radius)
            raise

        self._telemetry_event("arc", 1, radius)
        return True

    def drive(self, speed, turn_rate=0):
        self.check_abort()
        self._telemetry_event("drive", 0, speed, turn_rate)
        self.stop_drive()
        self.drive_base.drive(speed, turn_rate)

        try:
            while True:
                self.check_abort()
                self._telemetry_tick()
                wait(config.MOTION_POLL_MS)
        finally:
            self.stop_drive()
            self._telemetry_event("drive", 2, speed, turn_rate)

    def motor_angle(self, motor, speed, angle, then=Stop.HOLD, timeout_ms=None):
        self.check_abort()
        self._telemetry_event("motor_angle", 0, speed, angle)
        motor.run_angle(speed, angle, then=then, wait=False)
        result = self._wait_for_motor(motor, "motor_angle", timeout_ms)
        self._telemetry_event("motor_angle", 1, speed, angle)
        return result

    def motor_target(self, motor, speed, target, then=Stop.HOLD, timeout_ms=None):
        self.check_abort()
        self._telemetry_event("motor_target", 0, speed, target)
        motor.run_target(speed, target, then=then, wait=False)
        result = self._wait_for_motor(motor, "motor_target", timeout_ms)
        self._telemetry_event("motor_target", 1, speed, target)
        return result

    def motor_time(self, motor, speed, time, then=Stop.HOLD, timeout_ms=None):
        self.check_abort()
        self._telemetry_event("motor_time", 0, speed, time)
        motor.run_time(speed, time, then=then, wait=False)

        if timeout_ms is None:
            timeout_ms = max(config.MOTOR_TIMEOUT_MS, time + 2000)

        result = self._wait_for_motor(motor, "motor_time", timeout_ms)
        self._telemetry_event("motor_time", 1, speed, time)
        return result

    def _wait_for_motor(self, motor, action_name, timeout_ms):
        if timeout_ms is None:
            timeout_ms = config.MOTOR_TIMEOUT_MS

        try:
            self._wait_until_done(motor.done, timeout_ms, action_name)
        except (ProgramAborted, MotionTimeout):
            self.stop_attachment(motor)
            raise

        return True

    def motor_until_stalled(
        self,
        motor,
        speed=300,
        then=Stop.COAST,
        timeout_ms=None,
    ):
        if speed == 0:
            raise ValueError("motor_until_stalled needs a non-zero speed")

        self.check_abort()
        self._telemetry_event("motor_until_stalled", 0, speed)
        self.stop_attachment(motor)

        timer = StopWatch()
        progress_window_started = timer.time()
        progress_window_angle = motor.angle()
        direction = 1 if speed > 0 else -1
        timeout_ms = config.STALL_TIMEOUT_MS if timeout_ms is None else timeout_ms
        expected_progress = abs(speed) * config.STALL_PROGRESS_WINDOW_MS / 1000
        minimum_progress = max(
            config.STALL_MIN_PROGRESS_DEG,
            expected_progress * config.STALL_MIN_PROGRESS_RATIO,
        )
        stall_reason = ""

        motor.run(speed)

        try:
            while True:
                self.check_abort()
                self._telemetry_tick()
                now = timer.time()

                # pybricks stall status prüfen
                if motor.stalled():
                    stall_reason = "pybricks"
                    break

                # stall fallback über encoder fortschritt
                if now - progress_window_started >= config.STALL_PROGRESS_WINDOW_MS:
                    progress = (motor.angle() - progress_window_angle) * direction

                    if progress < minimum_progress:
                        stall_reason = "no_progress"
                        break

                    progress_window_started = now
                    progress_window_angle = motor.angle()

                if now >= timeout_ms:
                    raise MotionTimeout("motor_until_stalled timed out")

                wait(config.MOTION_POLL_MS)
        except (ProgramAborted, MotionTimeout):
            self.stop_attachment(motor)
            raise

        if then == Stop.HOLD:
            motor.hold()
        elif then == Stop.BRAKE:
            motor.brake()
        else:
            motor.stop()

        print("MOTOR_STALLED", stall_reason, motor.angle())
        self._telemetry_event("motor_until_stalled", 1, speed, motor.angle())
        return motor.angle()
