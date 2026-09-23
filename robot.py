from pybricks.parameters import Button, Stop
from pybricks.tools import StopWatch, wait

import robot_config as config


class ProgramAborted(Exception):
    """Signalisiert einen durch den Benutzer abgebrochenen Programmlauf."""

    pass


class MotionTimeout(Exception):
    """Signalisiert eine Bewegung, die ihr Zeitlimit überschritten hat."""

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
        """Initialisiert die Robotersteuerung mit Hub, DriveBase und Motoren.

        :param hub: Pybricks-Hub mit Tasten- und Sensorschnittstellen.
        :param drive_base: Konfigurierte Pybricks-DriveBase.
        :param left_motor: Linker Anbaumotor.
        :param right_motor: Rechter Anbaumotor.
        :param left_drive_motor: Linker Fahrmotor.
        :param right_drive_motor: Rechter Fahrmotor.
        """
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
        """Setzt den Empfänger für Lauf- und Bewegungsdaten.

        :param telemetry: Telemetrieobjekt oder ``None`` zum Deaktivieren.
        """
        self.telemetry = telemetry

    def begin_program(self, name="run"):
        """Beginnt einen Programmlauf und setzt den Abbruchzustand zurück.

        :param name: Name des Programmlaufs für die Telemetrie.
        """
        self.requested_stop = False
        self.previous_buttons = set(self.hub.buttons.pressed())
        if self.telemetry:
            self.telemetry.begin(name)

    def end_program(self, outcome="success"):
        """Beendet den Programmlauf und sendet dessen Telemetrie.

        :param outcome: Ergebnisbezeichnung des Programmlaufs.
        """
        if self.telemetry:
            self.telemetry.finish_and_send(outcome)
        self.requested_stop = False
        self.previous_buttons = set(self.hub.buttons.pressed())

    def _telemetry_tick(self):
        """Erfasst einen Telemetrie-Messpunkt, sofern Telemetrie aktiv ist."""
        if self.telemetry:
            self.telemetry.tick()

    def _telemetry_event(self, name, phase, value1=0, value2=0):
        """Erfasst ein Telemetrie-Ereignis.

        :param name: Name des Ereignisses.
        :param phase: Numerische Ereignisphase.
        :param value1: Erster Messwert.
        :param value2: Zweiter Messwert.
        """
        if self.telemetry:
            self.telemetry.event(name, phase, value1, value2)

    def should_stop(self):
        """Gibt zurück, ob ein Programmabbruch angefordert wurde.

        :return: ``True`` bei angefordertem Abbruch, sonst ``False``.
        """
        return self.requested_stop

    def check_abort(self):
        """Prüft die Mitteltaste und bricht den Programmlauf bei Bedarf ab.

        :raises ProgramAborted: Wenn ein Abbruch angefordert wurde.
        """
        buttons = set(self.hub.buttons.pressed())
        new_buttons = buttons - self.previous_buttons
        self.previous_buttons = buttons

        if Button.CENTER in new_buttons:
            self.requested_stop = True

        if self.requested_stop:
            raise ProgramAborted()

    def wait(self, milliseconds, step=config.MOTION_POLL_MS):
        """Wartet abbrechbar und aktualisiert währenddessen die Telemetrie.

        :param milliseconds: Gesamte Wartezeit in Millisekunden.
        :param step: Prüfintervall in Millisekunden.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        """
        self._telemetry_event("wait", 0, milliseconds)
        elapsed = 0

        while elapsed < milliseconds:
            self.check_abort()
            self._telemetry_tick()
            part = min(step, milliseconds - elapsed)
            wait(part)
            elapsed += part
        self._telemetry_event("wait", 1, milliseconds)

    def stop_drive(self, include_motors=False):
        """Stoppt die DriveBase und optional zusätzlich ihre Fahrmotoren.

        :param include_motors: Stoppt bei ``True`` auch beide Fahrmotoren direkt.
        """
        try:
            self.drive_base.stop()
        except Exception as error:
            print("STOP_ERROR drive_base", str(error))

        if include_motors:
            for motor in self.drive_motors:
                try:
                    motor.stop()
                except Exception as error:
                    print("STOP_ERROR drive_motor", str(error))

    def brake_drive(self):
        """Beendet den DriveBase-Regler und bremst beide Fahrmotoren sofort."""
        self.stop_drive()
        for motor in self.drive_motors:
            try:
                motor.brake()
            except Exception as error:
                print("BRAKE_ERROR drive_motor", str(error))

    def stop_attachment(self, motor):
        """Stoppt einen Anbaumotor.

        :param motor: Zu stoppender Pybricks-Motor.
        """
        try:
            motor.stop()
        except Exception as error:
            print("STOP_ERROR attachment", str(error))

    def stop_attachments(self):
        """Stoppt beide konfigurierten Anbaumotoren."""
        for motor in self.attachment_motors:
            self.stop_attachment(motor)

    def emergency_stop(self):
        """Stoppt DriveBase, Fahrmotoren und Anbaumotoren sofort."""
        self.stop_drive(include_motors=True)
        self.stop_attachments()

    def set_drivebase_settings(
        self,
        straight_speed=None,
        straight_acceleration=None,
        turn_rate=None,
        turn_acceleration=None,
    ):
        """Ändert ausgewählte DriveBase-Einstellungen.

        :param straight_speed: Geradeausgeschwindigkeit in Millimetern pro Sekunde.
        :param straight_acceleration: Geradeausbeschleunigung in Millimetern pro Quadratsekunde.
        :param turn_rate: Drehrate in Grad pro Sekunde.
        :param turn_acceleration: Drehbeschleunigung in Grad pro Quadratsekunde.
        """
        current = self.drive_base.settings()

        self.drive_base.settings(
            current[0] if straight_speed is None else straight_speed,
            current[1] if straight_acceleration is None else straight_acceleration,
            current[2] if turn_rate is None else turn_rate,
            current[3] if turn_acceleration is None else turn_acceleration,
        )
        self._telemetry_event("settings", 1)

    def reset_drivebase_settings(self):
        """Setzt die DriveBase auf die Standardwerte aus ``robot_config`` zurück."""
        self.drive_base.settings(*config.DEFAULT_DRIVEBASE_SETTINGS)
        self._configure_heading_tolerance()

    def _configure_heading_tolerance(self):
        """Konfiguriert die zulässige Winkelabweichung der DriveBase."""
        try:
            speed_tolerance, _ = self.drive_base.heading_control.target_tolerances()
            self.drive_base.heading_control.target_tolerances(
                speed_tolerance,
                config.TURN_POSITION_TOLERANCE_DEG,
            )
        except (AttributeError, OSError, TypeError) as error:
            print("TURN_TOLERANCE_UNAVAILABLE", str(error))

    def set_gyro_use(self, value):
        """Aktiviert oder deaktiviert die Gyro-Regelung der DriveBase.

        :param value: ``True`` aktiviert die Gyro-Regelung.
        """
        self.drive_base.use_gyro(value)
        if value:
            self._configure_heading_tolerance()
        self._telemetry_event("gyro", 1, 1 if value else 0)

    def reset_heading(self, angle=0):
        """Setzt die Fahrtrichtung neu, ohne den Streckenzähler zu verändern.

        :param angle: Neuer absoluter Richtungswert in Grad.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        """
        self.check_abort()
        distance = self.drive_base.distance()
        self.drive_base.reset(distance=distance, angle=angle)
        self._telemetry_event("reset_heading", 1, angle)

    def _reset_drive_control(self):
        """Verwirft alte Fahrziele, ohne Weg oder Richtung zu veraendern."""
        distance = self.drive_base.distance()
        angle = self.drive_base.angle()
        self.drive_base.reset(distance=distance, angle=angle)

    def straight_task(self, distance, then=Stop.HOLD, timeout_ms=None):
        """Erstellt eine Geradeausaufgabe für ``multitask``.

        :param distance: Relative Strecke in Millimetern.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: Aufgabenbeschreibung für ``multitask``.
        """
        return ("straight", distance, then, timeout_ms)

    def motor_angle_task(
        self,
        motor,
        speed,
        angle,
        then=Stop.HOLD,
        timeout_ms=None,
    ):
        """Erstellt eine Motorwinkelaufgabe für ``multitask``.

        :param motor: Zu bewegender Anbaumotor.
        :param speed: Motorgeschwindigkeit in Grad pro Sekunde.
        :param angle: Relativer Motorwinkel in Grad.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: Aufgabenbeschreibung für ``multitask``.
        """
        return ("motor_angle", motor, speed, angle, then, timeout_ms)

    def multitask(self, *tasks):
        """Führt mehrere Fahr- und Motoraufgaben gleichzeitig aus.

        :param tasks: Mit ``straight_task`` oder ``motor_angle_task`` erstellte Aufgaben.
        :return: ``True`` nach erfolgreichem Abschluss aller Aufgaben.
        :raises ValueError: Wenn eine Aufgabenbeschreibung ungültig ist.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn eine Aufgabe ihr Zeitlimit überschreitet.
        """
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
                    self._reset_drive_control()
                    self.drive_base.straight(distance, then=then, wait=False)
                    start_distance = self.drive_base.distance()
                    active.append({
                        "done": self.drive_base.done,
                        "timer": timer,
                        "timeout_ms": timeout_ms,
                        "name": "straight",
                        "values": distance,
                        "stop": self.stop_drive,
                        "progress": lambda start=start_distance: abs(
                            self.drive_base.distance() - start
                        ),
                        "min_progress": config.MOTION_MIN_PROGRESS_MM,
                        "armed": False,
                        "saw_not_done": False,
                    })
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
                    start_angle = motor.angle()
                    active.append({
                        "done": motor.done,
                        "timer": timer,
                        "timeout_ms": timeout_ms,
                        "name": "motor_angle",
                        "values": (speed, angle),
                        "stop": lambda motor=motor: self.stop_attachment(motor),
                        "progress": lambda motor=motor, start=start_angle: abs(
                            motor.angle() - start
                        ),
                        "min_progress": config.MOTION_MIN_PROGRESS_DEG,
                        "armed": False,
                        "saw_not_done": False,
                    })
                    attachment_motors.append(motor)
                else:
                    raise ValueError("unsupported multitask task: " + str(task_type))

            while active:
                self.check_abort()
                self._telemetry_tick()

                for action in active[:]:
                    done_state = action["done"]()
                    if not done_state:
                        action["saw_not_done"] = True

                    if not action["armed"]:
                        if (
                            action["progress"]() >= action["min_progress"]
                            or (
                                action["saw_not_done"]
                                and action["timer"].time()
                                >= config.MOTION_START_GUARD_MS
                            )
                        ):
                            action["armed"] = True

                    if action["armed"] and done_state:
                        name = action["name"]
                        values = action["values"]
                        if name == "straight":
                            self._telemetry_event(name, 1, values)
                        else:
                            self._telemetry_event(name, 1, values[0], values[1])
                        active.remove(action)
                    elif action["timer"].time() >= action["timeout_ms"]:
                        raise MotionTimeout(action["name"] + " timed out")

                if active:
                    wait(config.MOTION_POLL_MS)
        except Exception:
            for action in active:
                action["stop"]()
            raise

        return True

    def _wait_until_done(
        self,
        done,
        timeout_ms,
        action_name,
        progress=None,
        min_progress=0,
        completion=None,
    ):
        """Wartet überwacht auf den Abschluss einer asynchronen Bewegung.

        :param done: Funktion, die den Abschlusszustand liefert.
        :param timeout_ms: Maximale Dauer in Millisekunden.
        :param action_name: Bewegungsname für Fehlermeldungen.
        :param progress: Optionale Funktion zur Fortschrittsmessung.
        :param min_progress: Mindestfortschritt zum Aktivieren der Abschlussprüfung.
        :param completion: Optionale zusätzliche Abschlussprüfung.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        timer = StopWatch()
        armed = False
        saw_not_done = False

        while True:
            self.check_abort()
            self._telemetry_tick()

            done_state = done()
            if not done_state:
                saw_not_done = True

            if not armed:
                progress_reached = (
                    progress is not None and progress() >= min_progress
                )
                if progress_reached or (
                    timer.time() >= config.MOTION_START_GUARD_MS
                    and (min_progress <= 0 or saw_not_done)
                ):
                    armed = True

            if armed:
                is_done = (
                    completion(done_state)
                    if completion is not None
                    else done_state
                )
                if is_done:
                    return

            if timer.time() >= timeout_ms:
                raise MotionTimeout(action_name + " timed out")

            wait(config.MOTION_POLL_MS)

    def straight(self, distance, then=Stop.HOLD, timeout_ms=None):
        """Fährt eine relative Strecke mit eigener Distanzüberwachung.

        :param distance: Strecke in Millimetern; negative Werte fahren rückwärts.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        self.check_abort()
        self._telemetry_event("straight", 0, distance)
        self._reset_drive_control()
        start_distance = self.drive_base.distance()
        target_distance = start_distance + distance
        direction = 1 if distance >= 0 else -1
        timeout_ms = config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms
        try:
            straight_speed = abs(self.drive_base.settings()[0])
        except (AttributeError, OSError, TypeError):
            straight_speed = config.DEFAULT_STRAIGHT_SPEED
        brake_lead = max(
            config.STRAIGHT_MIN_BRAKE_LEAD_MM,
            straight_speed * config.STRAIGHT_BRAKE_REACTION_MS / 1000,
        )
        self.drive_base.straight(distance, then=then, wait=False)

        timer = StopWatch()
        next_log = 0
        armed = False
        saw_not_done = False

        print(
            "STRAIGHT_START",
            "start", start_distance,
            "target", target_distance,
            "distance", distance,
            "speed", straight_speed,
            "brake_lead", brake_lead,
            "timeout_ms", timeout_ms,
        )

        try:
            while True:
                self.check_abort()
                self._telemetry_tick()

                elapsed = timer.time()
                current_distance = self.drive_base.distance()
                error = target_distance - current_distance
                remaining = error * direction
                progress = abs(current_distance - start_distance)
                done_state = self.drive_base.done()

                if not done_state:
                    saw_not_done = True
                if not armed and (
                    progress >= config.MOTION_MIN_PROGRESS_MM
                    or (
                        elapsed >= config.MOTION_START_GUARD_MS
                        and saw_not_done
                    )
                ):
                    armed = True
                    print("STRAIGHT_ARMED", "ms", elapsed, "progress", progress)

                if elapsed >= next_log:
                    print(
                        "STRAIGHT_STATUS",
                        "ms", elapsed,
                        "distance", current_distance,
                        "target", target_distance,
                        "error", error,
                        "done", done_state,
                        "armed", armed,
                    )
                    next_log = elapsed + config.TURN_LOG_INTERVAL_MS

                if armed and remaining <= brake_lead:
                    print(
                        "STRAIGHT_TARGET_REACHED",
                        "ms", elapsed,
                        "distance", current_distance,
                        "error", error,
                        "remaining", remaining,
                        "brake_lead", brake_lead,
                    )
                    self.brake_drive()
                    self.wait(config.STRAIGHT_BRAKE_SETTLE_MS)
                    final_distance = self.drive_base.distance()
                    final_error = target_distance - final_distance
                    print(
                        "STRAIGHT_DONE",
                        "ms", timer.time(),
                        "distance", final_distance,
                        "error", final_error,
                        "source", "measured_distance_brake",
                    )
                    if abs(final_error) > config.STRAIGHT_COMPLETION_TOLERANCE_MM:
                        print("STRAIGHT_ACCURACY_WARNING", target_distance, final_error)
                    break

                if armed and done_state:
                    print(
                        "STRAIGHT_DONE",
                        "ms", elapsed,
                        "distance", current_distance,
                        "error", error,
                        "source", "pybricks",
                    )
                    break

                if elapsed >= timeout_ms:
                    print(
                        "STRAIGHT_TIMEOUT",
                        "ms", elapsed,
                        "distance", current_distance,
                        "target", target_distance,
                        "error", error,
                        "done", done_state,
                    )
                    raise MotionTimeout("straight timed out")

                wait(config.MOTION_POLL_MS)
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("straight", 2, distance)
            raise

        print(
            "STRAIGHT_FINISH",
            "target", target_distance,
            "distance", self.drive_base.distance(),
        )
        self._telemetry_event("straight", 1, distance)
        return True

    def _turn_once(
        self,
        angle,
        then,
        timeout_ms,
        absolute,
        target_angle,
    ):
        """Führt einen überwachten Drehabschnitt bis zum Zielwinkel aus.

        :param angle: An die DriveBase übergebener Drehwinkel in Grad.
        :param then: Motorverhalten nach Abschluss des Drehbefehls.
        :param timeout_ms: Maximale Dauer in Millisekunden.
        :param absolute: Verwendet bei ``True`` einen absoluten Drehbefehl.
        :param target_angle: Zu überwachender absoluter Gyro-Zielwinkel.
        :return: Quelle des erkannten Bewegungsabschlusses.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        self.stop_drive()
        start_angle = self.drive_base.angle()
        direction = 1 if target_angle >= start_angle else -1

        if absolute:
            self.drive_base.turn(angle, then=then, wait=False, absolute=True)
        else:
            self.drive_base.turn(angle, then=then, wait=False)

        timer = StopWatch()
        next_log = 0
        armed = False
        saw_not_done = False

        print(
            "TURN_START",
            "start", start_angle,
            "target", target_angle,
            "command", angle,
            "absolute", absolute,
            "timeout_ms", timeout_ms,
        )

        while True:
            self.check_abort()
            self._telemetry_tick()

            elapsed = timer.time()
            current_angle = self.drive_base.angle()
            error = target_angle - current_angle
            remaining = error * direction
            done_state = self.drive_base.done()
            progress = abs(current_angle - start_angle)

            if not done_state:
                saw_not_done = True
            if not armed and (
                progress >= config.MOTION_MIN_PROGRESS_DEG
                or (
                    elapsed >= config.MOTION_START_GUARD_MS
                    and saw_not_done
                )
            ):
                armed = True
                print("TURN_ARMED", "ms", elapsed, "progress", progress)

            if elapsed >= next_log:
                print(
                    "TURN_STATUS",
                    "ms", elapsed,
                    "angle", current_angle,
                    "target", target_angle,
                    "error", error,
                    "done", done_state,
                    "armed", armed,
                )
                next_log = elapsed + config.TURN_LOG_INTERVAL_MS

            in_target = armed and remaining <= config.TURN_BRAKE_LEAD_DEG
            if in_target:
                print(
                    "TURN_TARGET_REACHED",
                    "ms", elapsed,
                    "angle", current_angle,
                    "error", error,
                    "remaining", remaining,
                    "brake_lead", config.TURN_BRAKE_LEAD_DEG,
                )
                self.brake_drive()
                print(
                    "TURN_DONE",
                    "ms", elapsed,
                    "angle", self.drive_base.angle(),
                    "error", target_angle - self.drive_base.angle(),
                    "source", "measured_angle_brake",
                )
                return "measured_angle"

            if armed and done_state:
                print(
                    "TURN_SEGMENT_DONE",
                    "ms", elapsed,
                    "angle", current_angle,
                    "error", error,
                    "source", "pybricks",
                )
                return "pybricks"

            if elapsed >= timeout_ms:
                print(
                    "TURN_TIMEOUT",
                    "ms", elapsed,
                    "angle", current_angle,
                    "target", target_angle,
                    "error", error,
                    "done", done_state,
                    "progress", progress,
                )
                raise MotionTimeout("turn timed out")

            wait(config.MOTION_POLL_MS)

    def _turn_one_pass(self, angle, absolute, timeout_ms):
        """Dreht einmalig mit Gyro-Rückmeldung und angepasstem Geschwindigkeitsprofil."""
        self.check_abort()
        self.drive_base.brake()
        start = self.drive_base.angle()
        target = angle if absolute else start + angle
        delta = target - start
        if abs(delta) <= config.SMOOTH_TURN_ACCEPT_DEG:
            return True

        direction = 1 if delta > 0 else -1
        limit_ms = config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms
        timer = StopWatch()
        last_progress_ms = 0
        last_angle = start
        next_log_ms = 0
        self._telemetry_event("turn", 0, angle)
        print("SMOOTH_TURN_START", "start", start, "target", target,
              "max_rate", config.SMOOTH_TURN_MAX_RATE)

        try:
            while True:
                self.check_abort()
                self._telemetry_tick()
                now = timer.time()
                state = self.drive_base.state()
                heading = state[2]
                rate = max(0, state[3] * direction)
                remaining = (target - heading) * direction

                if abs(heading - last_angle) >= config.SMOOTH_TURN_STALL_PROGRESS_DEG:
                    last_angle = heading
                    last_progress_ms = now

                # Reaktionsweg und Bremsweg aus der gemessenen Drehrate schätzen.
                stop_lead = (config.SMOOTH_TURN_STOP_OFFSET_DEG
                             + rate * config.SMOOTH_TURN_STOP_DELAY_MS / 1000
                             + rate * rate / (2 * config.SMOOTH_TURN_BRAKE_DECEL))
                if remaining <= stop_lead:
                    self.drive_base.brake()
                    self.wait(config.SMOOTH_TURN_SETTLE_MS)
                    final = self.drive_base.angle()
                    error = target - final
                    brake_travel = (final - heading) * direction
                    print("SMOOTH_TURN_DONE", "ms", timer.time(),
                          "target", target, "final", final, "error", error,
                          "stop_rate", rate, "brake_angle", heading,
                          "brake_travel", brake_travel, "brake_lead", stop_lead)
                    if abs(error) > config.SMOOTH_TURN_ACCEPT_DEG:
                        print("SMOOTH_TURN_ACCURACY_WARNING", error)
                    self._telemetry_event("turn", 1, angle)
                    return True

                if now >= limit_ms:
                    raise MotionTimeout("smooth turn timed out")
                if now - last_progress_ms >= config.SMOOTH_TURN_STALL_MS:
                    raise MotionTimeout("smooth turn made no progress")

                approach = remaining - stop_lead
                profile_rate = (2 * config.SMOOTH_TURN_DECEL * approach) ** 0.5
                command_rate = min(config.SMOOTH_TURN_MAX_RATE,
                                   max(config.SMOOTH_TURN_MIN_RATE, profile_rate))
                self.drive_base.drive(0, direction * command_rate)

                if config.SMOOTH_TURN_DEBUG and now >= next_log_ms:
                    print("SMOOTH_TURN_STATUS", "ms", now, "heading", heading,
                          "remaining", remaining, "rate", rate,
                          "command_rate", command_rate)
                    next_log_ms = now + config.TURN_LOG_INTERVAL_MS
                wait(config.MOTION_POLL_MS)
        except Exception:
            self.drive_base.brake()
            self._telemetry_event("turn", 2, angle)
            raise

    def turn(
        self,
        angle,
        then=Stop.HOLD,
        timeout_ms=None,
        absolute=False,
        precise=True,
    ):
        """Dreht relativ oder absolut mit einem geregelten Ein-Pass-Profil.

        :param angle: Relativer Drehwinkel oder absoluter Zielwinkel in Grad.
        :param then: Motorverhalten nach Abschluss einer unpräzisen Drehung.
        :param timeout_ms: Maximale Dauer der Hauptdrehung in Millisekunden.
        :param absolute: Interpretiert ``angle`` bei ``True`` als absoluten Zielwinkel.
        :param precise: Verwendet die kontinuierliche Ein-Pass-Drehung.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn Ziel oder Zeitlimit nicht erreicht werden.
        """
        if precise:
            return self._turn_one_pass(angle, absolute, timeout_ms)

        self.check_abort()
        self._telemetry_event("turn", 0, angle)
        self.stop_drive()

        start_angle = self.drive_base.angle()
        target_angle = angle if absolute else start_angle + angle

        print(
            "TURN_REQUEST",
            "angle", angle,
            "start", start_angle,
            "target", target_angle,
            "absolute", absolute,
            "precise", precise,
        )

        try:
            turn_timeout = (
                config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms
            )
            turn_then = Stop.BRAKE if precise else then
            if precise:
                delta = target_angle - start_angle
                if delta > 0:
                    compensation = config.TURN_COMPENSATION_DEG
                elif delta < 0:
                    compensation = -config.TURN_COMPENSATION_DEG
                else:
                    compensation = 0
                commanded_angle = (
                    target_angle + compensation
                    if absolute
                    else angle + compensation
                )
            else:
                commanded_angle = angle

            self._turn_once(
                commanded_angle,
                turn_then,
                turn_timeout,
                absolute,
                target_angle,
            )

            if precise:
                self.wait(config.TURN_BRAKE_SETTLE_MS)
                error = target_angle - self.drive_base.angle()
                for attempt in range(config.TURN_CORRECTION_ATTEMPTS):
                    if abs(error) <= config.TURN_COMPLETION_TOLERANCE_DEG:
                        break
                    compensation = (
                        config.TURN_COMPENSATION_DEG
                        if error > 0
                        else -config.TURN_COMPENSATION_DEG
                    )
                    correction = error + compensation
                    print(
                        "TURN_CORRECTION_START",
                        "attempt", attempt + 1,
                        "target", target_angle,
                        "angle", self.drive_base.angle(),
                        "error", error,
                        "command", correction,
                    )
                    self._turn_once(
                        correction,
                        Stop.BRAKE,
                        config.TURN_CORRECTION_TIMEOUT_MS,
                        False,
                        target_angle,
                    )
                    self.wait(config.TURN_BRAKE_SETTLE_MS)
                    error = target_angle - self.drive_base.angle()
                    print(
                        "TURN_CORRECTION_RESULT",
                        "attempt", attempt + 1,
                        "target", target_angle,
                        "angle", self.drive_base.angle(),
                        "error", error,
                    )

                if abs(error) > config.TURN_COMPLETION_TOLERANCE_DEG:
                    print(
                        "TURN_TARGET_TIMEOUT",
                        "target", target_angle,
                        "angle", self.drive_base.angle(),
                        "error", error,
                    )
                    raise MotionTimeout("turn target not reached")

                print(
                    "TURN_ACCURACY_OK",
                    "target", target_angle,
                    "angle", self.drive_base.angle(),
                    "error", error,
                )
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("turn", 2, angle)
            raise

        print(
            "TURN_FINISH",
            "target", target_angle,
            "angle", self.drive_base.angle(),
        )
        self._telemetry_event("turn", 1, angle)
        return True

    def turn_to(self, heading, then=Stop.HOLD, timeout_ms=None, precise=True):
        """Dreht auf einen absoluten Gyro-Zielwinkel.

        :param heading: Absoluter Zielwinkel in Grad.
        :param then: Motorverhalten nach Abschluss einer unpräzisen Drehung.
        :param timeout_ms: Maximale Dauer der Hauptdrehung in Millisekunden.
        :param precise: Verwendet die kontinuierliche Ein-Pass-Drehung.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn Ziel oder Zeitlimit nicht erreicht werden.
        """
        return self.turn(
            heading,
            then=then,
            timeout_ms=timeout_ms,
            absolute=True,
            precise=precise,
        )

    def arc(
        self,
        radius,
        angle=None,
        distance=None,
        then=Stop.HOLD,
        timeout_ms=None,
    ):
        """Fährt einen Kreisbogen mit Winkel- oder Streckenziel.

        :param radius: Kurvenradius in Millimetern.
        :param angle: Optionaler relativer Bogenwinkel in Grad.
        :param distance: Optionale relative Bogenstrecke in Millimetern.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ValueError: Wenn kein oder mehr als ein Ziel angegeben wurde.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        if angle is None and distance is None:
            raise ValueError("arc needs angle or distance")
        if angle is not None and distance is not None:
            raise ValueError("arc accepts angle or distance, not both")

        self.check_abort()
        self._telemetry_event("arc", 0, radius, angle if angle is not None else distance)
        self.stop_drive()
        start_distance = self.drive_base.distance()
        start_angle = self.drive_base.angle()

        if angle is not None:
            self.drive_base.arc(radius, angle=angle, then=then, wait=False)
        else:
            self.drive_base.arc(radius, distance=distance, then=then, wait=False)

        try:
            self._wait_until_done(
                self.drive_base.done,
                config.DRIVE_TIMEOUT_MS if timeout_ms is None else timeout_ms,
                "arc",
                progress=(
                    (lambda: abs(self.drive_base.angle() - start_angle))
                    if angle is not None
                    else (lambda: abs(self.drive_base.distance() - start_distance))
                ),
                min_progress=(
                    config.MOTION_MIN_PROGRESS_DEG
                    if angle is not None
                    else config.MOTION_MIN_PROGRESS_MM
                ),
            )
        except (ProgramAborted, MotionTimeout):
            self.stop_drive()
            self._telemetry_event("arc", 2, radius)
            raise

        self._telemetry_event("arc", 1, radius)
        return True

    def drive(self, speed, turn_rate=0):
        """Fährt kontinuierlich bis zum Programmabbruch.

        :param speed: Fahrgeschwindigkeit in Millimetern pro Sekunde.
        :param turn_rate: Drehrate in Grad pro Sekunde.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        """
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
        """Bewegt einen Motor um einen relativen Winkel.

        :param motor: Zu bewegender Pybricks-Motor.
        :param speed: Motorgeschwindigkeit in Grad pro Sekunde.
        :param angle: Relativer Motorwinkel in Grad.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        self.check_abort()
        self._telemetry_event("motor_angle", 0, speed, angle)
        start_angle = motor.angle()
        motor.run_angle(speed, angle, then=then, wait=False)
        result = self._wait_for_motor(
            motor,
            "motor_angle",
            timeout_ms,
            start_angle,
        )
        self._telemetry_event("motor_angle", 1, speed, angle)
        return result

    def motor_target(self, motor, speed, target, then=Stop.HOLD, timeout_ms=None):
        """Bewegt einen Motor auf einen absoluten Zielwinkel.

        :param motor: Zu bewegender Pybricks-Motor.
        :param speed: Maximale Motorgeschwindigkeit in Grad pro Sekunde.
        :param target: Absoluter Motorzielwinkel in Grad.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        self.check_abort()
        self._telemetry_event("motor_target", 0, speed, target)
        start_angle = motor.angle()
        motor.run_target(speed, target, then=then, wait=False)
        result = self._wait_for_motor(
            motor,
            "motor_target",
            timeout_ms,
            start_angle,
        )
        self._telemetry_event("motor_target", 1, speed, target)
        return result

    def motor_time(self, motor, speed, time, then=Stop.HOLD, timeout_ms=None):
        """Lässt einen Motor für eine festgelegte Dauer laufen.

        :param motor: Zu bewegender Pybricks-Motor.
        :param speed: Motorgeschwindigkeit in Grad pro Sekunde.
        :param time: Laufzeit in Millisekunden.
        :param then: Motorverhalten nach Abschluss der Bewegung.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        self.check_abort()
        self._telemetry_event("motor_time", 0, speed, time)
        start_angle = motor.angle()
        motor.run_time(speed, time, then=then, wait=False)

        if timeout_ms is None:
            timeout_ms = max(config.MOTOR_TIMEOUT_MS, time + 2000)

        result = self._wait_for_motor(
            motor,
            "motor_time",
            timeout_ms,
            start_angle,
        )
        self._telemetry_event("motor_time", 1, speed, time)
        return result

    def _wait_for_motor(self, motor, action_name, timeout_ms, start_angle=None):
        """Wartet überwacht auf den Abschluss einer Motorbewegung.

        :param motor: Überwachter Pybricks-Motor.
        :param action_name: Bewegungsname für Fehlermeldungen.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :param start_angle: Optionaler Startwinkel zur Fortschrittsmessung.
        :return: ``True`` nach erfolgreichem Abschluss.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn das Zeitlimit überschritten wird.
        """
        if timeout_ms is None:
            timeout_ms = config.MOTOR_TIMEOUT_MS

        try:
            self._wait_until_done(
                motor.done,
                timeout_ms,
                action_name,
                progress=(
                    None
                    if start_angle is None
                    else lambda: abs(motor.angle() - start_angle)
                ),
                min_progress=(
                    0
                    if start_angle is None
                    else config.MOTION_MIN_PROGRESS_DEG
                ),
            )
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
        """Bewegt einen Motor bis zur Blockade oder zum Zeitlimit.

        :param motor: Zu bewegender Pybricks-Motor.
        :param speed: Motorgeschwindigkeit in Grad pro Sekunde.
        :param then: Motorverhalten nach erkannter Blockade.
        :param timeout_ms: Maximale Dauer in Millisekunden oder ``None``.
        :return: Motorwinkel beim erkannten Stillstand.
        :raises ValueError: Wenn ``speed`` gleich null ist.
        :raises ProgramAborted: Wenn der Benutzer den Lauf abbricht.
        :raises MotionTimeout: Wenn keine Blockade innerhalb des Zeitlimits erkannt wird.
        """
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

                if motor.stalled():
                    stall_reason = "pybricks"
                    break

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
