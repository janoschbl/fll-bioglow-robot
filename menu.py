from pybricks.parameters import Button
from pybricks.tools import StopWatch, wait

from robot import MotionTimeout, ProgramAborted


PROGRAM_POSITIONS = [
    (x, y)
    for y in range(3)
    for x in range(5)
]

DEBUG_POSITIONS = [
    (0, 3),
    (4, 3),
    (0, 4),
    (4, 4),
]

INDICATOR_POS = (2, 3)
PROGRAM_BRIGHTNESS = 45
DEBUG_BRIGHTNESS = [45, 45, 75, 75]


class Menu:
    def __init__(self, hub, robot, force_sensor=None):
        self.hub = hub
        self.robot = robot
        self.force_sensor = force_sensor

        self.program_slots = [None] * len(PROGRAM_POSITIONS)
        self.debug_slots = [None] * len(DEBUG_POSITIONS)

        self.running_program = None
        self.previous_buttons = set()
        self.active_debug_motor = None
        self.selected_menu_index = -1

        self.watch = StopWatch()
        self.debounce_time = 200
        self.last_action_time = 0

    def px(self, x, y, light):
        self.hub.display.pixel(y, x, light)

    def _next_free_slot(self, slots, label):
        for index, entry in enumerate(slots):
            if entry is None:
                return index + 1
        raise ValueError("No free " + label + " slots")

    def _register_entry(self, slots, entry, index, label):
        if index is None:
            index = self._next_free_slot(slots, label)

        if index < 1 or index > len(slots):
            raise ValueError(label + " index out of range")
        if slots[index - 1] is not None:
            raise ValueError(label + " slot already used")

        slots[index - 1] = entry
        return entry

    def program(self, index=None, name=None):
        def decorator(func):
            entry = {
                "name": name if name else func.__name__,
                "type": "function",
                "func": func,
            }
            self._register_entry(self.program_slots, entry, index, "Program")
            return func

        return decorator

    def register_motor_debug(self, motor, direction=1, index=None, name="Motor Debug"):
        entry = {
            "name": name,
            "type": "motor_debug",
            "motor": motor,
            "direction": direction,
        }
        self._register_entry(self.debug_slots, entry, index, "Debug")

    def build_entries(self):
        entries = []

        for index, entry in enumerate(self.program_slots):
            if entry is not None:
                entries.append({
                    "group": "program",
                    "slot_index": index,
                    "entry": entry,
                })

        for index, entry in enumerate(self.debug_slots):
            if entry is not None:
                entries.append({
                    "group": "debug",
                    "slot_index": index,
                    "entry": entry,
                })

        return entries

    def next_program_index(self, selected_menu_index):
        program_indexes = []

        for index, item in enumerate(self.build_entries()):
            if item["group"] == "program":
                program_indexes.append(index)

        if not program_indexes:
            return selected_menu_index
        if selected_menu_index not in program_indexes:
            return program_indexes[0]

        current = program_indexes.index(selected_menu_index)
        return program_indexes[(current + 1) % len(program_indexes)]

    def write_display(self):
        self.hub.display.off()
        entries = self.build_entries()
        selected = None

        if 0 <= self.selected_menu_index < len(entries):
            selected = entries[self.selected_menu_index]

        for index, entry in enumerate(self.program_slots):
            if entry is None:
                continue

            x, y = PROGRAM_POSITIONS[index]
            light = PROGRAM_BRIGHTNESS
            if selected and selected["group"] == "program" and selected["slot_index"] == index:
                light = 100
            self.px(x, y, light)

        for index, entry in enumerate(self.debug_slots):
            if entry is None:
                continue

            x, y = DEBUG_POSITIONS[index]
            light = DEBUG_BRIGHTNESS[index]
            if selected and selected["group"] == "debug" and selected["slot_index"] == index:
                light = 100
            self.px(x, y, light)

        indicator = 20
        if selected and selected["group"] == "debug":
            indicator = 80 if selected["entry"]["direction"] == 1 else 40

        self.px(INDICATOR_POS[0], INDICATOR_POS[1], indicator)

    def initial_animation(self):
        size = 5

        while size > 0:
            start = (5 - size) // 2
            end = start + size

            for x in range(5):
                for y in range(5):
                    on_vertical = (x == start or x == end - 1) and start <= y < end
                    on_horizontal = (y == start or y == end - 1) and start <= x < end
                    self.px(x, y, 100 if on_vertical or on_horizontal else 0)

            wait(100)
            self.hub.display.off()
            size -= 2

        while not self.hub.imu.ready():
            self.px(2, 2, 100)
            wait(500)
            self.hub.display.off()
            wait(500)

        self.px(2, 2, 100)
        wait(100)
        self.hub.display.off()
        wait(100)

    def stop_active_debug_motor(self):
        if self.active_debug_motor is not None:
            self.robot.stop_attachment(self.active_debug_motor)
            self.active_debug_motor = None

    def update_debug_motor(self, selected_entry):
        if selected_entry is None or selected_entry["type"] != "motor_debug":
            self.stop_active_debug_motor()
            return
        if self.force_sensor is None:
            self.stop_active_debug_motor()
            return

        motor = selected_entry["motor"]
        direction = selected_entry["direction"]

        try:
            force_pressed = self.force_sensor.pressed()
        except Exception as error:
            print("FORCE_SENSOR_ERROR", str(error))
            self.stop_active_debug_motor()
            return

        if force_pressed:
            try:
                force = max(0, min(10, self.force_sensor.force()))
                speed = 100 + int(force * force * 9)
            except Exception as error:
                print("FORCE_SENSOR_ERROR", str(error))
                speed = 200

            if self.active_debug_motor is not None and self.active_debug_motor is not motor:
                self.stop_active_debug_motor()

            motor.run(speed * direction)
            self.active_debug_motor = motor
        elif self.active_debug_motor is motor:
            self.stop_active_debug_motor()
        else:
            self.robot.stop_attachment(motor)

    def run_selected_program(self, entry):
        self.stop_active_debug_motor()
        self.robot.emergency_stop()
        self.robot.reset_drivebase_settings()

        self.running_program = entry
        self.robot.begin_program(entry["name"])
        self.hub.display.off()
        outcome = "success"

        try:
            entry["func"]()
        except ProgramAborted:
            outcome = "aborted"
            print("PROGRAM_ABORTED", entry["name"])
        except MotionTimeout as error:
            outcome = "timeout"
            print("PROGRAM_TIMEOUT", entry["name"], str(error))
        except Exception as error:
            outcome = "error"
            print("PROGRAM_ERROR", entry["name"], str(error))
        finally:
            self.robot.emergency_stop()
            self.robot.reset_drivebase_settings()
            try:
                self.robot.end_program(outcome)
            except Exception as error:
                print("TELEMETRY_SEND_ERROR", str(error))
            self.running_program = None
            self.previous_buttons = set(self.hub.buttons.pressed())
            self.hub.display.off()
            wait(150)

    def run(self):
        self.initial_animation()

        entries = self.build_entries()
        self.selected_menu_index = 0 if entries else -1
        self.previous_buttons = set(self.hub.buttons.pressed())
        self.write_display()

        print("DriveBase settings:")
        print(self.robot.drive_base.settings())

        while True:
            buttons = set(self.hub.buttons.pressed())
            new_buttons = buttons - self.previous_buttons
            current_time = self.watch.time()
            entries = self.build_entries()

            if entries:
                if not 0 <= self.selected_menu_index < len(entries):
                    self.selected_menu_index = 0

                if Button.LEFT in new_buttons and current_time - self.last_action_time > self.debounce_time:
                    self.selected_menu_index = (self.selected_menu_index - 1) % len(entries)
                    self.last_action_time = current_time
                elif Button.RIGHT in new_buttons and current_time - self.last_action_time > self.debounce_time:
                    self.selected_menu_index = (self.selected_menu_index + 1) % len(entries)
                    self.last_action_time = current_time
                elif Button.CENTER in new_buttons and current_time - self.last_action_time > self.debounce_time:
                    selected = entries[self.selected_menu_index]["entry"]
                    self.last_action_time = current_time

                    if selected["type"] == "function":
                        self.run_selected_program(selected)
                        self.selected_menu_index = self.next_program_index(self.selected_menu_index)
                        buttons = set(self.hub.buttons.pressed())
                    else:
                        selected["direction"] *= -1

                entries = self.build_entries()
                selected = entries[self.selected_menu_index]["entry"]
                self.update_debug_motor(selected)
            else:
                self.stop_active_debug_motor()

            self.write_display()
            self.previous_buttons = buttons
            wait(10)
