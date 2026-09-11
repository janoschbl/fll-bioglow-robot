try:
    import ustruct as struct
except ImportError:
    import struct

try:
    import ubinascii
except ImportError:
    ubinascii = None

from pybricks.tools import StopWatch

try:
    from pybricks.messaging import AppData
except ImportError:
    AppData = None

import robot_config as config


MAGIC = b"BGT1"
PACKET_PAYLOAD = 180
SAMPLE_FORMAT = "<BIhhhhhh9hi4i4h4hihihhH"
EVENT_FORMAT = "<BIBBii"
END_FORMAT = "<BIIHHB"

EVENT_IDS = {
    "straight": 1,
    "turn": 2,
    "arc": 3,
    "wait": 4,
    "motor_angle": 5,
    "motor_target": 6,
    "motor_time": 7,
    "motor_until_stalled": 8,
    "settings": 9,
    "gyro": 10,
    "drive": 11,
}

OUTCOMES = {
    "success": 0,
    "aborted": 1,
    "timeout": 2,
    "error": 3,
}


def _clamp(value, low, high):
    try:
        value = int(round(value))
    except Exception:
        return 0
    return min(high, max(low, value))


def _crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Telemetry:
    def __init__(self, hub, drive_base, motors, force_sensor=None):
        self.hub = hub
        self.drive_base = drive_base
        self.motors = motors
        self.force_sensor = force_sensor
        self.clock = StopWatch()
        self.app_data = None
        if AppData is not None:
            try:
                self.app_data = AppData([])
            except Exception as error:
                print("TELEMETRY_APPDATA_UNAVAILABLE", str(error))

        self.active = False
        self.data = bytearray()
        self.run_id = 0
        self.run_name = ""
        self.started_ms = 0
        self.next_sample_ms = 0
        self.samples = 0
        self.dropped = 0
        self.events = 0
        self.buffer_full = False

    def begin(self, name):
        # Pybricks verwendet kleine 30-Bit-Integer. 0xFFFFFFFF und die vollen
        # int32-Grenzen lösen bereits beim Laden des Moduls einen Overflow aus.
        self.run_id = (self.run_id + 1) & 0x3FFFFFFF
        self.run_name = str(name)[:80]
        self.started_ms = self.clock.time()
        self.next_sample_ms = self.started_ms
        self.data = bytearray()
        self.samples = 0
        self.dropped = 0
        self.events = 0
        self.buffer_full = False
        self.active = True
        self._send_metadata()
        self.tick(force=True)

    def _metadata(self):
        return struct.pack(
            "<HHHHH",
            config.TELEMETRY_SAMPLE_MS,
            config.WHEEL_DIAMETER_MM,
            config.AXLE_TRACK_MM,
            struct.calcsize(SAMPLE_FORMAT),
            struct.calcsize(EVENT_FORMAT),
        ) + bytes(self.run_name, "utf-8")

    def _send_metadata(self):
        try:
            self._send(self._packet(0, 0, self._metadata()))
        except Exception as error:
            print("TELEMETRY_LIVE_START_ERROR", str(error))

    def _append(self, record):
        if self.buffer_full or len(self.data) + len(record) > config.TELEMETRY_MAX_BYTES:
            self.buffer_full = True
            self.dropped += 1
            return False
        try:
            self.data.extend(record)
        except MemoryError:
            # Telemetrie darf die Robotersteuerung auch bei knappem RAM nie stoppen.
            self.buffer_full = True
            self.dropped += 1
            return False
        return True

    def tick(self, force=False):
        if not self.active:
            return
        now = self.clock.time()
        if not force and now < self.next_sample_ms:
            return
        while self.next_sample_ms <= now:
            self.next_sample_ms += config.TELEMETRY_SAMPLE_MS

        try:
            acceleration = self.hub.imu.acceleration()
            angular = self.hub.imu.angular_velocity()
            orientation = self.hub.imu.orientation()
            matrix = []
            for row in range(3):
                for column in range(3):
                    matrix.append(_clamp(orientation[row, column] * 32767, -32767, 32767))
            motor_angles = [_clamp(motor.angle(), -1073741823, 1073741823) for motor in self.motors]
            motor_speeds = [_clamp(motor.speed(), -32768, 32767) for motor in self.motors]
            motor_loads = []
            flags = 0
            for index, motor in enumerate(self.motors):
                try:
                    motor_loads.append(_clamp(motor.load(), -32768, 32767))
                    if motor.stalled():
                        flags |= 1 << index
                    if motor.done():
                        flags |= 1 << (index + 4)
                except Exception:
                    motor_loads.append(0)
            drive = self.drive_base.state()
            force = -32768
            if self.force_sensor is not None:
                try:
                    force = _clamp(self.force_sensor.force() * 100, -32767, 32767)
                    flags |= 1 << 8
                except Exception:
                    pass
            if self.hub.imu.ready():
                flags |= 1 << 9
            if self.hub.imu.stationary():
                flags |= 1 << 10

            record = struct.pack(
                SAMPLE_FORMAT,
                1,
                now - self.started_ms,
                *[_clamp(value / 4, -32768, 32767) for value in acceleration],
                *[_clamp(value * 10, -32768, 32767) for value in angular],
                *matrix,
                _clamp(self.hub.imu.heading() * 10, -1073741823, 1073741823),
                *motor_angles,
                *motor_speeds,
                *motor_loads,
                _clamp(drive[0], -1073741823, 1073741823),
                _clamp(drive[1], -32768, 32767),
                _clamp(drive[2] * 10, -1073741823, 1073741823),
                _clamp(drive[3] * 10, -32768, 32767),
                force,
                flags,
            )
            if self._append(record):
                self.samples += 1
                try:
                    self._send(self._packet(3, self.samples, record))
                except Exception as error:
                    if self.samples == 1:
                        print("TELEMETRY_LIVE_SAMPLE_ERROR", str(error))
        except Exception as error:
            self.dropped += 1
            if self.dropped == 1:
                print("TELEMETRY_SAMPLE_ERROR", str(error))

    def event(self, name, phase, value1=0, value2=0):
        if not self.active:
            return
        identifier = EVENT_IDS.get(name, 0)
        record = struct.pack(
            EVENT_FORMAT,
            2,
            self.clock.time() - self.started_ms,
            identifier,
            phase,
            _clamp(value1, -1073741823, 1073741823),
            _clamp(value2, -1073741823, 1073741823),
        )
        if self._append(record):
            self.events += 1

    def _packet(self, kind, sequence, payload):
        header = struct.pack("<4sBIHH", MAGIC, kind, self.run_id, sequence, len(payload))
        return header + struct.pack("<H", _crc16(header + payload)) + payload

    def _send(self, packet):
        if self.app_data is not None:
            self.app_data.write_bytes(packet)
            return
        if ubinascii is None:
            raise RuntimeError("Kein AppData- oder Base64-Transport verfügbar")
        print("@BGB1", ubinascii.b2a_base64(packet).decode().strip())

    def finish_and_send(self, outcome):
        if not self.active:
            return
        self.tick(force=True)
        self.active = False
        sequence = 0
        self._send(self._packet(0, sequence, self._metadata()))
        sequence += 1
        for offset in range(0, len(self.data), PACKET_PAYLOAD):
            self._send(self._packet(1, sequence, self.data[offset:offset + PACKET_PAYLOAD]))
            sequence += 1
        end = struct.pack(
            END_FORMAT,
            3,
            self.clock.time() - self.started_ms,
            self.samples,
            self.events,
            self.dropped,
            OUTCOMES.get(outcome, OUTCOMES["error"]),
        )
        self._send(self._packet(2, sequence, end))
        print("TELEMETRY_SENT", self.run_id, self.samples, self.events, self.dropped, outcome)
