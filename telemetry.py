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
    "reset_heading": 12,
    # Programmwechsel innerhalb einer Aufnahme; wert1 = Programmnummer im Menü.
    "program": 13,
}

SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)

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
    """Zeichnet Messpunkte und Befehle auf und sendet sie gesammelt.

    Aufnahme und Senden steuert das Programm selbst über
    ``robot.start_telemetry()`` und ``robot.send_telemetry()``. Ohne Aufnahme
    kostet die Telemetrie keine Rechenzeit.
    """

    def __init__(self, hub, drive_base, motors, force_sensor=None):
        self.hub = hub
        self.drive_base = drive_base
        self.motors = motors
        self.force_sensor = force_sensor
        self.clock = StopWatch()
        self.app_data = None
        self.buffer = None
        self.length = 0

        self.active = False
        self.run_id = 0
        self.run_name = ""
        self.sample_ms = config.TELEMETRY_SAMPLE_MS
        self.started_ms = 0
        self.next_sample_ms = 0
        self.samples = 0
        self.dropped = 0
        self.events = 0

    def _reserve(self):
        # Ein einziger fester Block statt wachsendem bytearray: Beim Vergrößern
        # braucht MicroPython alten und neuen Block gleichzeitig, und ab etwa
        # 60 KB gingen dadurch Messpunkte und Befehle verloren.
        if self.buffer is not None:
            return True
        size = config.TELEMETRY_BUFFER_BYTES
        while size >= 8000:
            try:
                self.buffer = bytearray(size)
                return True
            except MemoryError:
                size //= 2
        config.protokolliere("TELEMETRY_NO_MEMORY")
        return False

    def begin(self, name, sample_ms=None):
        """Beginnt eine neue Aufnahme; eine laufende wird verworfen."""
        if not self._reserve():
            return
        self.run_id = (self.run_id + 1) & 0x3FFFFFFF
        self.run_name = str(name)[:80]
        self.sample_ms = max(10, int(sample_ms or config.TELEMETRY_SAMPLE_MS))
        self.started_ms = self.clock.time()
        self.next_sample_ms = self.started_ms
        self.length = 0
        self.samples = 0
        self.dropped = 0
        self.events = 0
        self.active = True
        self.tick(force=True)

    def _metadata(self):
        return struct.pack(
            "<HHHHH",
            self.sample_ms,
            int(round(config.WHEEL_DIAMETER_MM * 10)),
            int(round(config.AXLE_TRACK_MM * 10)),
            SAMPLE_SIZE,
            EVENT_SIZE,
        ) + bytes(self.run_name, "utf-8")

    def _thin_out(self):
        """Verwirft jeden zweiten Messpunkt und verdoppelt den Abstand.

        Befehle bleiben vollständig erhalten. Kopiert wird über
        unpack_from/pack_into, das jede MicroPython-Fassung kann.
        """
        read = write = 0
        keep = True
        while read < self.length:
            is_sample = self.buffer[read] == 1
            form = SAMPLE_FORMAT if is_sample else EVENT_FORMAT
            size = SAMPLE_SIZE if is_sample else EVENT_SIZE
            if not is_sample or keep:
                if write != read:
                    struct.pack_into(form, self.buffer, write, *struct.unpack_from(form, self.buffer, read))
                write += size
            else:
                self.samples -= 1
                self.dropped += 1
            if is_sample:
                keep = not keep
            read += size
        self.length = write
        self.sample_ms *= 2

    def _room(self, size):
        if self.length + size <= len(self.buffer):
            return True
        if self.samples > 1:
            self._thin_out()
        if self.length + size <= len(self.buffer):
            return True
        self.dropped += 1
        return False

    def tick(self, force=False):
        if not self.active:
            return
        now = self.clock.time()
        if not force and now < self.next_sample_ms:
            return
        while self.next_sample_ms <= now:
            self.next_sample_ms += self.sample_ms
        if not self._room(SAMPLE_SIZE):
            return

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

            struct.pack_into(
                SAMPLE_FORMAT,
                self.buffer,
                self.length,
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
            self.length += SAMPLE_SIZE
            self.samples += 1
        except Exception as error:
            self.dropped += 1
            if self.dropped == 1:
                config.protokolliere("TELEMETRY_SAMPLE_ERROR", str(error))

    def event(self, name, phase, value1=0, value2=0):
        if not self.active or not self._room(EVENT_SIZE):
            return
        struct.pack_into(
            EVENT_FORMAT,
            self.buffer,
            self.length,
            2,
            self.clock.time() - self.started_ms,
            EVENT_IDS.get(name, 0),
            phase,
            _clamp(value1, -1073741823, 1073741823),
            _clamp(value2, -1073741823, 1073741823),
        )
        self.length += EVENT_SIZE
        self.events += 1

    def _packet(self, kind, sequence, payload):
        header = struct.pack("<4sBIHH", MAGIC, kind, self.run_id, sequence, len(payload))
        return header + struct.pack("<H", _crc16(header + payload)) + payload

    def _send(self, packet):
        if self.app_data is None and AppData is not None:
            try:
                self.app_data = AppData([])
            except Exception as error:
                config.protokolliere("TELEMETRY_APPDATA_UNAVAILABLE", str(error))
        if self.app_data is not None:
            self.app_data.write_bytes(packet)
            return
        if ubinascii is None:
            raise RuntimeError("Kein AppData- oder Base64-Transport verfügbar")
        # Rückfall ohne AppData geht über die Konsole und ist deshalb nicht abschaltbar.
        print("@BGB1", ubinascii.b2a_base64(packet).decode().strip())

    def finish_and_send(self, outcome):
        """Beendet die Aufnahme und überträgt sie an die Code-Werkstatt."""
        if not self.active:
            return
        self.tick(force=True)
        self.active = False
        # Die Laufzeit endet hier; die Übertragung danach zählt nicht mit.
        duration_ms = self.clock.time() - self.started_ms
        sequence = 0
        self._send(self._packet(0, sequence, self._metadata()))
        sequence += 1
        for offset in range(0, self.length, PACKET_PAYLOAD):
            self._send(self._packet(1, sequence, bytes(self.buffer[offset:min(offset + PACKET_PAYLOAD, self.length)])))
            sequence += 1
        end = struct.pack(
            END_FORMAT,
            3,
            duration_ms,
            self.samples,
            self.events,
            self.dropped,
            OUTCOMES.get(outcome, OUTCOMES["error"]),
        )
        self._send(self._packet(2, sequence, end))
        config.protokolliere("TELEMETRY_SENT", self.run_id, self.samples, self.events, self.dropped, outcome)
