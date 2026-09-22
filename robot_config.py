from pybricks.parameters import Direction, Port

LEFT_ATTACHMENT_PORT = Port.A
RIGHT_ATTACHMENT_PORT = Port.B
LEFT_DRIVE_PORT = Port.C
RIGHT_DRIVE_PORT = Port.D
FORCE_SENSOR_PORT = Port.F

LEFT_DRIVE_DIRECTION = Direction.COUNTERCLOCKWISE
RIGHT_DRIVE_DIRECTION = Direction.CLOCKWISE

WHEEL_DIAMETER_MM = 62.4
AXLE_TRACK_MM = 80

# TODO drivebase defaults aus benchmark, ggf noch ändern basierend auf kommenden Benchmarks mit neuem Roboter!
DEFAULT_STRAIGHT_SPEED = 450
DEFAULT_STRAIGHT_ACCELERATION = 700
DEFAULT_TURN_RATE = 100
DEFAULT_TURN_ACCELERATION = 300

DEFAULT_DRIVEBASE_SETTINGS = (
    DEFAULT_STRAIGHT_SPEED,
    DEFAULT_STRAIGHT_ACCELERATION,
    DEFAULT_TURN_RATE,
    DEFAULT_TURN_ACCELERATION,
)


# safety timeouts
MOTION_POLL_MS = 10
# A newly queued asynchronous command needs at least one control-cycle before
# ``done()`` can be trusted.  This is deliberately a short launch fence, not
# a multi-sample completion debounce: HOLD can make ``done()`` oscillate.
MOTION_START_GUARD_MS = 30
MOTION_MIN_PROGRESS_DEG = 0.5
MOTION_MIN_PROGRESS_MM = 2
DRIVE_TIMEOUT_MS = 20000
MOTOR_TIMEOUT_MS = 10000

# Geradeausfahrt: Reaktions- und Bremsweg werden aus der eingestellten
# Geschwindigkeit angenaehert. Bei 450 mm/s sind 20 ms rund neun Millimeter.
STRAIGHT_BRAKE_REACTION_MS = 20
STRAIGHT_MIN_BRAKE_LEAD_MM = 3
STRAIGHT_BRAKE_SETTLE_MS = 50
STRAIGHT_COMPLETION_TOLERANCE_MM = 15

# Heading completion. Die schnelle Drehung wird beim ersten Erreichen des
# echten Gyro-Ziels aktiv gebremst; es gibt keine zweite Korrekturbewegung.
TURN_POSITION_TOLERANCE_DEG = 1
# Drei reale 180-Grad-Laeufe drifteten nach dem Bremsbefehl noch
# 1,65 bis 1,74 Grad weiter. Entsprechend frueh wird gebremst.
TURN_BRAKE_LEAD_DEG = 1.7
# Nur die eigene Gyro-Messung entscheidet, ob die gesamte Drehung fertig ist.
# Pybricks done() beendet lediglich einen einzelnen Bewegungsabschnitt.
TURN_COMPLETION_TOLERANCE_DEG = 2
TURN_CORRECTION_ATTEMPTS = 2
TURN_CORRECTION_TIMEOUT_MS = 4000

# Reale Messung am Roboter fuer +/-33 und +/-47 Grad: 12/12 Laeufe unter
# 2 Grad Fehler und 1,5 Sekunden. Der DriveBase-Regler meldet sein Ende etwa
# sechs Grad vor dem echten Gyro-Ziel; diese Totzone wird direkt vorgegeben.
TURN_COMPENSATION_DEG = 6
TURN_BRAKE_SETTLE_MS = 120
# Manche Pybricks-Versionen lassen ``DriveBase.done()`` trotz erreichtem
# Gyro-Ziel auf False, insbesondere bei einer 180-Grad-Drehung. Ist der echte
# Zielwinkel fuer diese Dauer praktisch still erreicht, gilt die Drehung
# trotzdem als beendet.
TURN_LOG_INTERVAL_MS = 250

# stall fallback über encoder fortschritt
STALL_TIMEOUT_MS = 6000
STALL_PROGRESS_WINDOW_MS = 400
STALL_MIN_PROGRESS_DEG = 3
STALL_MIN_PROGRESS_RATIO = 0.10

# Run-Telemetrie: während des gesamten Programms mit 50 Hz puffern und erst
# nach Programmende als einen vollständigen Run übertragen.
TELEMETRY_SAMPLE_MS = 20
TELEMETRY_MAX_BYTES = 180000
