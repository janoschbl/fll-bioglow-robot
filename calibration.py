"""Kalibriert den Ein-Pass-Turn direkt auf dem verbundenen Roboter.

Das Programm vergleicht Bremsverzögerungen mit derselben Robot.turn()-Regelung,
die auch im normalen Lauf verwendet wird. Jeder Messlauf dreht nur in eine
Richtung und wird vor dem nächsten Versuch angehalten und neu genullt.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

import robot_config as config
from robot import MotionTimeout, ProgramAborted, Robot


# Nur die passive Bremsverzögerung variiert; das Fahrprofil bleibt gleich.
TEST_BREMSVERZOEGERUNGEN = (1500, 1800, 2100)
TEST_WINKEL_DEG = (33, -33, 47, -47, 180, -180)
WIEDERHOLUNGEN = 2

REGEL_PAUSE_MS = 150
MAX_DAUER_MS = 6000


def _warte_auf_gyro(robot):
    while not robot.hub.imu.ready():
        robot.check_abort()
        wait(100)


def _messlauf(robot, zielwinkel):
    """Führt eine Ein-Pass-Drehung aus und liefert Zeit und Winkelfehler."""
    drive_base = robot.drive_base
    drive_base.brake()
    drive_base.reset(distance=drive_base.distance(), angle=0)
    wait(REGEL_PAUSE_MS)

    uhr = StopWatch()
    try:
        robot.turn(zielwinkel, timeout_ms=MAX_DAUER_MS)
        erfolgreich = True
    except MotionTimeout as error:
        print("CAL_TIMEOUT", zielwinkel, str(error))
        erfolgreich = False

    dauer = uhr.time()
    endwinkel = drive_base.angle()
    fehler = zielwinkel - endwinkel
    bestanden = (
        erfolgreich
        and dauer < MAX_DAUER_MS
        and abs(fehler) <= config.SMOOTH_TURN_ACCEPT_DEG
    )
    return dauer, endwinkel, fehler, bestanden


def kalibrieren(robot):
    """Vergleicht Bremsverzögerungen und meldet das genaueste Profil."""
    drive_base = robot.drive_base
    robot.set_gyro_use(True)
    robot.reset_drivebase_settings()
    _warte_auf_gyro(robot)

    print("CAL_BEGIN")
    print(
        "CAL_FIELDS,bremsverzoegerung_deg_s2,wiederholung,ziel_deg,endwinkel_deg,"
        "fehler_deg,dauer_ms,bestanden"
    )

    ergebnisse = []
    try:
        for bremsverzoegerung in TEST_BREMSVERZOEGERUNGEN:
            config.SMOOTH_TURN_BRAKE_DECEL = bremsverzoegerung
            fehlerwerte = []
            dauerwerte = []
            bestanden_anzahl = 0
            messungen = 0

            print(
                "CAL_PROFILE",
                "max_rate", config.SMOOTH_TURN_MAX_RATE,
                "min_rate", config.SMOOTH_TURN_MIN_RATE,
                "decel", config.SMOOTH_TURN_DECEL,
                "delay_ms", config.SMOOTH_TURN_STOP_DELAY_MS,
                "offset_deg", config.SMOOTH_TURN_STOP_OFFSET_DEG,
                "brake_decel", bremsverzoegerung,
            )

            for wiederholung in range(1, WIEDERHOLUNGEN + 1):
                for zielwinkel in TEST_WINKEL_DEG:
                    robot.check_abort()
                    dauer, endwinkel, fehler, bestanden = _messlauf(
                        robot,
                        zielwinkel,
                    )
                    messungen += 1
                    bestanden_anzahl += 1 if bestanden else 0
                    fehlerwerte.append(abs(fehler))
                    dauerwerte.append(dauer)

                    print(
                        "CAL_RESULT,{},{},{},{:.3f},{:.3f},{},{}".format(
                            bremsverzoegerung,
                            wiederholung,
                            zielwinkel,
                            endwinkel,
                            fehler,
                            dauer,
                            1 if bestanden else 0,
                        )
                    )

            mittel_fehler = sum(fehlerwerte) / len(fehlerwerte)
            groesster_fehler = max(fehlerwerte)
            mittel_dauer = sum(dauerwerte) / len(dauerwerte)
            profil = {
                "bremsverzoegerung": bremsverzoegerung,
                "bestanden": bestanden_anzahl,
                "messungen": messungen,
                "mittel_fehler": mittel_fehler,
                "groesster_fehler": groesster_fehler,
                "mittel_dauer": mittel_dauer,
            }
            ergebnisse.append(profil)
            print(
                "CAL_SUMMARY,{},{},{},{:.3f},{:.3f},{:.1f}".format(
                    bremsverzoegerung,
                    bestanden_anzahl,
                    messungen,
                    mittel_fehler,
                    groesster_fehler,
                    mittel_dauer,
                )
            )

    finally:
        robot.brake_drive()
        robot.reset_drivebase_settings()

    bestes_profil = min(
        ergebnisse,
        key=lambda profil: (
            -profil["bestanden"],
            profil["groesster_fehler"],
            profil["mittel_fehler"],
            profil["mittel_dauer"],
        ),
    )
    config.SMOOTH_TURN_BRAKE_DECEL = bestes_profil["bremsverzoegerung"]
    print(
        "CAL_BEST,{},{},{},{:.3f},{:.3f},{:.1f}".format(
            bestes_profil["bremsverzoegerung"],
            bestes_profil["bestanden"],
            bestes_profil["messungen"],
            bestes_profil["mittel_fehler"],
            bestes_profil["groesster_fehler"],
            bestes_profil["mittel_dauer"],
        )
    )
    if bestes_profil["bestanden"] != bestes_profil["messungen"]:
        print("CAL_WARNING", "Keine Bremsverzögerung bestand alle Messungen")
    print("CAL_END")


def run(robot):
    """Einstieg für einen optionalen Menüaufruf."""
    kalibrieren(robot)


def main():
    hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)

    linker_motor = Motor(config.LEFT_DRIVE_PORT, config.LEFT_DRIVE_DIRECTION)
    rechter_motor = Motor(config.RIGHT_DRIVE_PORT, config.RIGHT_DRIVE_DIRECTION)
    drive_base = DriveBase(
        linker_motor,
        rechter_motor,
        wheel_diameter=config.WHEEL_DIAMETER_MM,
        axle_track=config.AXLE_TRACK_MM,
    )
    robot = Robot(hub, drive_base, None, None, linker_motor, rechter_motor)

    outcome = "success"
    try:
        robot.begin_program("smooth_turn_calibration")
        kalibrieren(robot)
    except ProgramAborted:
        outcome = "aborted"
        raise
    except Exception:
        outcome = "error"
        raise
    finally:
        robot.brake_drive()
        robot.end_program(outcome)


if __name__ == "__main__":
    main()
