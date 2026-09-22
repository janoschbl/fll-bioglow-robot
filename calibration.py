"""Eigenstaendige Kalibrierung fuer genaue Drehungen.

Diese Datei kann in der Code-Werkstatt direkt als Startdatei ausgefuehrt
werden. Sie importiert weder ``main.py`` noch ``programs.py``. Die Messwerte
werden als CSV-Zeilen ausgegeben.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait


RAD_DURCHMESSER_MM = 62.4
ACHSABSTAND_MM = 80

LINKER_ANTRIEB = Port.C
RECHTER_ANTRIEB = Port.D
LINKS_RICHTUNG = Direction.COUNTERCLOCKWISE
RECHTS_RICHTUNG = Direction.CLOCKWISE

TEST_WINKEL = (85, -85, 135, -135)
WIEDERHOLUNGEN = 3

REGEL_INTERVALL_MS = 10
REGEL_TIMEOUT_MS = 8000
REGEL_KP = 2.0
REGEL_MIN_RATE = 14
REGEL_MAX_RATE = 90
REGEL_NAH_MAX_RATE = 28
REGEL_NAH_GRENZE_DEG = 5
ZIEL_TOLERANZ_DEG = 0.75
RUHE_DREHRATE_DEG_S = 4
RUHEZEIT_MS = 180
MESS_PAUSE_MS = 250
TRACE_INTERVALL_MS = 100


def _drehrate(hub):
    try:
        return hub.imu.angular_velocity()[2]
    except Exception:
        return 0


def _begrenzen(wert, minimum, maximum):
    return max(minimum, min(maximum, wert))


def _abbruch_pruefen(hub):
    if Button.CENTER in hub.buttons.pressed():
        raise SystemExit("Kalibrierung am Hub abgebrochen")


def _antrieb_bremsen(drive_base, linker_motor, rechter_motor):
    drive_base.stop()
    linker_motor.brake()
    rechter_motor.brake()


def genaue_drehung(
    hub,
    drive_base,
    linker_motor,
    rechter_motor,
    ziel,
):
    """Regelt auf einen absoluten DriveBase-Winkel, ohne ``done()``."""
    uhr = StopWatch()
    ruhig_seit = None
    naechster_trace = 0
    letzter_befehl = 0

    while True:
        _abbruch_pruefen(hub)

        jetzt = uhr.time()
        winkel = drive_base.angle()
        fehler = ziel - winkel
        drehrate = _drehrate(hub)

        if abs(fehler) <= ZIEL_TOLERANZ_DEG:
            if letzter_befehl != 0:
                drive_base.stop()
                letzter_befehl = 0

            if abs(drehrate) <= RUHE_DREHRATE_DEG_S:
                if ruhig_seit is None:
                    ruhig_seit = jetzt
                elif jetzt - ruhig_seit >= RUHEZEIT_MS:
                    _antrieb_bremsen(
                        drive_base,
                        linker_motor,
                        rechter_motor,
                    )
                    return jetzt
            else:
                ruhig_seit = None
        else:
            ruhig_seit = None
            maximum = (
                REGEL_NAH_MAX_RATE
                if abs(fehler) <= REGEL_NAH_GRENZE_DEG
                else REGEL_MAX_RATE
            )
            befehl = _begrenzen(
                abs(fehler) * REGEL_KP,
                REGEL_MIN_RATE,
                maximum,
            )
            if fehler < 0:
                befehl = -befehl

            drive_base.drive(0, befehl)
            letzter_befehl = befehl

        if jetzt >= naechster_trace:
            print(
                "CAL_TRACE,{},{:.3f},{:.3f},{:.3f},{:.3f}".format(
                    ziel,
                    winkel,
                    fehler,
                    drehrate,
                    letzter_befehl,
                )
            )
            naechster_trace += TRACE_INTERVALL_MS

        if jetzt >= REGEL_TIMEOUT_MS:
            _antrieb_bremsen(
                drive_base,
                linker_motor,
                rechter_motor,
            )
            raise RuntimeError(
                "Drehung nach {} ms nicht stabil: Ziel {}, Winkel {:.3f}, Fehler {:.3f}".format(
                    jetzt,
                    ziel,
                    winkel,
                    fehler,
                )
            )

        wait(REGEL_INTERVALL_MS)


def kalibrieren(hub, drive_base, linker_motor, rechter_motor):
    drive_base.use_gyro(True)
    print("CAL_BEGIN")
    print(
        "CAL_FIELDS,wiederholung,ziel_deg,"
        "endwinkel_deg,fehler_deg,dauer_ms,"
        "ruhe_drehrate_deg_s,bestanden"
    )

    bestanden = 0
    gesamt = 0

    for wiederholung in range(1, WIEDERHOLUNGEN + 1):
        for ziel in TEST_WINKEL:
            _abbruch_pruefen(hub)
            drive_base.reset(distance=drive_base.distance(), angle=0)
            wait(MESS_PAUSE_MS)

            dauer = genaue_drehung(
                hub,
                drive_base,
                linker_motor,
                rechter_motor,
                ziel,
            )
            wait(MESS_PAUSE_MS)

            endwinkel = drive_base.angle()
            fehler = ziel - endwinkel
            ruhe_drehrate = _drehrate(hub)
            ist_bestanden = abs(fehler) < 2
            gesamt += 1
            if ist_bestanden:
                bestanden += 1

            print(
                "CAL_RESULT,{},{},{:.3f},{:.3f},{},{:.3f},{}".format(
                    wiederholung,
                    ziel,
                    endwinkel,
                    fehler,
                    dauer,
                    ruhe_drehrate,
                    1 if ist_bestanden else 0,
                )
            )

    _antrieb_bremsen(drive_base, linker_motor, rechter_motor)
    print("CAL_SUMMARY,{},{},{}".format(bestanden, gesamt, gesamt - bestanden))
    print("CAL_END")


def run(robot):
    """Kompatibler Einstieg fuer einen optionalen Menue-Aufruf."""
    kalibrieren(
        robot.hub,
        robot.drive_base,
        robot.left_drive_motor,
        robot.right_drive_motor,
    )


def main():
    hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)

    linker_motor = Motor(LINKER_ANTRIEB, LINKS_RICHTUNG)
    rechter_motor = Motor(RECHTER_ANTRIEB, RECHTS_RICHTUNG)
    drive_base = DriveBase(
        linker_motor,
        rechter_motor,
        wheel_diameter=RAD_DURCHMESSER_MM,
        axle_track=ACHSABSTAND_MM,
    )

    try:
        kalibrieren(hub, drive_base, linker_motor, rechter_motor)
    finally:
        _antrieb_bremsen(drive_base, linker_motor, rechter_motor)


if __name__ == "__main__":
    main()
