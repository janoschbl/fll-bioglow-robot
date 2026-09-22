"""Eigenstaendiger Geschwindigkeitstest fuer genaue Drehungen.

Diese Datei wird in der Code-Werkstatt direkt als Startdatei ausgefuehrt.
Sie importiert weder ``main.py`` noch ``programs.py``. Die normale
DriveBase-Regelung erledigt die schnelle Grobdrehung; nur ein tatsaechlich
verbliebener Fehler wird mit einer kurzen zweiten Drehung korrigiert.
"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait


RAD_DURCHMESSER_MM = 62.4
ACHSABSTAND_MM = 80

LINKER_ANTRIEB = Port.C
RECHTER_ANTRIEB = Port.D
LINKS_RICHTUNG = Direction.COUNTERCLOCKWISE
RECHTS_RICHTUNG = Direction.CLOCKWISE

TEST_WINKEL = (33, -33, 47, -47)
# Die Soll-Drehrate bleibt bewusst bei 100 Grad/s. Untersucht wird, welche
# Beschleunigung schnell anfaehrt, ohne die Genauigkeit zu verschlechtern.
TEST_PROFILE = ((100, 300), (100, 600), (100, 900))
WIEDERHOLUNGEN = 3

REGEL_INTERVALL_MS = 5
STARTSCHUTZ_MS = 30
MIN_FORTSCHRITT_DEG = 0.5
ZIEL_TOLERANZ_DEG = 1
KORREKTUR_GRENZE_DEG = 1.25
KORREKTUR_RATE = 60
KORREKTUR_BESCHLEUNIGUNG = 900
MAX_DAUER_MS = 1500
MESS_PAUSE_MS = 150


def _abbruch_pruefen(hub):
    if Button.CENTER in hub.buttons.pressed():
        raise SystemExit("Kalibrierung am Hub abgebrochen")


def _antrieb_bremsen(drive_base, linker_motor, rechter_motor):
    drive_base.stop()
    linker_motor.brake()
    rechter_motor.brake()


def _warte_fahrbefehl(hub, drive_base, startwinkel, gesamt_uhr):
    """Wartet auf den ersten echten Abschluss und ignoriert altes ``done``."""
    gestartet = False

    while True:
        _abbruch_pruefen(hub)
        jetzt = gesamt_uhr.time()
        fortschritt = abs(drive_base.angle() - startwinkel)
        fertig = drive_base.done()

        if fortschritt >= MIN_FORTSCHRITT_DEG or not fertig:
            gestartet = True
        if gestartet and fertig:
            return
        if jetzt >= MAX_DAUER_MS:
            drive_base.stop()
            raise RuntimeError(
                "Drehung nach {} ms nicht fertig (Winkel {:.3f})".format(
                    jetzt,
                    drive_base.angle(),
                )
            )
        # Sehr kurze Bewegungen koennen schon beim ersten Poll fertig sein.
        if jetzt >= STARTSCHUTZ_MS and fertig and fortschritt > 0:
            return
        wait(REGEL_INTERVALL_MS)


def _fahr_drehung(hub, drive_base, ziel, gesamt_uhr):
    startwinkel = drive_base.angle()
    drive_base.turn(ziel, then=Stop.BRAKE, wait=False, absolute=True)
    _warte_fahrbefehl(hub, drive_base, startwinkel, gesamt_uhr)


def schnelle_drehung(hub, drive_base, ziel, rate, beschleunigung):
    """Dreht absolut und gibt Laufzeit, Endwinkel und Korrekturzahl zurueck."""
    einstellungen = drive_base.settings()
    drive_base.settings(
        einstellungen[0],
        einstellungen[1],
        rate,
        beschleunigung,
    )

    gesamt_uhr = StopWatch()
    _fahr_drehung(hub, drive_base, ziel, gesamt_uhr)
    korrekturen = 0

    if abs(ziel - drive_base.angle()) > KORREKTUR_GRENZE_DEG:
        drive_base.settings(
            einstellungen[0],
            einstellungen[1],
            KORREKTUR_RATE,
            KORREKTUR_BESCHLEUNIGUNG,
        )
        _fahr_drehung(hub, drive_base, ziel, gesamt_uhr)
        korrekturen = 1

    drive_base.stop()
    dauer = gesamt_uhr.time()
    endwinkel = drive_base.angle()
    drive_base.settings(*einstellungen)
    return dauer, endwinkel, korrekturen


def kalibrieren(hub, drive_base, linker_motor, rechter_motor):
    drive_base.use_gyro(True)
    try:
        geschwindigkeitstoleranz, _ = drive_base.heading_control.target_tolerances()
        drive_base.heading_control.target_tolerances(
            geschwindigkeitstoleranz,
            ZIEL_TOLERANZ_DEG,
        )
    except (AttributeError, OSError, TypeError) as error:
        print("CAL_TOLERANCE_UNAVAILABLE", str(error))

    print("CAL_BEGIN")
    print(
        "CAL_FIELDS,rate,beschleunigung,wiederholung,ziel_deg,"
        "endwinkel_deg,fehler_deg,dauer_ms,korrekturen,bestanden"
    )

    gesamt_bestanden = 0
    gesamt = 0
    statistik = {}

    for rate, beschleunigung in TEST_PROFILE:
        schluessel = (rate, beschleunigung)
        # bestanden, gesamt, dauer_summe, max_fehler, max_dauer, korrekturen
        statistik[schluessel] = [0, 0, 0, 0, 0, 0]

        for wiederholung in range(1, WIEDERHOLUNGEN + 1):
            for ziel in TEST_WINKEL:
                _abbruch_pruefen(hub)
                drive_base.reset(distance=drive_base.distance(), angle=0)
                wait(MESS_PAUSE_MS)

                try:
                    dauer, endwinkel, korrekturen = schnelle_drehung(
                        hub, drive_base, ziel, rate, beschleunigung
                    )
                except RuntimeError as error:
                    dauer = MAX_DAUER_MS
                    endwinkel = drive_base.angle()
                    korrekturen = -1
                    print("CAL_TIMEOUT", str(error))

                fehler = ziel - endwinkel
                ist_bestanden = abs(fehler) < 2 and dauer < MAX_DAUER_MS
                gesamt += 1
                gesamt_bestanden += 1 if ist_bestanden else 0

                profil = statistik[schluessel]
                profil[0] += 1 if ist_bestanden else 0
                profil[1] += 1
                profil[2] += dauer
                profil[3] = max(profil[3], abs(fehler))
                profil[4] = max(profil[4], dauer)
                profil[5] += max(0, korrekturen)

                print(
                    "CAL_RESULT,{},{},{},{},{:.3f},{:.3f},{},{},{}".format(
                        rate,
                        beschleunigung,
                        wiederholung,
                        ziel,
                        endwinkel,
                        fehler,
                        dauer,
                        korrekturen,
                        1 if ist_bestanden else 0,
                    )
                )
                wait(MESS_PAUSE_MS)

    bestes_profil = None
    beste_dauer = None
    for rate, beschleunigung in TEST_PROFILE:
        profil = statistik[(rate, beschleunigung)]
        mittlere_dauer = profil[2] / profil[1]
        print(
            "CAL_PROFILE,{},{},{},{},{:.1f},{:.3f},{},{}".format(
                rate,
                beschleunigung,
                profil[0],
                profil[1],
                mittlere_dauer,
                profil[3],
                profil[4],
                profil[5],
            )
        )
        if profil[0] == profil[1] and (
            beste_dauer is None or mittlere_dauer < beste_dauer
        ):
            bestes_profil = (rate, beschleunigung)
            beste_dauer = mittlere_dauer

    _antrieb_bremsen(drive_base, linker_motor, rechter_motor)
    print(
        "CAL_SUMMARY,{},{},{}".format(
            gesamt_bestanden, gesamt, gesamt - gesamt_bestanden
        )
    )
    print("CAL_BEST,{},{}".format(bestes_profil, beste_dauer))
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
