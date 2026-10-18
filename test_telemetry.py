"""Prüft Aufnahme und Senden der Telemetrie ohne angeschlossenen Hub."""

import contextlib
import io
import struct
import sys
import types
import unittest
from unittest.mock import Mock, patch


parameters = types.ModuleType("pybricks.parameters")
parameters.Direction = types.SimpleNamespace(COUNTERCLOCKWISE=0, CLOCKWISE=1)
parameters.Port = types.SimpleNamespace(A=0, B=1, C=2, D=3, F=5)
parameters.Button = types.SimpleNamespace(CENTER=0)
parameters.Stop = types.SimpleNamespace(HOLD=0, BRAKE=1, COAST=2)
tools = types.ModuleType("pybricks.tools")
tools.StopWatch = lambda: types.SimpleNamespace(time=lambda: 0)
tools.wait = Mock()
sys.modules["pybricks"] = types.ModuleType("pybricks")
sys.modules["pybricks.parameters"] = parameters
sys.modules["pybricks.tools"] = tools
# Lokale MicroPython-Typstubs führen pack() nicht aus.
sys.modules["ustruct"] = struct

import robot_config as config
import telemetry
from menu import Menu
from robot import ProgramAborted, Robot


class Uhr:
    def __init__(self):
        self.ms = 0

    def time(self):
        return self.ms


def sensoren():
    hub = Mock()
    hub.imu.acceleration.return_value = (0, 0, 9800)
    hub.imu.angular_velocity.return_value = (0, 0, 0)
    hub.imu.orientation.return_value = {(r, c): 1 if r == c else 0 for r in range(3) for c in range(3)}
    hub.imu.heading.return_value = 0
    hub.buttons.pressed.return_value = []
    drive = Mock()
    drive.state.return_value = (0, 0, 0, 0)
    motor = Mock()
    motor.angle.return_value = 0
    motor.speed.return_value = 0
    motor.load.return_value = 0
    motor.stalled.return_value = False
    motor.done.return_value = True
    return hub, drive, [motor] * 4


def datensaetze(daten):
    """Zerlegt den Puffer wie die Code-Werkstatt in Messpunkte und Befehle."""
    samples, befehle, position = [], [], 0
    while position < daten.length:
        art = daten.buffer[position]
        if art == 1:
            samples.append(struct.unpack_from("<I", daten.buffer, position + 1)[0])
            position += telemetry.SAMPLE_SIZE
        else:
            befehle.append(struct.unpack_from(telemetry.EVENT_FORMAT, daten.buffer, position)[2:4])
            position += telemetry.EVENT_SIZE
    return samples, befehle


class TelemetrieTest(unittest.TestCase):
    def test_ohne_start_kein_sensorzugriff_und_kein_senden(self):
        hub, drive, motors = Mock(), Mock(), [Mock() for _ in range(4)]
        with patch.object(telemetry, "AppData", Mock()) as transport:
            daten = telemetry.Telemetry(hub, drive, motors)
            daten.tick(force=True)
            daten.event("straight", 0, 100)
            daten.finish_and_send("success")
        transport.assert_not_called()
        self.assertEqual(hub.mock_calls, [])
        self.assertEqual(drive.mock_calls, [])
        self.assertIsNone(daten.buffer)

    def test_sendet_appdata_mit_laufzeit_ohne_uebertragung(self):
        uhr = Uhr()
        with patch.object(telemetry, "AppData", Mock()) as transport:
            daten = telemetry.Telemetry(*sensoren())
            daten.clock = uhr
            daten.begin("Testlauf")
            uhr.ms = 1500
            daten.event("straight", 0, 100)
            transport.return_value.write_bytes.side_effect = lambda paket: setattr(uhr, "ms", uhr.ms + 60)
            daten.finish_and_send("success")
        pakete = [aufruf.args[0] for aufruf in transport.return_value.write_bytes.call_args_list]
        self.assertEqual([paket[4] for paket in pakete], [0, 1, 1, 2])
        _, dauer, samples, befehle, verworfen, ergebnis = struct.unpack(telemetry.END_FORMAT, pakete[-1][15:])
        self.assertEqual((dauer, samples, befehle, verworfen, ergebnis), (1500, 2, 1, 0, 0))
        self.assertFalse(daten.active)

    def test_voller_puffer_wird_ausgeduennt_statt_abgeschnitten(self):
        uhr = Uhr()
        with patch.object(config, "TELEMETRY_BUFFER_BYTES", 8000):
            daten = telemetry.Telemetry(*sensoren())
            daten.clock = uhr
            daten.begin("Lang")
            for schritt in range(1, 400):
                uhr.ms = schritt * 20
                daten.tick()
                if schritt % 10 == 0:
                    daten.event("turn", 0, schritt)
        samples, befehle = datensaetze(daten)
        self.assertEqual(len(befehle), 39)
        self.assertEqual(samples[0], 0)
        self.assertGreater(samples[-1], 7000)
        self.assertEqual(len(samples), daten.samples)
        self.assertGreater(daten.sample_ms, 20)
        self.assertLessEqual(daten.length, 8000)


class RoboterTelemetrieTest(unittest.TestCase):
    def roboter_mit_menue(self):
        hub, drive, motors = sensoren()
        roboter = Robot(hub, drive, Mock(), Mock(), Mock(), Mock())
        daten = telemetry.Telemetry(hub, drive, motors)
        daten.app_data = Mock()
        roboter.set_telemetry(daten)
        return roboter, Menu(hub, roboter), daten, daten.app_data.write_bytes

    def test_aufnahme_ueber_zwei_programme(self):
        roboter, menue, daten, senden = self.roboter_mit_menue()
        menue.program(index=1, name="Erstes")(lambda: roboter.start_telemetry())
        menue.program(index=2, name="Zweites")(lambda: roboter.send_telemetry())
        with contextlib.redirect_stdout(io.StringIO()):
            menue.run_selected_program(menue.program_slots[0])
            self.assertTrue(roboter.telemetry_active())
            senden.assert_not_called()
            menue.run_selected_program(menue.program_slots[1])
        self.assertFalse(roboter.telemetry_active())
        self.assertEqual(senden.call_args_list[0].args[0][15 + 10:], b"Erstes")
        programme = [befehl for befehl in datensaetze(daten)[1] if befehl[0] == telemetry.EVENT_IDS["program"]]
        self.assertEqual(programme, [(13, 0), (13, 1), (13, 0), (13, 1)])

    def test_abbruch_sendet_laufende_aufnahme(self):
        roboter, menue, daten, senden = self.roboter_mit_menue()

        def abbrechen():
            roboter.start_telemetry("Runde")
            raise ProgramAborted()

        menue.program(index=1, name="Erstes")(abbrechen)
        with contextlib.redirect_stdout(io.StringIO()):
            menue.run_selected_program(menue.program_slots[0])
        self.assertFalse(roboter.telemetry_active())
        self.assertEqual(senden.call_args_list[-1].args[0][-1], telemetry.OUTCOMES["aborted"])

    def test_menue_geht_ohne_telemetrie_zum_naechsten_programm(self):
        hub = Mock()
        hub.buttons.pressed.return_value = []
        roboter = Robot(hub, Mock(), Mock(), Mock(), Mock(), Mock())
        menue = Menu(hub, roboter)
        erstes, zweites = Mock(), Mock()
        menue.program(index=1, name="Erstes")(erstes)
        menue.program(index=2, name="Zweites")(zweites)
        ausgabe = io.StringIO()
        with patch.object(config, "CONSOLE_OUTPUT", False), \
                contextlib.redirect_stdout(ausgabe):
            menue.run_selected_program(menue.program_slots[0])
        erstes.assert_called_once_with()
        zweites.assert_not_called()
        self.assertEqual(menue.next_program_index(0), 1)
        self.assertIsNone(menue.running_program)
        self.assertEqual(ausgabe.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
