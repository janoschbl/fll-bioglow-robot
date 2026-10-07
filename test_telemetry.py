"""Prüft den Telemetrieschalter ohne angeschlossenen Hub."""

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
from robot import Robot


class TelemetrieTest(unittest.TestCase):
    def test_deaktiviert_ohne_transport_sensorzugriff_oder_ausgabe(self):
        hub, drive, motors = Mock(), Mock(), [Mock() for _ in range(4)]
        ausgabe = io.StringIO()
        with patch.object(config, "TELEMETRY_ENABLED", False), \
                patch.object(telemetry, "AppData", Mock()) as transport, \
                contextlib.redirect_stdout(ausgabe):
            daten = telemetry.Telemetry(hub, drive, motors)
            daten.begin("Testlauf")
            daten.tick(force=True)
            daten.event("straight", 0, 100)
            daten.finish_and_send("success")
            daten._send(b"Test")
            config.protokolliere("Testausgabe")
        transport.assert_not_called()
        self.assertEqual(hub.mock_calls, [])
        self.assertEqual(drive.mock_calls, [])
        self.assertTrue(all(not motor.mock_calls for motor in motors))
        self.assertFalse(daten.active)
        self.assertEqual(daten.data, bytearray())
        self.assertEqual(ausgabe.getvalue(), "")

    def test_aktiviert_sendet_appdata_und_status(self):
        ausgabe = io.StringIO()
        with patch.object(config, "TELEMETRY_ENABLED", True), \
                patch.object(telemetry, "AppData", Mock()) as transport, \
                contextlib.redirect_stdout(ausgabe):
            daten = telemetry.Telemetry(Mock(), Mock(), [])
            # Sensoren sind hier unabhängig vom Transport getestet.
            with patch.object(daten, "tick"):
                daten.begin("Testlauf")
                daten.event("straight", 0, 100)
                daten.finish_and_send("success")
        pakete = transport.return_value.write_bytes.call_args_list
        self.assertEqual([aufruf.args[0][4] for aufruf in pakete], [0, 1, 2])
        self.assertIn("TELEMETRY_SENT", ausgabe.getvalue())
        self.assertFalse(daten.active)

    def test_deaktivieren_verhindert_senden_eines_bestehenden_laufs(self):
        with patch.object(config, "TELEMETRY_ENABLED", True), \
                patch.object(telemetry, "AppData", Mock()) as transport:
            daten = telemetry.Telemetry(Mock(), Mock(), [])
            with patch.object(daten, "tick"):
                daten.begin("Testlauf")
        with patch.object(config, "TELEMETRY_ENABLED", False):
            daten.finish_and_send("success")
            daten._send(b"Test")
        transport.return_value.write_bytes.assert_not_called()

    def test_menue_geht_ohne_telemetrie_zum_naechsten_programm(self):
        hub = Mock()
        hub.buttons.pressed.return_value = []
        roboter = Robot(hub, Mock(), Mock(), Mock(), Mock(), Mock())
        menue = Menu(hub, roboter)
        erstes, zweites = Mock(), Mock()
        menue.program(index=1, name="Erstes")(erstes)
        menue.program(index=2, name="Zweites")(zweites)
        ausgabe = io.StringIO()
        with patch.object(config, "TELEMETRY_ENABLED", False), \
                contextlib.redirect_stdout(ausgabe):
            menue.run_selected_program(menue.program_slots[0])
        erstes.assert_called_once_with()
        zweites.assert_not_called()
        self.assertEqual(menue.next_program_index(0), 1)
        self.assertIsNone(menue.running_program)
        self.assertEqual(ausgabe.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
