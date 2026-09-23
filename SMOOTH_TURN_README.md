# Ein-Pass-Drehung kalibrieren

`robot.turn(...)` und `robot.turn_to(...)` verwenden mit `precise=True` eine
kontinuierliche Drehung. Die Drehrate sinkt vor dem Ziel; der Roboter bremst
einmal und fährt keine 5°-Nachkorrektur. `precise=False` behält den bisherigen
Pybricks-Drehpfad bei. Geraden, Multitask und Anbaumotoren bleiben unverändert.

## Kalibrierung auf dem Hub

1. `robot.py`, `robot_config.py` und `calibration.py` auf den Hub laden.
2. Den Roboter mit freiem Platz auf eine ebene Wettkampfmatte stellen und
   `calibration.py` starten. Das Programm dreht jeweils ±33°, ±47° und ±180°;
   jede Drehung wird zweimal mit drei Bremsverzögerungen wiederholt.
3. Die Zeilen `CAL_RESULT`, `CAL_SUMMARY` und `CAL_BEST` aus der Konsole
   auswerten. Ein Versuch gilt bei einem Gyrofehler bis 1° als bestanden.
   `CAL_BEST` nennt die genaueste Bremsverzögerung; diesen Wert anschließend
   als `SMOOTH_TURN_BRAKE_DECEL` in `robot_config.py` übernehmen.
4. Zum Abschluss `smooth_turn_bench.py` ausführen. Es wiederholt dreimal die
   Drehung zwischen 0° und 180° und gibt zusätzlich `SMOOTH_TURN_DONE` aus.
   Die echte Ausrichtung mit einer Markierung auf der Matte vergleichen: Der
   Gyro misst keine absolute Richtung und erkennt keinen Radschlupf.

Der Kalibrierlauf vergleicht nur die passive Bremsverzögerung, damit jeweils
eine Variable verändert wird. Höchstgeschwindigkeit, Mindestgeschwindigkeit
und das Annäherungsprofil bleiben bei den Startwerten. Wenn kein Profil alle Versuche besteht, zuerst
Akku, Reifen, Matte und Radschlupf prüfen und den Kalibrierlauf wiederholen.

## Parameter in `robot_config.py`

- `SMOOTH_TURN_MAX_RATE`: Höchsttempo auf langen Drehungen, Startwert 160°/s.
- `SMOOTH_TURN_MIN_RATE`: Langsamste Dauerdrehung, Startwert 28°/s.
- `SMOOTH_TURN_DECEL`: Abbremsung vor dem Ziel, Startwert 450°/s².
- `SMOOTH_TURN_STOP_DELAY_MS`: Reaktionszeit bis zur Bremse, Startwert 20 ms.
- `SMOOTH_TURN_STOP_OFFSET_DEG`: Fester Bremsvorhalt, Startwert 0,35°.
  Bei zu kurzen Drehungen den Wert schrittweise senken; bei Überschießen erhöhen.
- `SMOOTH_TURN_BRAKE_DECEL`: Geschätzte passive Bremsverzögerung. Aus sechs
  180°-Fahrten des echten Hubs ergab sich etwa 1.800°/s². Ein kleinerer Wert
  lässt früher bremsen, ein größerer Wert später.
- `SMOOTH_TURN_ACCEPT_DEG`: Schwelle für die Genauigkeitswarnung nach dem Halt.
- `SMOOTH_TURN_DEBUG`: Schaltet die regelmäßigen Statuszeilen ein oder aus.

Die Simulationstests auf dem Rechner startest du mit
`python -m unittest -q test_robot.py`. Sie prüfen mehrere Winkel und simulierte
Bremsstärken, ersetzen aber keine Messung auf der Matte.

`SMOOTH_TURN_DONE` enthält nun zusätzlich den Winkel beim Bremsbefehl,
den tatsächlichen Auslauf und den berechneten Bremsvorhalt. Diese Werte
erlauben eine Nachkalibrierung aus echten Fahrten.

Im ersten Hub-Lauf lag die Drehrate beim Bremsen zwischen 82 und 102°/s;
alle sechs Drehungen schossen um 1,87 bis 3,13° über. Der zusätzliche Weg
passt zu `Drehrate² / (2 × 1815°/s²)` mit höchstens 0,27° Restabweichung
für diese sechs Messwerte. 1.800°/s² ist deshalb ein Startwert für den
nächsten Hub-Lauf, noch kein bestätigtes Endergebnis. Der Benchmark lässt
die regelmäßigen Statusmeldungen jetzt aus und gibt weiterhin jede
abgeschlossene Drehung aus.
