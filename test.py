"""Testlauf fuer eine eigene, praezise Geradeausfahrt (drive_mm), Version 2.

Laeuft komplett unabhaengig von main.py / robot.py. Liest nur robot_config.
Gyro haelt die Richtung, beide Radencoder messen die Strecke, ein
Geschwindigkeitsprofil bremst vorausschauend. Neu in v2:
- Endanflug regelt Strecke UND Richtung gemeinsam
- Seitenversatz wird aus Encoderweg + Gyro geschaetzt und zurueckgelenkt
- MODE "tune": automatische Parametersuche, danach A/B-Vergleich
- MODE "stress": Geschwindigkeiten, Wiederholgenauigkeit, 700 mm, Kette

SICHERHEIT: Jede Vorwaertsfahrt wird genauso weit zurueckgefahren. Der
Roboter entfernt sich nie mehr als MAX_FORWARD_MM vom Startpunkt.
"""

from math import sin, cos, sqrt, pi
from pybricks.hubs import PrimeHub
from pybricks.parameters import Stop
from pybricks.pupdevices import Motor
from pybricks.tools import StopWatch, wait

import robot_config as config

MODE = "smoke"          # smoke | test | tune | stress

WHEEL_DIAMETER_MM = config.WHEEL_DIAMETER_MM
AXLE_TRACK_MM = config.AXLE_TRACK_MM
DIST_SCALE = 1.0        # reale mm / Encoder-mm
MAX_FORWARD_MM = 700    # harte Grenze nach vorne (Platz ca. 800 mm)
MAX_BACK_MM = 15
START_OFFSET_MM = 0
WALL_GAP_MM = 3.0       # Roboter steht hinten an der Wand: Heimposition 3 mm davor
POS_TOL_DEG = 1
HEAD_ABORT_DEG = 12
HEAD_JUMP_DEG = 5       # nach dem Anhalten groesser = Roboter wurde gedreht -> Stopp
HEAD_FINAL_MAX = 2.0    # max. Richtungskorrektur im Endanflug (Grad)
LOOP_MS = 5
LOG_MS = 100
TIMEOUT_MS = 8000
SETTLE_TIMEOUT_MS = 300
SETTLE_SPEED = 15       # deg/s, darunter gilt ein Motor als stehend
DEG2RAD = 0.0174532925
MM_PER_DEG = 3.14159265 * WHEEL_DIAMETER_MM / 360 * DIST_SCALE

DEFAULT = {
    "v_max": 450, "v_fast": 450, "accel": 700, "decel": 500,
    "v_min": 25, "k_app": 6.0, "final_zone": 1.5, "final_speed": 120,
    "tries": 2, "retry_tol": 1.0, "retry_head": 0.6,
    "kp": 8.0, "kd": 0.4, "corr_max": 80,
    "lat_k": 0.0, "lat_max": 2.0, "lat_fade": 60,
    "ff": 0.0, "ff_len": 40,
}

# Suchraum fuer "tune": je Parameter die Kandidatenwerte
SPACE = (
    ("decel", (500, 700, 900)),
    ("v_min", (20, 25, 40)),
    ("kp", (4.0, 6.0, 8.0)),
    ("final_zone", (1.5, 2.0, 3.0)),
    ("lat_k", (0.0, 0.6, 1.2)),
    ("retry_head", (0.3, 0.6, 99)),
)
# straight_pid: innen Rad-Positionsregelung (Encoder), aussen Gyro-Korrektur
PID = {
    "v": 480,          # mm/s Reisegeschwindigkeit
    "acc": 1100,       # mm/s^2 Spitzenbeschleunigung (Sinus-Rampe)
    "dec": 1100,       # mm/s^2 Spitzenverzoegerung
    "kw": 25.0,        # 1/s: Positionsfehler je Rad -> Zusatzgeschwindigkeit
    "alpha": 0.1,      # Filter fuer Gyro-minus-Encoder-Drift (je Schleife)
    "tol": 0.5,        # mm Endtoleranz je Rad
    "settle_ms": 250,  # max. Nachregelzeit nach Profilende
    "mode": "dF",      # "casc": Gyro-PD gibt Drehrate vor, Encoder setzen sie um
    "kdiff": 25.0,     # 1/s: Gleichlauf-Regelung der Raddifferenz
    "kh": 20.0,        # 1/s: Heading-Fehler -> Drehrate
    "kr": 0.1,         # Daempfung auf gemessene Drehrate
    "wmax": 60.0,      # deg/s max. Korrektur-Drehrate
    "finish": "settle",  # Endanflug wie drive_mm (run_target, Strecke + Richtung)
    "retry_head": 0.25,  # Endanflug: Richtung bis auf 0,25 Grad
    "tries": 3,          # max. Versuche im Endanflug
}

TUNE_DISTS = (300, 100, 300)
TUNE_MARGIN = 0.15

# MODE "speedrun": gleiche Strecke bei verschiedenen Geschwindigkeiten
SPEEDS = (150, 250, 350, 450, 500)
RUNS = 60
BENCH_RUNS = 60
RUN_DIST = 500
BIN_MM = 10             # Heading-Profil: ein Wert pro 10 mm Fahrstrecke
WALL_DUTY = 20          # sanftes Andruecken an die Wand (Prozent)
WALL_PUSH_MS = 500
WALL_SEEK_DUTY = 25      # rueckwaerts suchen, bis die Wand erreicht ist
WALL_SEEK_MAX_MM = 750

# Index im Ergebnis-Tupel
R_LABEL, R_DIST, R_ERR, R_HEAD, R_MAXHD, R_OVER, R_TDRIVE, R_TSETTLE, R_LAT, R_LOOP, R_RMS, R_MAXX = range(12)


def rnd(x, n=2):
    return round(float(x), n)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Driver:
    def __init__(self, p):
        self.p = p
        self.verbose = True
        self.hub = PrimeHub()
        self.L = Motor(config.LEFT_DRIVE_PORT, config.LEFT_DRIVE_DIRECTION)
        self.R = Motor(config.RIGHT_DRIVE_PORT, config.RIGHT_DRIVE_DIRECTION)
        for m in (self.L, self.R):
            st, pt = m.control.target_tolerances()
            m.control.target_tolerances(st, POS_TOL_DEG)
        while not self.hub.imu.ready():
            wait(50)
        self.hub.imu.reset_heading(0)
        self.L.reset_angle(0)
        self.R.reset_angle(0)
        self.origin = self.enc_mm() - START_OFFSET_MM + WALL_GAP_MM
        self.nominal = self.enc_mm()
        self.target_heading = 0
        self.v = 0
        self.x = 0.0            # geschaetzter Seitenversatz in mm (rechts +)
        self.results = []
        self.q = dict(PID)
        self.trace2 = None
        self.trace = None
        self.tr_n = 0

    def enc_mm(self):
        return (self.L.angle() + self.R.angle()) * 0.5 * MM_PER_DEG

    def pos(self):
        return self.enc_mm() - self.origin

    def stop(self):
        self.L.hold()
        self.R.hold()
        self.v = 0

    def stopped(self):
        return abs(self.L.speed()) < SETTLE_SPEED and abs(self.R.speed()) < SETTLE_SPEED

    def settle(self, target, th):
        """Endanflug: Strecke und Richtung gemeinsam auf das Ziel bringen."""
        p = self.p
        n = 0
        t = StopWatch()
        for n in range(1, p["tries"] + 1):
            herr = th - self.hub.imu.heading()
            delta = (target - self.enc_mm()) / MM_PER_DEG
            s = clamp(herr, -HEAD_FINAL_MAX, HEAD_FINAL_MAX) * DEG2RAD * AXLE_TRACK_MM / 2 / MM_PER_DEG
            self.L.run_target(p["final_speed"], self.L.angle() + delta + s, Stop.HOLD, False)
            self.R.run_target(p["final_speed"], self.R.angle() + delta - s, Stop.HOLD, False)
            t.reset()
            while t.time() < SETTLE_TIMEOUT_MS:
                if t.time() >= 60 and self.stopped():
                    break
                wait(LOOP_MS)
            e = self.enc_mm() - target
            he = th - self.hub.imu.heading()
            if self.verbose:
                print("SETTLE", n, "err", rnd(e), "herr", rnd(he), "ms", t.time())
            if abs(e) <= p["retry_tol"] and abs(he) <= p["retry_head"]:
                break
        self.v = 0
        return n

    def straight_pid(self, dist, v_max=None, label=""):
        """Geradeaus: beide Raeder folgen einem gemeinsamen Weg-Zeit-Profil
        (Positionsregelung je Rad), die Richtung kommt als Raddifferenz dazu.
        Der Gyro korrigiert nur langsam, was die Encoder nicht sehen (Schlupf)."""
        q = self.q
        target = self.nominal + dist
        rel = target - self.origin
        if rel > MAX_FORWARD_MM or rel < -MAX_BACK_MM:
            print("SAFETY_REFUSE", label, "target_rel", rel)
            return None
        D = abs(dist)
        V = v_max or q["v"]
        Aa = q["acc"]
        Ad = q["dec"]
        kk = pi / 4 * (1 / Aa + 1 / Ad)
        if kk * V * V > D:
            V = sqrt(D / kk)
        Ta = pi * V / (2 * Aa)
        Td = pi * V / (2 * Ad)
        sa = V * Ta / 2
        Tc = max(0, (D - sa - V * Td / 2) / V) if V > 0 else 0
        T = Ta + Tc + Td
        th = self.target_heading
        Lm = self.L.angle() * MM_PER_DEG
        Rm = self.R.angle() * MM_PER_DEG
        c0 = (Lm + Rm) / 2
        span = target - c0
        sg = 1 if span >= 0 else -1
        h = self.hub.imu.heading()
        dF = h - (Lm - Rm) / AXLE_TRACK_MM * 57.2957795
        kw = q["kw"]
        al = q["alpha"]
        half = AXLE_TRACK_MM / 2 * DEG2RAD
        casc = q.get("mode", "dF") == "casc"
        kd2 = q.get("kdiff", kw)
        th_enc = (Lm - Rm) / AXLE_TRACK_MM * 57.2957795
        t_last = 0
        h_last = h
        rate_f = 0.0
        timer = StopWatch()
        loops = 0
        ss = 0.0
        max_hd = 0.0
        max_te = 0.0
        max_de = 0.0
        overshoot = 0.0
        still = 0
        t_prev = 0
        x = 0.0
        max_x = 0.0
        c_prev = c0
        tT = T * 1000
        while True:
            tm = timer.time()
            t = tm / 1000
            if t < Ta:
                s = V / 2 * (t - Ta / pi * sin(pi * t / Ta))
                v = V / 2 * (1 - cos(pi * t / Ta))
            elif t < Ta + Tc:
                s = sa + V * (t - Ta)
                v = V
            elif t < T:
                u = t - Ta - Tc
                s = sa + V * Tc + V / 2 * (u + Td / pi * sin(pi * u / Td))
                v = V / 2 * (1 + cos(pi * u / Td))
            else:
                s = D
                v = 0
            Lm = self.L.angle() * MM_PER_DEG
            Rm = self.R.angle() * MM_PER_DEG
            h = self.hub.imu.heading()
            eh = (Lm - Rm) / AXLE_TRACK_MM * 57.2957795
            dF += al * ((h - eh) - dF)
            dt = max(1, tm - t_last) / 1000
            t_last = tm
            rate_f = 0.7 * rate_f + 0.3 * (h - h_last) / dt
            h_last = h
            if casc:
                w = clamp(q["kh"] * (th - h) - q["kr"] * rate_f, -q["wmax"], q["wmax"])
                th_enc += w * dt
                vdiff = w * DEG2RAD * AXLE_TRACK_MM / 2
            else:
                if tm >= tT and q.get("endfix", 1):
                    dF = h - eh
                th_enc = th - dF
                vdiff = 0
            delta = th_enc * half
            if D > 0:
                c = c0 + span * s / D
                vc = v * span / D
            else:
                c = c0
                vc = 0
            eL = c + delta - Lm
            eR = c - delta - Rm
            em = c - (Lm + Rm) / 2
            ed = delta - (Lm - Rm) / 2
            self.L.run((vc + vdiff + kw * em + kd2 * ed) / MM_PER_DEG)
            self.R.run((vc - vdiff + kw * em - kd2 * ed) / MM_PER_DEG)
            cm = (Lm + Rm) / 2
            x += (cm - c_prev) * sin(h * DEG2RAD)
            c_prev = cm
            loops += 1
            herr = th - h
            ss += herr * herr
            if abs(herr) > max_hd:
                max_hd = abs(herr)
            if abs(x) > max_x:
                max_x = abs(x)
            te = max(abs(eL), abs(eR))
            if te > max_te:
                max_te = te
            if (cm - target) * sg > overshoot:
                overshoot = (cm - target) * sg
            tr = self.trace
            if tr is not None:
                kb = int(abs(cm - c0) / BIN_MM)
                while self.tr_n <= kb and self.tr_n < len(tr):
                    tr[self.tr_n] = int(round(h * 10))
                    self.tr_n += 1
            de = abs((Lm - Rm) / 2 - delta)
            if de > max_de:
                max_de = de
            t2 = self.trace2
            if t2 is not None:
                kb2 = int(abs(cm - c0) / BIN_MM)
                if kb2 < len(t2):
                    t2[kb2] = int(round(dF * 10))
            rel_now = cm - self.origin
            if (abs(herr) > HEAD_ABORT_DEG or tm > TIMEOUT_MS or te > 40
                    or rel_now > MAX_FORWARD_MM + 20 or rel_now < -MAX_BACK_MM - 20):
                self.stop()
                raise RuntimeError("ABORT %s herr %s t %s te %s rel %s" % (
                    label, rnd(herr), tm, rnd(te), rnd(rel_now)))
            if tm >= tT:
                if q.get("finish", "own") == "settle":
                    break
                if (abs(eL) < q["tol"] and abs(eR) < q["tol"] and abs(herr) < 0.15
                        and abs(self.L.speed()) < SETTLE_SPEED
                        and abs(self.R.speed()) < SETTLE_SPEED):
                    still += tm - t_prev
                else:
                    still = 0
                if still >= 30 or tm > tT + q["settle_ms"]:
                    break
            t_prev = tm
            wait(LOOP_MS)
        if q.get("finish", "own") == "settle":
            sp = self.p
            self.p = dict(sp)
            for k in ("tries", "retry_tol", "retry_head", "final_speed"):
                if k in q:
                    self.p[k] = q[k]
            self.settle(target, th)
            self.p = sp
        else:
            self.L.hold()
            self.R.hold()
        self.v = 0
        self.nominal = target
        h = self.hub.imu.heading()
        cm = (self.L.angle() + self.R.angle()) * 0.5 * MM_PER_DEG
        tr = self.trace
        if tr is not None:
            while self.tr_n < len(tr):
                tr[self.tr_n] = int(round(h * 10))
                self.tr_n += 1
        err = (cm - target) * sg
        t_run = int(tT)
        t_settle = timer.time() - t_run
        loop_ms = timer.time() / max(1, loops)
        res = (label, dist, rnd(err), rnd(th - h), rnd(max_hd), rnd(overshoot),
               t_run, t_settle, rnd(x), rnd(loop_ms),
               rnd(sqrt(ss / max(1, loops))), rnd(max_x))
        print("RESULT", label, "dist", dist, "err_mm", res[R_ERR],
              "head_err", res[R_HEAD], "max_hd", res[R_MAXHD], "over", res[R_OVER],
              "t_drive", t_run, "t_settle", t_settle, "lat_mm", res[R_LAT],
              "loop_ms", res[R_LOOP], "rms", res[R_RMS], "max_x", res[R_MAXX],
              "max_te", rnd(max_te), "max_de", rnd(max_de), "drift", rnd(dF, 2), "V", rnd(V))
        self.results.append(res)
        return res

    def drive_mm(self, dist, precision=True, v_max=None, label=""):
        p = self.p
        target = self.nominal + dist
        rel = target - self.origin
        if rel > MAX_FORWARD_MM or rel < -MAX_BACK_MM:
            print("SAFETY_REFUSE", label, "target_rel", rel)
            return None
        d = 1 if dist >= 0 else -1
        vmax = v_max or (p["v_max"] if precision else p["v_fast"])
        th = self.target_heading
        timer = StopWatch()
        t_prev = 0
        h_prev = self.hub.imu.heading()
        pos_prev = self.enc_mm()
        rate_f = 0
        next_log = 0
        max_hd = 0
        overshoot = 0
        loops = 0
        ss = 0.0
        max_x = 0.0
        pos0 = pos_prev
        if self.verbose:
            print("SEG_START", label, "dist", dist, "prec", int(precision),
                  "pos", rnd(self.pos()), "target_rel", rnd(rel), "vmax", vmax,
                  "x", rnd(self.x))
        while True:
            t = timer.time()
            pos = self.enc_mm()
            h = self.hub.imu.heading()
            self.x += (pos - pos_prev) * sin(h * DEG2RAD)
            pos_prev = pos
            rem = (target - pos) * d
            dt = max(1, t - t_prev) / 1000
            rate_f = 0.7 * rate_f + 0.3 * ((h - h_prev) / dt)
            t_prev = t
            h_prev = h
            loops += 1
            if abs(th - h) > max_hd:
                max_hd = abs(th - h)
            ss += (th - h) * (th - h)
            if abs(self.x) > max_x:
                max_x = abs(self.x)
            tr = self.trace
            if tr is not None:
                k = int(abs(pos - pos0) / BIN_MM)
                while self.tr_n <= k and self.tr_n < len(tr):
                    tr[self.tr_n] = int(round(h * 10))
                    self.tr_n += 1
            if -rem > overshoot:
                overshoot = -rem

            rel_now = pos - self.origin
            if (abs(th - h) > HEAD_ABORT_DEG or t > TIMEOUT_MS
                    or rel_now > MAX_FORWARD_MM + 20 or rel_now < -MAX_BACK_MM - 20):
                self.stop()
                raise RuntimeError("ABORT %s herr %s t %s rel %s" % (
                    label, rnd(th - h), t, rnd(rel_now)))

            if precision:
                if rem <= p["final_zone"]:
                    break
                mag = min(vmax, (2 * p["decel"] * abs(rem)) ** 0.5,
                          max(p["v_min"], p["k_app"] * abs(rem)))
                v_t = d * mag
            else:
                if rem <= 0:
                    break
                v_t = d * vmax

            if abs(v_t) > abs(self.v) and v_t * self.v >= 0:
                step = p["accel"] * dt
                self.v = clamp(v_t, self.v - step, self.v + step)
            else:
                self.v = v_t

            # Zielrichtung: Basisrichtung + sanfte Rueckfuehrung auf die Linie
            f = clamp(rem / p["lat_fade"], 0, 1)
            th_eff = th - d * clamp(p["lat_k"] * self.x, -p["lat_max"], p["lat_max"]) * f
            ffc = 0.0
            if d > 0 and p["ff"]:
                ffc = p["ff"] * clamp(1 - (pos - pos0) / p["ff_len"], 0, 1)
            corr = clamp(p["kp"] * (th_eff - h) - p["kd"] * rate_f + ffc,
                         -p["corr_max"], p["corr_max"])
            self.L.run((self.v + corr) / MM_PER_DEG)
            self.R.run((self.v - corr) / MM_PER_DEG)

            if self.verbose and t >= next_log:
                print("S", t, rnd(rel_now, 1), rnd(rem, 1), rnd(self.v),
                      rnd(th_eff - h, 2), rnd(corr, 1), "x", rnd(self.x, 2))
                next_log = t + LOG_MS
            wait(LOOP_MS)

        t_run = timer.time()
        loop_ms = t_run / max(1, loops)
        self.nominal = target
        if not precision:
            print("SEG_HANDOVER", label, "t", t_run, "v", rnd(self.v),
                  "over_mm", rnd(-rem), "herr", rnd(th - h), "x", rnd(self.x),
                  "loop_ms", rnd(loop_ms))
            return None
        ts = StopWatch()
        tries = self.settle(target, th)
        t_settle = ts.time()
        h = self.hub.imu.heading()
        if abs(th - h) > HEAD_JUMP_DEG:
            self.stop()
            raise RuntimeError("HEAD_JUMP %s herr %s" % (label, rnd(th - h)))
        tr = self.trace
        if tr is not None:
            while self.tr_n < len(tr):
                tr[self.tr_n] = int(round(h * 10))
                self.tr_n += 1
        self.x += (self.enc_mm() - pos_prev) * sin(h * DEG2RAD)
        err = (self.enc_mm() - target) * d
        res = (label, dist, rnd(err), rnd(th - h), rnd(max_hd), rnd(overshoot),
               t_run, t_settle, rnd(self.x), rnd(loop_ms),
               rnd(sqrt(ss / max(1, loops))), rnd(max_x))
        print("RESULT", label, "dist", dist, "err_mm", res[R_ERR],
              "head_err", res[R_HEAD], "max_hd", res[R_MAXHD],
              "over", res[R_OVER], "t_drive", t_run, "t_settle", t_settle,
              "lat_mm", res[R_LAT], "loop_ms", res[R_LOOP], "rms", res[R_RMS],
              "max_x", res[R_MAXX], "tries", tries)
        self.results.append(res)
        return res


# ------------------------------------------------------------ Auswertung
def stats(rs, title):
    n = len(rs)
    if n == 0:
        print("STATS", title, "n 0")
        return
    se = [r[R_ERR] for r in rs]
    ae = [abs(v) for v in se]
    ah = [abs(r[R_HEAD]) for r in rs]
    m = sum(se) / n
    sd = sqrt(sum((v - m) * (v - m) for v in se) / n)
    print("STATS", title, "n", n, "max_err", rnd(max(ae)), "mean_err", rnd(sum(ae) / n),
          "bias", rnd(m), "sd", rnd(sd), "max_head", rnd(max(ah)),
          "mean_head", rnd(sum(ah) / n), "max_hd", rnd(max(r[R_MAXHD] for r in rs)),
          "max_lat", rnd(max(abs(r[R_LAT]) for r in rs)),
          "t_drive", rnd(sum(r[R_TDRIVE] for r in rs) / n, 0),
          "t_settle", rnd(sum(r[R_TSETTLE] for r in rs) / n, 0))


def score(rs):
    n = len(rs)
    max_e = max(abs(r[R_ERR]) for r in rs)
    mean_e = sum(abs(r[R_ERR]) for r in rs) / n
    max_h = max(abs(r[R_HEAD]) for r in rs)
    max_l = max(abs(r[R_LAT]) for r in rs)
    t_mean = sum(r[R_TDRIVE] + r[R_TSETTLE] for r in rs) / n / 1000.0
    return 2 * max_e + mean_e + max_h + 0.5 * max_l + 0.4 * t_mean


def pairs(dr, dists, tag, wait_ms=100):
    for i, dist in enumerate(dists):
        dr.drive_mm(dist, True, label="%sF%d_%d" % (tag, dist, i))
        wait(wait_ms)
        dr.drive_mm(-dist, True, label="%sB%d_%d" % (tag, dist, i))
        wait(wait_ms)


def evaluate(dr, name):
    i0 = len(dr.results)
    pairs(dr, TUNE_DISTS, "T")
    rs = dr.results[i0:]
    s = score(rs)
    stats(rs, name)
    return s


def tune(dr, base):
    dr.verbose = False
    best = dict(base)
    dr.p = best
    best_s = evaluate(dr, "base")
    print("TUNE base score", rnd(best_s))
    for name, vals in SPACE:
        cur = best[name]
        for val in vals:
            if val == cur:
                continue
            cand = dict(best)
            cand[name] = val
            dr.p = cand
            s = evaluate(dr, "%s=%s" % (name, val))
            print("TUNE", name, val, "score", rnd(s), "best", rnd(best_s))
            if s < best_s - TUNE_MARGIN:
                best_s = s
                best = cand
        dr.p = best
    dr.verbose = True
    print("BEST_PARAMS", " ".join("%s=%s" % (k, best[k]) for k in sorted(best)),
          "score", rnd(best_s))
    return best


def suite(dr, tag):
    i0 = len(dr.results)
    pairs(dr, (100, 300, 500, 100, 300, 500), tag)
    stats(dr.results[i0:], tag)


def chain(dr, tag):
    i0 = len(dr.results)
    dr.drive_mm(200, False, label=tag + "C1")
    dr.drive_mm(200, False, label=tag + "C2")
    dr.drive_mm(150, True, label=tag + "C3")
    wait(100)
    dr.drive_mm(-550, True, label=tag + "CB")
    stats(dr.results[i0:], tag + "chain")


def stress(dr):
    base = dr.p
    for vm in (250, 350, 450):
        dr.p = dict(base)
        dr.p["v_max"] = vm
        i0 = len(dr.results)
        pairs(dr, (300, 500), "V%d_" % vm)
        stats(dr.results[i0:], "speed%d" % vm)
    dr.p = base
    i0 = len(dr.results)
    pairs(dr, (500, 500, 500, 500, 500), "R_")
    stats(dr.results[i0:], "repeat500")
    chain(dr, "X_")
    dr.p = dict(base)
    dr.p["v_max"] = 350
    i0 = len(dr.results)
    pairs(dr, (700,), "L_")
    stats(dr.results[i0:], "long700")
    dr.p = base


def wall_square(dr):
    """Sanft rueckwaerts an die Wand, Heading und Encoder nullen, 3 mm davor parken."""
    dr.stop()
    # erst rueckwaerts fahren, bis beide Raeder stehen (Wand erreicht)
    a0 = dr.enc_mm()
    dr.L.dc(-WALL_SEEK_DUTY)
    dr.R.dc(-WALL_SEEK_DUTY)
    sw = StopWatch()
    still = 0
    while True:
        wait(10)
        slow = abs(dr.L.speed()) < 30 and abs(dr.R.speed()) < 30
        if sw.time() > 250 and slow:
            still += 10
        else:
            still = 0
        if still >= 100 or a0 - dr.enc_mm() > WALL_SEEK_MAX_MM or sw.time() > 9000:
            break
    if sw.time() > 800:
        print("WALL_SEEK mm", rnd(a0 - dr.enc_mm()), "ms", sw.time())
    dr.L.dc(-WALL_DUTY)
    dr.R.dc(-WALL_DUTY)
    wait(WALL_PUSH_MS)
    dr.L.brake()
    dr.R.brake()
    wait(150)
    dr.hub.imu.reset_heading(0)
    dr.L.reset_angle(0)
    dr.R.reset_angle(0)
    dr.origin = dr.enc_mm() + WALL_GAP_MM
    dr.nominal = dr.enc_mm()
    dr.x = 0.0
    dr.v = 0
    dr.drive_mm(WALL_GAP_MM, True, v_max=100, label="HOME")
    wait(100)


def speedrun(dr):
    nb = int(RUN_DIST / BIN_MM) + 1
    dr.verbose = False
    base = dict(dr.p)
    for r in range(RUNS):
        v = SPEEDS[r % len(SPEEDS)]
        wall_square(dr)
        dr.p = dict(base)
        dr.p["v_max"] = v
        tf = [0] * nb
        dr.trace = tf
        dr.tr_n = 0
        rf = dr.drive_mm(RUN_DIST, True, v_max=v, label="R%dF" % r)
        wait(150)
        tb = [0] * nb
        dr.trace = tb
        dr.tr_n = 0
        rb = dr.drive_mm(-RUN_DIST, True, v_max=v, label="R%dB" % r)
        dr.trace = None
        wait(100)
        print("TF", r, v, " ".join([str(a) for a in tf]))
        print("TB", r, v, " ".join([str(a) for a in tb]))
        print("RS", r, v, rf[R_TDRIVE], rf[R_ERR], rf[R_HEAD], rf[R_RMS], rf[R_MAXHD],
              rf[R_MAXX], rf[R_LAT], rb[R_TDRIVE], rb[R_ERR], rb[R_HEAD], rb[R_RMS],
              rb[R_MAXHD], rb[R_MAXX], rb[R_LAT])
    dr.p = base
    print("SPEEDRUN_DONE")


def bench(dr):
    """Vergleich: robot.straight() (alt, Pybricks-DriveBase) gegen drive_mm (neu)."""
    from pybricks.robotics import DriveBase
    from robot import Robot
    dr.verbose = False
    db = DriveBase(dr.L, dr.R, wheel_diameter=config.WHEEL_DIAMETER_MM,
                   axle_track=config.AXLE_TRACK_MM)
    db.use_gyro(True)
    rb = Robot(dr.hub, db, Motor(config.LEFT_ATTACHMENT_PORT),
               Motor(config.RIGHT_ATTACHMENT_PORT), dr.L, dr.R)
    rb.reset_drivebase_settings()
    nb = int(RUN_DIST / BIN_MM) + 1
    combos = [(m, v) for m in ("old", "new") for v in SPEEDS]
    base = dict(dr.p)
    for r in range(BENCH_RUNS):
        m, v = combos[r % len(combos)]
        db.stop()
        wait(100)
        wall_square(dr)
        db.stop()
        rb.reset_heading(0)
        dr.p = dict(base)
        tf = [0] * nb
        st = {"n": 0}
        start = dr.enc_mm()
        sw = StopWatch()
        if m == "old":
            rb.set_drivebase_settings(straight_speed=v)

            def tick():
                k = int((dr.enc_mm() - start) / BIN_MM)
                if k > nb - 1:
                    k = nb - 1
                if k < 0:
                    k = 0
                h = dr.hub.imu.heading()
                while st["n"] <= k:
                    tf[st["n"]] = int(round(h * 10))
                    st["n"] += 1

            rb._telemetry_tick = tick
            rb.straight(RUN_DIST)
            rb._telemetry_tick = lambda: None
            while st["n"] < nb:
                tf[st["n"]] = tf[st["n"] - 1]
                st["n"] += 1
        else:
            dr.trace = tf
            dr.tr_n = 0
            dr.drive_mm(RUN_DIST, True, v_max=v, label="B%dF" % r)
            dr.trace = None
        tf_t = sw.time()
        wait(250)
        ef = dr.enc_mm() - start - RUN_DIST
        hf = dr.hub.imu.heading()
        start2 = dr.enc_mm()
        sw.reset()
        db.stop()
        dr.nominal = dr.enc_mm()
        dr.x = 0.0
        dr.drive_mm(start - dr.enc_mm(), True, v_max=250, label="B%dB" % r)
        tb_t = sw.time()
        wait(250)
        eb = dr.enc_mm() - start
        hb = dr.hub.imu.heading()
        q = 0
        mx = 0
        for a in tf:
            q += a * a
            if abs(a) > mx:
                mx = abs(a)
        rms = sqrt(q / nb) / 10
        print("BT", r, m, v, " ".join([str(a) for a in tf]))
        print("BR", r, m, v, tf_t, rnd(ef), rnd(hf), rnd(rms), rnd(mx / 10),
              tb_t, rnd(eb), rnd(hb))
    db.stop()
    dr.p = base
    print("BENCH_DONE")


# MODE "fast": Parametersaetze fuer schnelles, konstantes Fahren vergleichen
FAST_RUNS = 6
FAST_CFG = (
    ("A_basis", {"v_max": 450, "accel": 700, "decel": 500, "ff": 0.0}),
    ("F_dec1000", {"v_max": 500, "accel": 700, "decel": 1000, "ff": 0.0}),
    ("G_a1000", {"v_max": 500, "accel": 1000, "decel": 1000, "ff": 0.0}),
    ("H_a1000kp", {"v_max": 500, "accel": 1000, "decel": 1000, "ff": 0.0, "kp": 12.0, "kd": 0.8}),
    ("I_a1400kp", {"v_max": 500, "accel": 1400, "decel": 1000, "ff": 0.0, "kp": 12.0, "kd": 0.8}),
)


def fastbench(dr):
    dr.verbose = False
    nb = int(RUN_DIST / BIN_MM) + 1
    base = dict(dr.p)
    n = len(FAST_CFG)
    for r in range(FAST_RUNS * n):
        ci = r % n
        name, over = FAST_CFG[ci]
        wall_square(dr)
        dr.p = dict(base)
        for k in over:
            dr.p[k] = over[k]
        tf = [0] * nb
        dr.trace = tf
        dr.tr_n = 0
        start = dr.enc_mm()
        sw = StopWatch()
        dr.drive_mm(RUN_DIST, True, v_max=dr.p["v_max"], label="F%dF" % r)
        dr.trace = None
        tf_t = sw.time()
        wait(250)
        ef = dr.enc_mm() - start - RUN_DIST
        hf = dr.hub.imu.heading()
        dr.p = dict(base)
        dr.nominal = dr.enc_mm()
        dr.x = 0.0
        dr.drive_mm(start - dr.enc_mm(), True, v_max=250, label="F%dB" % r)
        wait(150)
        q = 0
        mx = 0
        for a in tf:
            q += a * a
            if abs(a) > mx:
                mx = abs(a)
        print("FT", r, name, " ".join([str(a) for a in tf]))
        print("FR", r, name, tf_t, rnd(ef), rnd(hf), rnd(sqrt(q / nb) / 10), rnd(mx / 10))
    dr.p = base
    print("FAST_DONE")


# MODE "pid": straight_pid gegen drive_mm
PID_RUNS = 4
PID_MOTOR_ACC = 5000    # deg/s^2, Motor-Beschleunigungsgrenze fuer run()
PID_CFG = (
    ("E1_a900", "pid", {"v": 450, "acc": 900, "dec": 900, "kw": 25.0, "alpha": 0.1, "mode": "dF", "kdiff": 25.0}),
    ("E2_d1200", "pid", {"v": 450, "acc": 900, "dec": 1200, "kw": 25.0, "alpha": 0.1, "mode": "dF", "kdiff": 25.0}),
    ("E3_v480", "pid", {"v": 480, "acc": 1100, "dec": 1100, "kw": 25.0, "alpha": 0.1, "mode": "dF", "kdiff": 25.0}),
    ("A_drive_mm", "mm", {"v_max": 450, "accel": 700, "decel": 500}),
)


def pidbench(dr):
    dr.verbose = False
    for m in (dr.L, dr.R):
        try:
            m.stop()
            sp, ac, tq = m.control.limits()
            m.control.limits(sp, PID_MOTOR_ACC, tq)
            print("LIMITS", m.control.limits())
        except Exception as e:
            print("LIMITS_FAIL", repr(e))
    nb = int(RUN_DIST / BIN_MM) + 1
    base = dict(dr.p)
    n = len(PID_CFG)
    for r in range(PID_RUNS * n):
        name, kind, over = PID_CFG[r % n]
        wall_square(dr)
        tf = [0] * nb
        t2 = [0] * nb
        dr.trace = tf
        dr.trace2 = t2
        dr.tr_n = 0
        start = dr.enc_mm()
        sw = StopWatch()
        if kind == "pid":
            dr.q = dict(PID)
            for k in over:
                dr.q[k] = over[k]
            dr.straight_pid(RUN_DIST, label="P%dF" % r)
        else:
            dr.p = dict(base)
            for k in over:
                dr.p[k] = over[k]
            dr.drive_mm(RUN_DIST, True, v_max=dr.p["v_max"], label="P%dF" % r)
        dr.trace = None
        dr.trace2 = None
        tf_t = sw.time()
        wait(250)
        ef = dr.enc_mm() - start - RUN_DIST
        hf = dr.hub.imu.heading()
        dr.p = dict(base)
        dr.nominal = dr.enc_mm()
        dr.x = 0.0
        dr.drive_mm(start - dr.enc_mm(), True, v_max=250, label="P%dB" % r)
        if kind == "pid":
            print("DT", r, name, " ".join([str(a) for a in t2]))
        wait(150)
        q = 0
        mx = 0
        for a in tf:
            q += a * a
            if abs(a) > mx:
                mx = abs(a)
        print("FT", r, name, " ".join([str(a) for a in tf]))
        print("FR", r, name, tf_t, rnd(ef), rnd(hf), rnd(sqrt(q / nb) / 10), rnd(mx / 10))
    dr.p = base
    print("PID_DONE")


# MODE "final": robot.straight (alt) gegen drive_mm gegen straight_pid
FINAL_RUNS = 6
FINAL_METHODS = ("pidA", "pidB", "pidC")
PID_VARIANTS = {
    "pidA": {"dec": 800},
    "pidB": {"retry_head": 0.25, "tries": 3},
    "pidC": {"dec": 800, "retry_head": 0.25, "tries": 3},
}


def finalbench(dr):
    from pybricks.robotics import DriveBase
    from robot import Robot
    dr.verbose = False
    for m in (dr.L, dr.R):
        try:
            m.stop()
            sp, ac, tq = m.control.limits()
            m.control.limits(sp, PID_MOTOR_ACC, tq)
        except Exception as e:
            print("LIMITS_FAIL", repr(e))
    db = DriveBase(dr.L, dr.R, wheel_diameter=config.WHEEL_DIAMETER_MM,
                   axle_track=config.AXLE_TRACK_MM)
    db.use_gyro(True)
    rb = Robot(dr.hub, db, Motor(config.LEFT_ATTACHMENT_PORT),
               Motor(config.RIGHT_ATTACHMENT_PORT), dr.L, dr.R)
    rb.reset_drivebase_settings()
    nb = int(RUN_DIST / BIN_MM) + 1
    base = dict(dr.p)
    n = len(FINAL_METHODS)
    for r in range(FINAL_RUNS * n):
        m = FINAL_METHODS[r % n]
        db.stop()
        wait(100)
        wall_square(dr)
        db.stop()
        rb.reset_heading(0)
        dr.p = dict(base)
        dr.q = dict(PID)
        tf = [0] * nb
        st = {"n": 0}
        dr.nominal = dr.enc_mm()
        start = dr.enc_mm()
        sw = StopWatch()
        if m == "old":
            rb.set_drivebase_settings(straight_speed=450)

            def tick():
                k = int((dr.enc_mm() - start) / BIN_MM)
                if k > nb - 1:
                    k = nb - 1
                if k < 0:
                    k = 0
                h = dr.hub.imu.heading()
                while st["n"] <= k:
                    tf[st["n"]] = int(round(h * 10))
                    st["n"] += 1

            rb._telemetry_tick = tick
            rb.straight(RUN_DIST)
            rb._telemetry_tick = lambda: None
            while st["n"] < nb:
                tf[st["n"]] = tf[st["n"] - 1]
                st["n"] += 1
        else:
            dr.trace = tf
            dr.tr_n = 0
            if m == "mm":
                dr.drive_mm(RUN_DIST, True, v_max=450, label="X%dF" % r)
            else:
                dr.q.update(PID_VARIANTS.get(m, {}))
                dr.straight_pid(RUN_DIST, label="X%dF" % r)
            dr.trace = None
        tt = sw.time()
        wait(250)
        ef = dr.enc_mm() - start - RUN_DIST
        hf = dr.hub.imu.heading()
        db.stop()
        dr.p = dict(base)
        dr.nominal = dr.enc_mm()
        dr.x = 0.0
        if m == "old":
            rb.straight(start - dr.enc_mm())
            wait(250)
            print("XBACK", r, rnd(dr.enc_mm() - start), rnd(dr.hub.imu.heading()))
        else:
            dr.drive_mm(start - dr.enc_mm(), True, v_max=250, label="X%dB" % r)
        wait(150)
        q = 0
        mx = 0
        for a in tf:
            q += a * a
            if abs(a) > mx:
                mx = abs(a)
        print("XT", r, m, " ".join([str(a) for a in tf]))
        print("XR", r, m, tt, rnd(ef), rnd(hf), rnd(sqrt(q / nb) / 10), rnd(mx / 10))
    db.stop()
    dr.p = base
    print("FINAL_DONE")


def main():
    dr = Driver(dict(DEFAULT))
    print("START mode", MODE, "batt_mV", dr.hub.battery.voltage(),
          "mm_per_deg", rnd(MM_PER_DEG, 4))
    fail = None
    try:
        dr.drive_mm(WALL_GAP_MM, True, v_max=100, label="HOME")
        wait(200)
        if MODE == "smoke":
            pairs(dr, (50, 150), "S")
        elif MODE == "test":
            suite(dr, "suite")
            chain(dr, "X_")
        elif MODE == "tune":
            best = tune(dr, dr.p)
            dr.p = dict(DEFAULT)
            suite(dr, "A_default")
            dr.p = best
            suite(dr, "B_tuned")
        elif MODE == "stress":
            stress(dr)
        elif MODE == "speedrun":
            speedrun(dr)
        elif MODE == "final":
            finalbench(dr)
        elif MODE == "pid":
            pidbench(dr)
        elif MODE == "fast":
            fastbench(dr)
        elif MODE == "bench":
            bench(dr)
    except Exception as e:
        fail = e
        print("EXCEPTION", repr(e))
        dr.stop()
        wait(300)
        back = dr.origin - dr.enc_mm()
        if abs(dr.hub.imu.heading()) > HEAD_JUMP_DEG:
            print("NO_AUTO_RETURN heading", rnd(dr.hub.imu.heading()))
            raise fail
        print("RETURN_TO_START", rnd(back))
        dr.nominal = dr.enc_mm()
        dr.p = dict(DEFAULT)
        dr.drive_mm(back, True, v_max=150, label="RETURN")
    print("END_POS_MM", rnd(dr.pos()), "HEADING", rnd(dr.hub.imu.heading()),
          "LAT_EST", rnd(dr.x))
    stats(dr.results, "ALL")
    if fail is not None:
        raise fail


main()
