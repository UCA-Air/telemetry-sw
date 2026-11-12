"""UCA&Air — Lógica de la estación de tierra v1 (sin pantalla).

Hace tres cosas:
  1. Separa las tramas que llegan del LR02 y cuenta recibidas, perdidas y corruptas.
  2. Devuelve al nodo un informe (0xE1) para que su pantalla muestre las pérdidas.
  3. Sigue los cambios de canal/nivel que propone el nodo (0xE2 → 0xE3) y, si
     tras cambiar no oye al nodo, vuelve a la configuración anterior.
"""
from __future__ import annotations

import collections
import csv
import os
import time

import enlace as E

RENDEZVOUS = (1, 5)          # canal 01 = 434,4 MHz, nivel 5 = SF7 (igual que el nodo)
SILENCE_TO_RENDEZVOUS = 60.0


class Station:
    IDLE, CONFIRM = "normal", "confirmando"

    def __init__(self, ser, channel=1, level=5, power_idx=-1, log_dir=None,
                 allow_out_of_band=False, ignore_changes=False, clock=time.monotonic):
        self.ser = ser
        self.lr = E.Lr02(ser, log=self.event)
        self.clock = clock
        self.allow_oob = allow_out_of_band
        self.ignore_changes = ignore_changes       # solo para pruebas: acusa pero no cambia
        self.power_idx = power_idx
        self.events = collections.deque(maxlen=8)
        self.deframer = E.Deframer()
        self.info = E.ModuleInfo()
        self.channel, self.level = channel, level
        self.phase = self.IDLE
        self.cfg_id = 0
        self.change_id = None
        self.old = None
        self.deadline = 0.0
        self.plane = None
        self.imu = None
        self.reports_sent = 0
        self.frames_total = 0
        self._reset_counters()
        self._csv = self._open_logs(log_dir) if log_dir else None

    # ---------------------------------------------------------------- utilidades
    def event(self, msg: str):
        self.events.append(time.strftime("%H:%M:%S ") + msg)

    def _reset_counters(self):
        now = self.clock()
        self.rx = self.lost = 0
        self.bad_base = self.deframer.bad_crc
        self.last_seq = None
        self.last_frame_at = None
        self.last_report_at = 0.0
        self.recent = collections.deque(maxlen=50)   # 1 = llegó, 0 = perdida
        self.since = now
        self.last_heard = now

    @property
    def bad(self):
        return self.deframer.bad_crc - self.bad_base

    @property
    def sf(self):
        return self.info.sf if self.info.sf > 0 else E.level_sf(self.level)

    @property
    def freq_mhz(self):
        return self.info.freq_hz / 1e6 if self.info.freq_hz else E.channel_mhz(self.channel)

    def loss_pct(self):
        t = self.rx + self.lost
        return 100.0 * self.lost / t if t else 0.0

    def recent_loss_pct(self):
        return 100.0 * (1 - sum(self.recent) / len(self.recent)) if self.recent else 0.0

    def plane_interval(self):
        i = self.info
        return E.interval_for(E.LEN[E.T_PLANE], self.sf, i.cr, i.preamble, i.crc)

    def _airtime(self, ftype):
        i = self.info
        return E.airtime_s(E.LEN[ftype] + 2, self.sf, i.cr, i.preamble, i.crc)

    def report_interval(self):
        # La estación también respeta el 10 %: como mucho un informe cada este tiempo
        return max(2.0, self._airtime(E.T_REPORT) * 1.2 / E.DUTY_LIMIT)

    def confirm_timeout(self):
        # Un poco más que el del nodo, para que sea el nodo quien decida primero
        return 10.0 + 3 * self.plane_interval() + self._airtime(E.T_REPORT)

    # ---------------------------------------------------------------- módulo
    def apply(self, channel, level, why=""):
        ok, info = self.lr.apply(channel, level, self.power_idx)
        if info.ok:
            self.info = info
        if ok:
            self.channel, self.level = channel, level
        elif info.ok:
            self.channel, self.level = info.channel, info.level
            self.event(f"ERROR al configurar: {self.lr.error}")
        else:
            self.event("el módulo no responde (¿TX/RX cruzados? ¿9600 baudios?)")
        self.cfg_id = (self.cfg_id + 1) & 0xFF
        self.deframer.reset()
        self._reset_counters()
        if info.ok:
            self.event(f"{why}{self.freq_mhz:.3f} MHz, nivel {self.level} (SF{self.sf}), {info.power_dbm} dBm")
        return ok

    def start(self):
        return self.apply(self.channel, self.level, "inicio: ")

    def _write(self, frame: bytes):
        self.ser.write(E.pack(frame))
        self.ser.flush()

    # ---------------------------------------------------------------- bucle
    def poll(self):
        n = self.ser.in_waiting
        data = self.ser.read(n) if n else b""
        if data:
            if self._csv:
                self._raw.write(f"{time.time():.3f} {data.hex()}\n")
            for f in self.deframer.feed(data):
                self._on_frame(f)
        now = self.clock()
        if self.phase == self.CONFIRM and now > self.deadline:
            self.phase = self.IDLE
            ch, lv = self.old
            self.event("el nodo no aparece en la nueva configuración: VUELTA ATRÁS")
            self.apply(ch, lv, "vuelta atrás: ")
        if (self.phase == self.IDLE and (self.channel, self.level) != RENDEZVOUS
                and now - self.last_heard > SILENCE_TO_RENDEZVOUS):
            self.event("un minuto sin oír al nodo: vuelvo al canal de encuentro")
            self.apply(*RENDEZVOUS, "encuentro: ")

    def _on_frame(self, f: bytes):
        now = self.clock()
        self.last_heard = now
        t = f[0]
        if t in (E.T_PLANE, E.T_IMU):
            self._count(f[1])
            self.last_frame_at = now
            self.frames_total += 1
            d = E.parse(f)
            if t == E.T_PLANE:
                self.plane = d
            else:
                self.imu = d
            self._log(t, d)
            if self.phase == self.CONFIRM:
                self.phase = self.IDLE
                self.event("cambio confirmado: el nodo se oye en la nueva configuración")
            if now - self.last_report_at >= self.report_interval():
                self._send_report(f[1])
        elif t == E.T_CHANGE:
            self._on_change(E.parse(f))

    def _count(self, seq):
        if self.last_seq is None:
            self.rx = 1
        else:
            gap = (seq - self.last_seq - 1) & 0xFF
            if gap > 100:                 # salto enorme: el nodo se ha reiniciado
                gap = 0
                self.event("el nodo parece haberse reiniciado")
            self.lost += gap
            self.recent.extend([0] * min(gap, 50))
            self.rx += 1
        self.recent.append(1)
        self.last_seq = seq

    def _send_report(self, last_seq):
        r = E.build(E.T_REPORT, last_seq=last_seq, rx=min(self.rx, 65535),
                    lost=min(self.lost, 65535), bad=min(self.bad, 65535), cfg_id=self.cfg_id)
        self._write(r)
        self.last_report_at = self.clock()
        self.reports_sent += 1

    def _on_change(self, c):
        ch, lv, cid = c["channel"], c["level"], c["change_id"]
        if self.phase == self.CONFIRM and cid == self.change_id:
            return                                       # repetición de una propuesta ya aceptada
        f = E.channel_mhz(ch)
        valid = 0 <= ch <= 0x1E and 0 <= lv <= 7
        legal = E.in_band(f) or self.allow_oob
        accepted = valid and legal
        self._write(E.build(E.T_ACK, change_id=cid, accepted=int(accepted)))
        if not accepted:
            why = "fuera de la banda de 433,05–434,79 MHz" if valid else "valores imposibles"
            self.event(f"RECHAZO canal {ch:02X} ({f:.1f} MHz) nivel {lv}: {why}")
            return
        self.event(f"el nodo pide canal {ch:02X} ({f:.1f} MHz) nivel {lv} (SF{E.level_sf(lv)}): acepto")
        if self.ignore_changes:
            self.event("(prueba) ignoro el cambio a propósito")
            return
        time.sleep(self._airtime(E.T_ACK) + 0.1)        # que el acuse salga antes de cambiar
        self.old = (self.channel, self.level)
        self.change_id = cid
        self.apply(ch, lv, "cambio: ")
        self.phase = self.CONFIRM
        self.deadline = self.clock() + self.confirm_timeout()

    # ---------------------------------------------------------------- registro
    def _open_logs(self, d):
        os.makedirs(d, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._raw = open(os.path.join(d, f"{stamp}_bruto.txt"), "w", buffering=1)
        files = {}
        for t, name in ((E.T_PLANE, "avion"), (E.T_IMU, "imu")):
            fh = open(os.path.join(d, f"{stamp}_{name}.csv"), "w", newline="", buffering=1)
            w = csv.writer(fh)
            w.writerow(["t_unix", "canal", "nivel"] + E.FIELDS[t][1:])
            files[t] = (fh, w)
        return files

    def _log(self, t, d):
        if self._csv:
            self._csv[t][1].writerow([f"{time.time():.3f}", self.channel, self.level]
                                     + [d[k] for k in E.FIELDS[t][1:]])

    def close(self):
        if self._csv:
            for fh, _ in self._csv.values():
                fh.close()
            self._raw.close()
