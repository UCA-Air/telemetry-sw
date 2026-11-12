"""Emulador de dos módulos DX-LR02 unidos por una "radio" de mentira (solo Linux/macOS).

Crea dos puertos serie virtuales (pty). Cada uno se comporta como un LR02:
  - modo transparente: lo que entra por el UART sale por radio, con su tiempo en el aire
  - "+++\\r\\n" alterna el modo AT ("Entry AT" / "Exit AT" + "Power on")
  - AT, AT+HELP, AT+MODE, AT+CHANNEL, AT+LEVEL, AT+POWE (formato sin '=' por defecto)
  - el otro módulo solo recibe si está en la misma frecuencia y nivel, no está
    transmitiendo (semidúplex) y no está reiniciándose; además hay pérdidas al azar.

  python3 emulador_lr02.py --perdidas 0.05      imprime las rutas de los dos puertos
"""
from __future__ import annotations

import argparse
import os
import random
import select
import sys
import threading
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "estacion"))
import enlace as E  # noqa: E402

POWE_DBM = [22, 20, 17, 14, 11, 8, 5, 2]       # inventado: el real no está documentado


class Module:
    def __init__(self, name, syntax_eq=False, channel=0, level=0):
        self.name = name
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.syntax_eq = syntax_eq
        self.cfg = dict(mode=0, channel=channel, level=level, powe=0)    # lo que se ve en HELP
        self.active = dict(self.cfg)                                    # lo que usa la radio
        self.at = False
        self.reset_until = 0.0
        self.uart = bytearray()
        self.uart_t = 0.0
        self.tx_queue = []
        self.tx_busy_until = 0.0
        self.tx_log = []                        # (inicio, fin) de cada emisión
        self.rx_bytes = 0

    def out(self, data: bytes):
        os.write(self.master, data)

    # ---------------------------------------------------------------- AT
    def help_text(self):
        c = self.cfg
        sf = 12 - c["level"]
        f = int(round((433.0 + 1.4 * c["channel"]) * 1e6))
        return ("LoRa Parameter:\r\n+VERSION=V2.2.0\r\n"
                f"MODE:{c['mode']}\r\nLEVEL:{c['level']} >> {125000 * sf / 2 ** sf * 4 / 6:.6f}bps\r\n"
                f"SLEEP:2\r\nFrequency:{f}hz\r\nMAC:ff,ff\r\nBandwidth:0\r\n"
                f"Spreading Factor:{sf}\r\nCoding rate:2\r\nCRC:0(false)\r\nPreamble:8\r\n"
                f"IQ:0(false)\r\nPower:{POWE_DBM[c['powe']]}dBm\r\n")

    def at_line(self, line: str):
        line = line.strip()
        if not line:
            return
        if line == "AT":
            return self.out(b"OK\r\n")
        if line == "AT+HELP":
            return self.out(self.help_text().encode())
        for name, key, base, hi in (("CHANNEL", "channel", 16, 0x1E), ("LEVEL", "level", 10, 7),
                                    ("MODE", "mode", 10, 2), ("POWE", "powe", 10, 7)):
            pre = f"AT+{name}"
            if line.startswith(pre):
                v = line[len(pre):]
                if v.startswith("=") != self.syntax_eq:
                    return self.out(b"ERROR\r\n")
                v = v.lstrip("=")
                try:
                    n = int(v, base)
                except ValueError:
                    return self.out(b"ERROR\r\n")
                if not 0 <= n <= hi:
                    return self.out(b"ERROR\r\n")
                self.cfg[key] = n
                return self.out(b"OK\r\n")
        self.out(b"ERROR\r\n")

    # ---------------------------------------------------------------- UART → paquete
    def packet(self, data: bytes, now, radio):
        if data == b"+++\r\n":
            if not self.at:
                self.at = True
                self.out(b"Entry AT\r\n")
            else:
                self.at = False
                self.out(b"Exit AT\r\n")
                self.reset_until = now + 0.4
                radio.later(now + 0.4, lambda: self.out(b"Power on\r\n"))
                self.active = dict(self.cfg)
            return
        if self.at:
            for ln in data.decode("latin-1").split("\n"):
                self.at_line(ln)
            return
        if now < self.reset_until:
            return
        a = self.active
        air = E.airtime_s(len(data), 12 - a["level"])
        start = max(now, self.tx_busy_until)
        self.tx_busy_until = start + air
        self.tx_log.append((start, start + air))
        radio.later(start + air, lambda: radio.deliver(self, data, start, start + air, dict(a)))


class Radio:
    def __init__(self, loss=0.0, seed=1, **kw):
        self.a = Module("A", **kw)
        self.b = Module("B", **kw)
        self.loss = loss
        self.rng = random.Random(seed)
        self.timers = []
        self.stop = False
        self.delivered = self.dropped = 0

    def later(self, t, fn):
        self.timers.append((t, fn))

    def deliver(self, src, data, t0, t1, cfg):
        dst = self.b if src is self.a else self.a
        d = dst.active
        same = d["channel"] == cfg["channel"] and d["level"] == cfg["level"]
        busy = any(s < t1 and e > t0 for s, e in dst.tx_log[-5:])
        if (not same or dst.at or time.monotonic() < dst.reset_until or busy
                or self.rng.random() < self.loss):
            self.dropped += 1
            return
        self.delivered += 1
        dst.rx_bytes += len(data)
        dst.out(data)

    def run(self):
        mods = {self.a.master: self.a, self.b.master: self.b}
        while not self.stop:
            now = time.monotonic()
            r, _, _ = select.select(list(mods), [], [], 0.002)
            for fd in r:
                m = mods[fd]
                try:
                    m.uart += os.read(fd, 4096)
                except OSError:
                    continue
                m.uart_t = now
            now = time.monotonic()
            for m in mods.values():
                # el módulo da por terminado el paquete tras ~10 ms sin bytes por el UART
                if m.uart and now - m.uart_t > 0.010:
                    data, m.uart[:] = bytes(m.uart), b""
                    m.packet(data, now, self)
            due = [x for x in self.timers if x[0] <= now]
            self.timers = [x for x in self.timers if x[0] > now]
            for _, fn in due:
                fn()

    def start(self):
        threading.Thread(target=self.run, daemon=True).start()
        return self


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--perdidas", type=float, default=0.0)
    ap.add_argument("--con-igual", action="store_true", help="el módulo exige AT+NOMBRE=valor")
    a = ap.parse_args()
    r = Radio(loss=a.perdidas, syntax_eq=a.con_igual)
    print(f"Módulo A (nodo):     {r.a.path}")
    print(f"Módulo B (estación): {r.b.path}")
    print("Ctrl+C para parar.")
    try:
        r.run()
    except KeyboardInterrupt:
        print(f"\nEntregados {r.delivered}, perdidos {r.dropped}")


if __name__ == "__main__":
    main()
