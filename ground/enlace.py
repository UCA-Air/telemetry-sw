"""UCA&Air — Enlace v1 (DX-LR02, 433 MHz): protocolo, cálculos de radio y módulo LR02.

Espejo exacto de firmware/nodo/include/enlace_proto.h y lr02.h. Si cambias algo
allí, cámbialo aquí (las pruebas de pruebas/ comprueban que coinciden byte a byte).
"""
from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field

# ------------------------------------------------------------------ marcas y tipos
SYNC = b"\xAA\x55"
T_PLANE, T_IMU, T_REPORT, T_CHANGE, T_ACK = 0xD2, 0xD3, 0xE1, 0xE2, 0xE3

# Formatos little-endian SIN el CRC final (se añade aparte)
FMT = {
    T_PLANE:  "<BBiihhHHhhHhHHHHhBB",   # 38 + 2 = 40 B
    T_IMU:    "<BBhhhhhhHI",            # 20 + 2 = 22 B
    T_REPORT: "<BBHHHBB",               # 10 + 2 = 12 B
    T_CHANGE: "<BBBBBB",                # 6 + 2 = 8 B
    T_ACK:    "<BBBB",                  # 4 + 2 = 6 B
}
FIELDS = {
    T_PLANE: ("type seq lat_e7 lon_e7 alt_baro_dm alt_gps_m gs_cms cog_cdeg roll_cdeg pitch_cdeg "
              "hdg_cdeg vz_cms current_ca vbat_mv mah dist_m temp_cc sats flags").split(),
    T_IMU: "type seq ax_mg ay_mg az_mg gx_ddps gy_ddps gz_ddps gmax_mg press_pa".split(),
    T_REPORT: "type last_seq rx lost bad cfg_id reserved".split(),
    T_CHANGE: "type change_id channel level reserved pad".split(),
    T_ACK: "type change_id accepted reserved".split(),
}
LEN = {t: struct.calcsize(f) + 2 for t, f in FMT.items()}
assert LEN == {T_PLANE: 40, T_IMU: 22, T_REPORT: 12, T_CHANGE: 8, T_ACK: 6}


def crc16(data: bytes) -> int:
    """CRC-16/MODBUS (igual que en el nodo)."""
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def build(ftype: int, **values) -> bytes:
    """Construye una trama (sin la marca de inicio) con su CRC."""
    vals = [values.get(k, 0) for k in FIELDS[ftype]]
    vals[0] = ftype
    body = struct.pack(FMT[ftype], *vals)
    return body + struct.pack("<H", crc16(body))


def pack(frame: bytes) -> bytes:
    """Lo que se escribe al módulo: marca + trama."""
    return SYNC + frame


def parse(frame: bytes) -> dict:
    t = frame[0]
    return dict(zip(FIELDS[t], struct.unpack(FMT[t], frame[:-2])))


class Deframer:
    """Encuentra tramas en el flujo de bytes del LR02 (que no separa paquetes)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def reset(self):
        self.buf.clear()

    def feed(self, data: bytes):
        """Devuelve la lista de tramas válidas (bytes, sin la marca)."""
        out = []
        self.buf += data
        b = self.buf
        while len(b) >= 3:
            if b[0] != 0xAA or b[1] != 0x55:
                i = b.find(SYNC, 1)
                del b[: i if i > 0 else max(1, len(b) - 1)]
                continue
            L = LEN.get(b[2], 0)
            if not L:
                del b[:1]
                continue
            if len(b) < L + 2:
                break
            f = bytes(b[2:L + 2])
            if crc16(f[:-2]) == struct.unpack("<H", f[-2:])[0]:
                out.append(f)
                del b[:L + 2]
            else:
                self.bad_crc += 1
                del b[:1]
        if len(b) > 256:
            del b[:-128]
        return out


# ------------------------------------------------------------------ radio
BW_HZ = 125000.0
BAND_LO, BAND_HI = 433.050, 434.790
DUTY_LIMIT = 0.10
ERP_LIMIT_DBM = 10.0


def level_sf(level: int) -> int:
    return 12 - level


def channel_mhz(ch: int) -> float:
    return 433.0 + 1.4 * ch


def airtime_s(payload: int, sf: int, cr: int = 2, preamble: int = 8, crc: bool = False) -> float:
    tsym = 2.0 ** sf / BW_HZ
    de = 1 if sf >= 11 else 0
    num = 8 * payload - 4 * sf + 28 + (16 if crc else 0)
    den = 4 * (sf - 2 * de)
    npay = 8 + ((num + den - 1) // den) * (cr + 4) if num > 0 else 8
    return (preamble + 4.25) * tsym + npay * tsym


def snr_min(sf: int) -> float:
    return -7.5 - 2.5 * (sf - 7)


def sensitivity_dbm(sf: int) -> float:
    return -174.0 + 10 * math.log10(BW_HZ) + 6.0 + snr_min(sf)


def range_km(tx_dbm, sf, f_mhz, h_tx, h_rx, gains_dbi=4.0, margin_db=10.0) -> float:
    pl = tx_dbm + gains_dbi - sensitivity_dbm(sf) - margin_db
    d_free = 10 ** ((pl - 20 * math.log10(f_mhz) - 32.44) / 20)
    d_flat = 10 ** ((pl + 20 * math.log10(h_tx * h_rx)) / 40) / 1000
    return min(d_free, d_flat)


def in_band(f_mhz: float) -> bool:
    half = BW_HZ / 2e6
    return f_mhz - half >= BAND_LO and f_mhz + half <= BAND_HI


def interval_for(frame_len: int, sf: int, cr=2, preamble=8, crc=False) -> float:
    """Segundos entre tramas para no pasar del 10 % (con 20 % de margen), mínimo 1 s."""
    return max(1.0, airtime_s(frame_len + 2, sf, cr, preamble, crc) * 1.2 / DUTY_LIMIT)


# ------------------------------------------------------------------ módulo LR02
@dataclass
class ModuleInfo:
    ok: bool = False
    version: str = "?"
    mode: int = -1
    level: int = -1
    sf: int = -1
    cr: int = 2
    preamble: int = 8
    power_dbm: int = -99
    crc: bool = False
    freq_hz: int = 0
    raw: str = field(default="", repr=False)

    @property
    def channel(self) -> int:
        return int((self.freq_hz / 1e6 - 433.0) / 1.4 + 0.5) if self.freq_hz else -1


def parse_help(text: str) -> ModuleInfo:
    import re

    def num(key, conv=int, default=None):
        m = re.search(re.escape(key) + r"\s*(-?\d+)", text)
        return conv(m.group(1)) if m else default

    i = ModuleInfo(raw=text)
    m = re.search(r"VERSION=(\S+)", text)
    if m:
        i.version = m.group(1)
    i.mode = num("MODE:", default=-1)
    i.level = num("LEVEL:", default=-1)
    i.freq_hz = num("Frequency:", default=0)
    i.sf = num("Spreading Factor:", default=-1)
    i.cr = num("Coding rate:", default=2)
    i.crc = bool(num("CRC:", default=0))
    i.preamble = num("Preamble:", default=8)
    i.power_dbm = num("Power:", default=-99)
    i.ok = i.freq_hz > 0 and i.level >= 0
    return i


class Lr02:
    """Controla el LR02 por el puerto serie (mismo procedimiento que lr02.h)."""

    def __init__(self, ser, log=print):
        self.ser = ser
        self.log = log
        self.error = ""

    def _send(self, s: str):
        self.ser.write(s.encode())
        self.ser.flush()

    def _drain(self):
        self.ser.reset_input_buffer()

    def _wait(self, token: str | None, timeout: float, quiet: float = 0.0) -> str:
        buf = b""
        t0 = last = time.monotonic()
        while time.monotonic() - t0 < timeout:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                last = time.monotonic()
            txt = buf.decode("latin-1")
            if token and token in txt and time.monotonic() - last >= quiet:
                return txt
            if not token and buf and time.monotonic() - last >= quiet:
                return txt
            time.sleep(0.005)
        return buf.decode("latin-1")

    def enter_at(self) -> bool:
        self._drain()
        for _ in range(3):
            self._send("+++\r\n")
            r = self._wait(None, 1.2, 0.15)
            if "Entry AT" in r:
                return True
            if "Exit AT" in r:                       # estaba en AT y lo acabamos de sacar
                self._wait("Power on", 3.0)
                time.sleep(0.2)
                self._drain()
                continue
            self._send("AT\r\n")
            if "OK" in self._wait("OK", 0.6):
                return True
            time.sleep(0.3)
        self.error = "el módulo no entra en modo AT"
        return False

    def exit_at(self):
        self._send("+++\r\n")
        self._wait("Power on", 3.0)
        time.sleep(0.15)
        self._drain()

    def command(self, name: str, value: str) -> bool:
        for fmt in ("AT+{}{}\r\n", "AT+{}={}\r\n"):
            self._send(fmt.format(name, value))
            if "OK" in self._wait(None, 0.8, 0.12):
                return True
        self.error = f"AT+{name} rechazado"
        self.log(self.error)
        return False

    def help(self) -> ModuleInfo:
        self._send("AT+HELP\r\n")
        info = parse_help(self._wait("Power", 1.5, 0.12))
        if not info.ok:
            self.error = "AT+HELP ilegible"
        return info

    def read_info(self) -> ModuleInfo:
        if not self.enter_at():
            return ModuleInfo()
        info = self.help()
        self.exit_at()
        return info

    def apply(self, channel: int, level: int, power_idx: int = -1) -> tuple[bool, ModuleInfo]:
        if not self.enter_at():
            return False, ModuleInfo()
        ok = True
        self.command("MODE", "0")
        if channel >= 0:
            ok &= self.command("CHANNEL", f"{channel:02X}")
        if level >= 0:
            ok &= self.command("LEVEL", str(level))
        if power_idx >= 0:
            ok &= self.command("POWE", str(power_idx))
        info = self.help()
        self.exit_at()
        if channel >= 0 and info.channel != channel:
            self.error, ok = "canal no aplicado", False
        if level >= 0 and info.level != level:
            self.error, ok = "nivel no aplicado", False
        return ok, info
