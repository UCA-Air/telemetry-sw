"""Pruebas del protocolo: tramas, separador y coincidencia con el firmware C++.

  python3 -m pytest -q pruebas/test_protocolo.py     (o simplemente: python3 pruebas/test_protocolo.py)
"""
import os
import random
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "estacion"))
import enlace as E  # noqa: E402


def test_longitudes():
    assert E.LEN == {0xD2: 40, 0xD3: 22, 0xE1: 12, 0xE2: 8, 0xE3: 6}


def test_crc_modbus():
    assert E.crc16(b"123456789") == 0x4B37          # valor de referencia de CRC-16/MODBUS


def test_ida_y_vuelta():
    f = E.build(E.T_PLANE, seq=3, lat_e7=365319000, lon_e7=-61987000, alt_baro_dm=-12, vbat_mv=16200)
    d = E.parse(f)
    assert d["lat_e7"] == 365319000 and d["lon_e7"] == -61987000 and d["alt_baro_dm"] == -12


def test_separador_trozos_basura_y_corrupcion():
    rng = random.Random(3)
    frames = [E.build(E.T_IMU, seq=i, press_pa=100000 + i) for i in range(30)]
    stream = b""
    for i, f in enumerate(frames):
        stream += bytes(rng.randrange(256) for _ in range(rng.randrange(4)))   # basura entre tramas
        p = bytearray(E.pack(f))
        if i == 10:
            p[8] ^= 0xFF                                                    # una corrupta
        stream += p
    d = E.Deframer()
    out = []
    i = 0
    while i < len(stream):                                                  # trozos de 1–9 bytes
        k = rng.randrange(1, 10)
        out += d.feed(stream[i:i + k])
        i += k
    seqs = [E.parse(f)["seq"] for f in out]
    assert seqs == [s for s in range(30) if s != 10]
    assert d.bad_crc >= 1


def test_banda_y_tiempos():
    assert not E.in_band(E.channel_mhz(0)) and E.in_band(E.channel_mhz(1)) and not E.in_band(E.channel_mhz(2))
    assert 0.05 < E.airtime_s(42, 7) < 0.12            # SF7: unos 94 ms
    assert 2.0 < E.airtime_s(42, 12) < 2.5             # SF12: unos 2,2 s


def test_cruce_con_cpp():
    gpp = shutil.which("g++")
    if not gpp:
        print("(sin g++: me salto el cruce)")
        return
    exe = os.path.join(tempfile.mkdtemp(), "cruce")
    subprocess.check_call([gpp, "-std=c++17", "-I", os.path.join(HERE, "..", "firmware", "nodo", "include"),
                           os.path.join(HERE, "cruce.cpp"), "-o", exe])
    lines = subprocess.check_output([exe], text=True).splitlines()
    frames = {}
    for ln in lines:
        k, *v = ln.split()
        if k == "air":
            sf = int(v[0])
            assert abs(float(v[1]) - E.airtime_s(42, sf)) < 1e-5
            assert abs(float(v[2]) - E.airtime_s(14, sf, 2, 8, True)) < 1e-5
            assert abs(float(v[3]) - E.range_km(10, sf, 434.4, 100, 2)) / float(v[3]) < 1e-3
        elif k == "band":
            assert v == ["0", "1", "0"]
        else:
            frames[k] = bytes.fromhex(v[0])
    for name, raw in frames.items():
        got = E.Deframer().feed(raw)
        assert len(got) == 1, name                     # CRC y longitud coinciden con C++
    # Las tramas que construye Python son idénticas byte a byte
    assert E.pack(E.build(E.T_REPORT, last_seq=9, rx=300, lost=12, bad=3, cfg_id=4)) == frames["report"]
    assert E.pack(E.build(E.T_CHANGE, change_id=5, channel=1, level=3)) == frames["change"]
    assert E.pack(E.build(E.T_ACK, change_id=5, accepted=1)) == frames["ack"]
    # Los datos inventados están en rangos físicos
    p = E.parse(frames["plane"][2:])
    m = E.parse(frames["imu"][2:])
    assert 50 < p["alt_baro_dm"] / 10 < 100 and 36.5 < p["lat_e7"] / 1e7 < 36.56
    assert 0.95 < m["az_mg"] / 1000 < 1.2 and 99000 < m["press_pa"] < 101400
    print("plano:", p)
    print("imu:  ", m)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("OK", name)
