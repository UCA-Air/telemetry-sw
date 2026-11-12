"""UCA&Air — Estación de tierra v1 (LR02 + adaptador USB-TTL en el ordenador).

Uso:
  python estacion.py --listar                 puertos serie disponibles
  python estacion.py --port COM5 --diag       lee y muestra la configuración del módulo
  python estacion.py --port COM5              estación en marcha (Ctrl+C para salir)
  python estacion.py --port COM5 --canal 1 --nivel 5 --registro registros

Funciona en el cmd de Windows, en PowerShell y en terminales de Linux/macOS.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import enlace as E          # noqa: E402
from nucleo import Station  # noqa: E402

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("Falta pyserial:  pip install -r requirements.txt")

# ------------------------------------------------------------------ colores ANSI
if os.name == "nt":
    os.system("")            # activa las secuencias ANSI en la consola de Windows 10/11
B, DIM, R, G, Y, C, X = "\x1b[1m", "\x1b[2m", "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[36m", "\x1b[0m"


def bar(pct_ok: float, width=20) -> str:
    n = int(round(pct_ok / 100 * width))
    col = G if pct_ok >= 95 else Y if pct_ok >= 80 else R
    return col + "█" * n + DIM + "░" * (width - n) + X


def ago(t, now):
    return "—" if t is None else f"{now - t:4.1f} s"


def render(st: Station, port: str) -> str:
    now = st.clock()
    i = st.info
    L = []
    L.append(f"{B}{C}UCA&Air · Estación de tierra v1{X}   {DIM}{port} · {time.strftime('%H:%M:%S')} · Ctrl+C sale{X}")
    L.append("")
    # --- enlace
    legal_f = E.in_band(st.freq_mhz)
    legal_p = i.power_dbm <= E.ERP_LIMIT_DBM
    L.append(f"{B}ENLACE{X}   {B}{st.freq_mhz:.3f} MHz{X}  canal {st.channel:02X}  nivel {st.level} (SF{st.sf}, 125 kHz)  "
             f"{i.power_dbm} dBm   "
             + (f"{G}banda OK{X}" if legal_f else f"{R}FUERA DE BANDA{X}")
             + ("" if legal_p else f"  {Y}potencia > 10 mW{X}"))
    ok_recent = 100 - st.recent_loss_pct()
    silent = st.last_frame_at is None or now - st.last_frame_at > max(10, 3 * st.plane_interval())
    estado = f"{R}SIN SEÑAL DEL NODO{X}" if silent else f"{G}recibiendo{X}"
    L.append(f"         {estado}   última trama hace {ago(st.last_frame_at, now)}   "
             f"una cada {st.plane_interval():.1f} s (límite 10 %)")
    L.append(f"         recibidas {B}{st.rx}{X}  perdidas {B}{st.lost}{X} ({st.loss_pct():.1f} %)  "
             f"CRC malo {st.bad}   últimas 50 {bar(ok_recent)} {ok_recent:.0f} %")
    fase = f"{Y}confirmando cambio ({st.deadline - now:.0f} s){X}" if st.phase == st.CONFIRM else "normal"
    L.append(f"         informes al nodo {st.reports_sent}   fase: {fase}   "
             f"alcance teórico: suelo {E.range_km(i.power_dbm if i.ok else 22, st.sf, st.freq_mhz, 1.5, 1.5):.1f} km, "
             f"avión a 100 m {E.range_km(i.power_dbm if i.ok else 22, st.sf, st.freq_mhz, 100, 2):.0f} km")
    L.append("")
    # --- avión
    p = st.plane
    if p:
        sats = p["sats"] & 0x3F
        fix = {0: "sin fix", 1: "2D", 2: "3D", 3: "3D+"}[p["sats"] >> 6]
        L.append(f"{B}AVIÓN{X}    alt. baro {p['alt_baro_dm'] / 10:6.1f} m   alt. GPS {p['alt_gps_m']:4d} m   "
                 f"Vz {p['vz_cms'] / 100:+5.1f} m/s   {DIM}(trama {p['seq']}){X}")
        L.append(f"         posición {p['lat_e7'] / 1e7:.6f}, {p['lon_e7'] / 1e7:.6f}   {sats} sat {fix}   "
                 f"recorrido {p['dist_m']} m")
        L.append(f"         rumbo {p['hdg_cdeg'] / 100:5.1f}°  rumbo suelo {p['cog_cdeg'] / 100:5.1f}°  "
                 f"vel. suelo {p['gs_cms'] / 100:4.1f} m/s   alabeo {p['roll_cdeg'] / 100:+5.1f}°  "
                 f"cabeceo {p['pitch_cdeg'] / 100:+5.1f}°")
        L.append(f"         batería {p['vbat_mv'] / 1000:5.2f} V  {p['current_ca'] / 100:4.1f} A  {p['mah']} mAh   "
                 f"temperatura {p['temp_cc'] / 100:4.1f} °C   sensores 0x{p['flags']:02X}")
    else:
        L.append(f"{B}AVIÓN{X}    {DIM}sin datos todavía{X}")
    L.append("")
    m = st.imu
    if m:
        L.append(f"{B}IMU{X}      aceleración x {m['ax_mg'] / 1000:+5.2f}  y {m['ay_mg'] / 1000:+5.2f}  "
                 f"z {m['az_mg'] / 1000:+5.2f} g   pico {m['gmax_mg'] / 1000:4.2f} g")
        L.append(f"         giro x {m['gx_ddps'] / 10:+6.1f}  y {m['gy_ddps'] / 10:+6.1f}  "
                 f"z (viraje) {m['gz_ddps'] / 10:+6.1f} °/s   presión {m['press_pa'] / 100:7.2f} hPa")
    else:
        L.append(f"{B}IMU{X}      {DIM}sin datos todavía{X}")
    L.append("")
    L.append(f"{B}EVENTOS{X}")
    for e in list(st.events)[-6:]:
        L.append("  " + e)
    return "\x1b[H" + "\n".join(x + "\x1b[K" for x in L) + "\n\x1b[J"


def diag(ser, port):
    lr = E.Lr02(ser)
    print(f"Leyendo el módulo en {port}…")
    info = lr.read_info()
    if not info.ok:
        print(f"{R}No responde{X} ({lr.error}). Revisa: TX del módulo → RX del adaptador y al revés, "
              "GND común, 9600 baudios.")
        return 1
    print(info.raw.strip())
    print()
    f = info.freq_hz / 1e6
    print(f"Canal {info.channel:02X} = {f:.3f} MHz  →  "
          + (f"{G}dentro de banda{X}" if E.in_band(f) else f"{R}FUERA de 433,05–434,79 MHz: usa --canal 1{X}"))
    print(f"Potencia {info.power_dbm} dBm  →  "
          + (f"{G}legal{X}" if info.power_dbm <= E.ERP_LIMIT_DBM else f"{Y}supera 10 mW (prueba --potencia){X}"))
    return 0


def main():
    ap = argparse.ArgumentParser(description="Estación de tierra UCA&Air v1 (DX-LR02)")
    ap.add_argument("--port", help="puerto serie del adaptador (COM5, /dev/ttyUSB0…)")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--listar", action="store_true", help="lista los puertos serie")
    ap.add_argument("--diag", action="store_true", help="muestra la configuración del módulo y sale")
    ap.add_argument("--canal", type=lambda s: int(s, 0), default=1, help="canal inicial (1 = 434,4 MHz)")
    ap.add_argument("--nivel", type=int, default=5, choices=range(8), help="nivel inicial 0–7 (5 = SF7)")
    ap.add_argument("--potencia", type=int, default=-1, help="índice AT+POWE (−1 = no tocar)")
    ap.add_argument("--registro", default="registros", help="carpeta de registros ('' = sin registro)")
    ap.add_argument("--permitir-fuera-banda", action="store_true", help="acepta canales fuera de la banda legal")
    ap.add_argument("--ignorar-cambios", action="store_true", help=argparse.SUPPRESS)   # pruebas
    a = ap.parse_args()

    if a.listar or not a.port:
        ports = list(list_ports.comports())
        print("Puertos serie:" if ports else "No hay puertos serie. ¿Está enchufado el adaptador?")
        for p in ports:
            print(f"  {p.device:12s} {p.description}")
        if not a.port:
            print("\nEjemplo:  python estacion.py --port COM5")
        return 0

    try:
        ser = serial.Serial(a.port, a.baud, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"No puedo abrir {a.port}: {e}")
    time.sleep(0.3)
    if a.diag:
        return diag(ser, a.port)

    st = Station(ser, a.canal, a.nivel, a.potencia, log_dir=a.registro or None,
                 allow_out_of_band=a.permitir_fuera_banda, ignore_changes=a.ignorar_cambios)
    print("Configurando el módulo…")
    st.start()
    sys.stdout.write("\x1b[2J")
    last = 0.0
    try:
        while True:
            st.poll()
            if time.monotonic() - last > 0.4:
                sys.stdout.write(render(st, a.port))
                sys.stdout.flush()
                last = time.monotonic()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        st.close()
        ser.close()
        print(f"\n{X}Fin. Recibidas {st.rx}, perdidas {st.lost} ({st.loss_pct():.1f} %).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
