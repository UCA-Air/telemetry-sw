"""Prueba completa sin hardware: nodo (firmware compilado para el PC) ↔ emulador LR02 ↔ estación.

Comprueba, con un 5 % de pérdidas en la "radio":
  1. emparejamiento: la estación recibe y el nodo muestra las pérdidas que le informa
  2. cambio de nivel desde los botones del nodo, seguido por la estación
  3. vuelta atrás: la estación acusa pero no cambia → el nodo vuelve solo
  4. rechazo: el nodo pide el canal 00 (433,0 MHz, fuera de banda) y la estación dice que no

  python3 pruebas/test_integracion.py          (Linux/macOS, necesita g++ y pyserial)
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "estacion"))
sys.path.insert(0, HERE)
import serial                      # noqa: E402
from emulador_lr02 import Radio    # noqa: E402
from nucleo import Station         # noqa: E402

tmp = tempfile.mkdtemp()
exe = os.path.join(tmp, "host_nodo")
subprocess.check_call(["g++", "-std=c++17", "-O1", "-I", os.path.join(HERE, "..", "firmware", "nodo", "include"),
                       os.path.join(HERE, "host_nodo.cpp"), "-o", exe])

radio = Radio(loss=0.05).start()      # los módulos salen de fábrica en canal 00, nivel 0
node = subprocess.Popen([exe, radio.a.path, "9600", os.path.join(tmp, "cfg.txt")],
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
screen, logs = [""], []


def reader():
    for ln in node.stdout:
        if " SCR " in ln:
            screen[0] = ln.split(" SCR ", 1)[1].strip()
        else:
            logs.append(ln.strip())
            print("   nodo |", ln.strip())


threading.Thread(target=reader, daemon=True).start()
ser = serial.Serial(radio.b.path, 9600, timeout=0)
st = Station(ser)
st.start()
seen_events = 0


def run(seconds, until=None):
    global seen_events
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        st.poll()
        while seen_events < len(st.events):
            print("estación|", list(st.events)[seen_events])
            seen_events += 1
        if seen_events > len(st.events):
            seen_events = len(st.events)
        if until and until():
            return True
        time.sleep(0.005)
    return bool(until and until())


def press(keys):
    for k in keys:
        node.stdin.write(k + "\n")
        node.stdin.flush()
        run(0.15)


def check(cond, msg):
    print(("  ✔ " if cond else "  ✘ ") + msg)
    if not cond:
        print("pantalla:", screen[0])
        node.kill()
        sys.exit(1)


try:
    print("1) Emparejamiento en canal 01 / nivel 5")
    check(run(25, lambda: st.rx >= 10 and "Perd" in screen[0]), "la estación recibe y el nodo ve el informe")
    run(5)
    check(st.plane is not None and st.imu is not None, "llegan tramas de avión y de IMU")
    print(f"     recibidas {st.rx}, perdidas {st.lost} ({st.loss_pct():.1f} %), pantalla {screen[0]}")

    print("2) Cambio a nivel 4 (SF8) con los botones: ABAJO ABAJO · CENTRO · ABAJO · CENTRO")
    press(["d", "d", "o", "d", "o"])
    check(run(30, lambda: any("CAMBIO OK" in l for l in logs)), "el nodo confirma el cambio")
    check(st.level == 4 and st.phase == st.IDLE, "la estación está en nivel 4")
    rx0 = st.rx
    run(6)
    check(st.rx > rx0, "siguen llegando tramas en nivel 4")

    print("3) Vuelta atrás: la estación acusa el nivel 3 pero no cambia")
    st.ignore_changes = True
    logs.clear()
    press(["o", "d", "o"])
    check(run(40, lambda: any("VUELTA ATRAS" in l for l in logs)), "el nodo vuelve atrás solo")
    st.ignore_changes = False
    rx0 = st.rx
    check(run(20, lambda: st.rx > rx0 + 3), "y la estación lo vuelve a oír en nivel 4")

    print("4) Rechazo: el nodo pide el canal 00 (433,0 MHz)")
    logs.clear()
    press(["u", "o", "d", "o"])          # página FRECUENCIA, canal 01 → 00
    check(run(15, lambda: any("RECHAZADO" in l for l in logs)), "la estación lo rechaza por estar fuera de banda")
    check(st.channel == 1, "los dos siguen en el canal 01")
    print(f"\nTodo bien. Radio: {radio.delivered} entregados, {radio.dropped} perdidos/no oídos.")
finally:
    node.kill()
    radio.stop = True
