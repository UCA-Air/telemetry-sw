# UCA&Air · Enlace LoRa v1 (433 MHz)

Primer emparejamiento entre un **nodo de vuelo** y una **estación de tierra**:

- **Nodo de vuelo:** ESP32-C3 SuperMini, módulo DX-LR02, pantalla de 0,66" y 3 botones.
- **Estación de tierra:** otro DX-LR02 con su adaptador USB-TTL, conectado al ordenador.

Como todavía no hay sensores, el nodo **se inventa un vuelo**. Ese vuelo es coherente: los virajes de 25° dan 1,10 g, la presión baja con la altura, la batería se gasta… y los datos viajan en el formato real de las tramas.

```
 ┌───────── NODO ─────────┐        433 MHz        ┌────── ESTACIÓN ──────┐
 │ vuelo inventado        │  ── 0xD2 avión ─────▶ │ cuenta recibidas,    │
 │ menú 64×48 + 3 botones │  ── 0xD3 IMU ───────▶ │ perdidas, CRC malos  │
 │ muestra pérdidas,      │ ◀── 0xE1 informe ──── │ muestra los datos    │
 │ frecuencia, alcance    │  ── 0xE2 propuesta ─▶ │ sigue los cambios    │
 │                        │ ◀── 0xE3 acuse ────── │ guarda CSV           │
 └────────────────────────┘                       └──────────────────────┘
```

## Estructura

| Carpeta | Qué hay |
|---|---|
| `airborne/` | Proyecto PlatformIO del nodo. El núcleo está en `include/` y es C++ portable. |
| `ground/` | Estación en Python: `estacion.py` es la consola y `nucleo.py` la lógica. `enlace.py` refleja el protocolo byte a byte. |
| `tools/tests/` | Emulador de dos LR02, nodo compilado para PC, pruebas de protocolo y prueba completa. |

---

## 1. Montaje del nodo

| Pieza | Pin | → ESP32-C3 SuperMini |
|---|---|---|
| LR02 | TXD | GPIO20 |
| LR02 | RXD | GPIO7 |
| LR02 | VCC / GND | 5V / GND |
| LR02 | M0, M1, AUX | sin conectar (ver problemas) |
| OLED 0,66" | SDA / SCL | GPIO5 / GPIO6 |
| OLED | VCC / GND | 3V3 / GND |
| Botón ARRIBA | — | GPIO3 ↔ GND |
| Botón CENTRO | — | GPIO4 ↔ GND (el botón BOOT de la placa hace lo mismo) |
| Botón ABAJO | — | GPIO10 ↔ GND |

Los botones no necesitan resistencias, porque se usan las internas del chip.

**Ojo con el GPIO21.** No se usa a propósito: el C3 escribe ahí sus mensajes de arranque, y el LR02 los emitiría por radio.

**Estación.** Los pines van así entre el LR02 y el adaptador USB-TTL:

- TXD del LR02 → RXD del adaptador.
- RXD del LR02 → TXD del adaptador.
- VCC → 5V y GND → GND.

Si el kit trae el adaptador ya montado, solo hay que enchufarlo.

## 2. Grabar el nodo

**Con PlatformIO (recomendado):**

```
cd airborne
pio run -t upload
pio device monitor
```

Antes de grabar, pon las coordenadas de vuestro campo en `platformio.ini` (`HOME_LAT` y `HOME_LON`).

**Con Arduino IDE:**

1. Crea una carpeta `nodo/`.
2. Copia `src/main.cpp` a esa carpeta como `nodo.ino`.
3. Copia también los 5 ficheros de `include/`.
4. Elige la placa *ESP32C3 Dev Module* y activa *USB CDC On Boot: Enabled*.
5. Instala la librería **U8g2**.

## 3. Arrancar la estación (Windows)

```
cd ground
pip install -r requirements.txt
python estacion.py --listar
python estacion.py --port COM5 --diag
python estacion.py --port COM5
```

- `--listar` dice en qué COM está el adaptador.
- `--diag` lee la configuración del módulo y avisa de lo ilegal.
- La última línea pone la estación en marcha. Para salir, Ctrl+C.

Los registros se guardan en `ground/registros/`:

- `*_avion.csv` y `*_imu.csv`: los datos ya decodificados.
- `*_bruto.txt`: todo lo recibido, en hexadecimal.

## 4. El menú del nodo

**ARRIBA** y **ABAJO** pasan de página. **CENTRO** entra a editar. Mientras editas, ARRIBA y ABAJO cambian el valor (si los mantienes pulsados, se repiten) y CENTRO aplica.

| # | Página | Qué muestra / hace |
|---|---|---|
| 1 | ENLACE | % de pérdidas según la estación, recibidas/enviadas, antigüedad del último informe y frecuencia. Si no hay informes, muestra «SIN ESTACION». |
| 2 | FRECUENCIA | Frecuencia y canal, y si está dentro de la banda legal. **Editable:** canal 00–1E. |
| 3 | NIVEL | Nivel 0–7 (SF12…SF5), velocidad y cada cuánto sale una trama. **Editable.** |
| 4 | POTENCIA | dBm que dice el módulo y aviso si pasa de 10 mW. **Editable:** índice AT+POWE. |
| 5 | ALCANCE | Alcance teórico en suelo (1,5 m) y con el avión a 100 m. |
| 6 | VUELO | Altitud, velocidad sobre el suelo, alabeo, cabeceo y rumbo. |
| 7 | IMU | Factor de carga y velocidad de viraje. |
| 8 | GPS | Latitud y longitud. |
| 9 | MODULO | Lo que devuelve AT+HELP: versión, frecuencia real, SF y potencia. |
| 10 | EMISION | Tramas enviadas y el intervalo. CENTRO pausa o reanuda la emisión. |

### Cómo funciona un cambio de canal o de nivel

Es como cambiar de canal con un walkie sin perder al compañero. El nodo no cambia solo:

1. **Propone** el cambio (0xE2).
2. La estación **acusa** (0xE3) y los dos cambian a la vez.
3. En la nueva configuración, el nodo espera el primer informe de la estación: «CAMBIO OK».
4. Si la estación no aparece, el nodo hace **VUELTA ATRAS** y la estación igual.

Hay además una red de seguridad: con **un minuto de silencio**, los dos vuelven al canal de encuentro (canal 01, nivel 5).

La estación **rechaza** los canales fuera de banda. La potencia solo afecta al nodo, así que se cambia sin avisar a la estación.

---

## ⚠️ Antes de encender: legalidad y límites del LR02

- **Frecuencia.** La banda libre en España/UE es **433,050–434,790 MHz**. Con pasos de 1,4 MHz, el único canal del LR02 que cabe es el **01 (434,4 MHz)**.
  - El módulo sale de fábrica en el **canal 00 (433,0 MHz), que está fuera**.
  - El software arranca siempre en el 01.
- **Potencia.** El límite es **10 mW e.r.p. (10 dBm)**, y el LR02 viene a **22 dBm** (160 mW).
  - No se sabe qué índice de AT+POWE corresponde a cada dBm. Prueba índices en la página POTENCIA y mira qué valor dice el módulo.
  - Para pruebas en la mesa, la potencia de fábrica ya es demasiada. Bájala.
- **Ciclo de trabajo.** Como máximo el 10 %. Los dos extremos espacian sus tramas solos para cumplirlo.
  - Con SF7 sale una trama cada ~1 s.
  - Con SF12, una cada ~27 s.
- **Sin RSSI ni SNR.** El LR02 en modo transparente no los da. La calidad del enlace se mide **solo por las pérdidas**, igual que se juzga una conversación por cuántas frases hay que repetir.
- **No es la radio final.** El vuelo real usará SX1262 a 868 MHz. Esto sirve para aprender el emparejamiento y validar el protocolo, el menú y la estación.

## Lo que no he podido verificar sin hardware

| Duda | Qué hace el código | Si falla |
|---|---|---|
| Formato de los comandos de configuración: `AT+LEVEL5` o `AT+LEVEL=5` | Prueba los dos | `--diag` muestra lo que responde el módulo |
| Si el canal sube 1,4 MHz por paso en el LR02 | Lee la frecuencia real de AT+HELP | La página MODULO muestra la verdadera |
| Si M0/M1 deben ir a GND para el modo normal | Los deja al aire | Llévalos a GND |
| Pantalla: SSD1306 de 64×48 en 0x3C | Si no la encuentra, sigue sin ella | Cambia el constructor U8G2 en `main.cpp` |
| Compilación real del ESP32 | Desde aquí no pude descargar la plataforma | Revisada la sintaxis con g++; el núcleo está compilado y probado en PC |

### Problemas típicos

- **«SIN MODULO» o `--diag` no responde.**
  - Revisa que TX y RX estén cruzados y que el GND sea común.
  - El puerto debe ir a 9600 baudios.
  - Si alguien cambió AT+BAUD, el módulo estará a otra velocidad: usa `--baud`.
- **«SIN ESTACION» aunque la estación recibe.** Los informes vuelven a la vez que el nodo emite. Mira el % de pérdidas en la estación. Si el problema sigue, baja de nivel.
- **Aparece «Entry AT» en los datos.** Algún extremo se quedó en modo AT. Reinicia el nodo o vuelve a lanzar la estación: los dos lo detectan y salen solos.

## Pruebas sin hardware (Linux/macOS)

```
pip install pyserial pytest
python3 tools/tests/test_protocolo.py
python3 tools/tests/test_integracion.py
```

- `test_protocolo.py` comprueba las tramas, el separador y que coincidan byte a byte con el C++.
- `test_integracion.py` hace una prueba completa: nodo C++ ↔ emulador ↔ estación, con un 5 % de pérdidas.

También se puede probar a mano:

```
python3 tools/tests/emulador_lr02.py
./tools/tests/host_nodo /dev/pts/X
python ground/estacion.py --port /dev/pts/Y
```

- El emulador da las rutas X e Y.
- `host_nodo` se compila con g++ (ver `host_nodo.cpp`). En él, las teclas u/d/o + Enter hacen de botones.

Resultado de la prueba completa:

- Emparejamiento ✔
- Cambio a nivel 4 con los botones ✔
- Vuelta atrás cuando la estación no sigue ✔
- Rechazo del canal 00 ✔
