# telemetry-sw

**Airborne telemetry box and ground station software for UCA&Air's test flights.**

Software de la caja de telemetría de pruebas y de la estación de tierra de UCA&Air, el
club aeroespacial de la Universidad de Cádiz. Es el banco de pruebas del área: el mismo
equipo sirve para los aviones prototipo y para los cohetes prototipo.

## Por qué los dos extremos están en el mismo repositorio

Porque comparten el **formato de trama**. Si el aire y la tierra viven en repositorios
distintos, el día que alguien añada un campo al paquete los dos lados se desincronizan
y eso se descubre en el campo de vuelo, con el prototipo en la mano. Aquí, un cambio en
`shared/` entra de una vez para los dos.

## Estructura

```
airborne/  Caja de telemetría: lectura de sensores, registro a bordo y envío por radio.
ground/    Estación de tierra: recepción, almacenamiento y panel de seguimiento.
shared/    Formato de trama, identificadores y utilidades comunes. Fuente única de verdad.
tools/     Reproducción de vuelos grabados, análisis y simulador para probar sin volar.
docs/      Notas técnicas y decisiones de diseño.
```

## Hardware de referencia

Según la lista unificada de componentes del proyecto:

| Elemento | Dónde |
|---|---|
| Placa ESP32-S3 con radio LoRa 868 MHz | Caja de telemetría |
| Barómetro, IMU y GNSS | Caja de telemetría |
| Registro en microSD | Caja de telemetría |
| Placa receptora con la misma radio | Estación de tierra |
| Reloj de tiempo real | Estación de tierra: mantiene la hora sin GNSS |
| Sensores de ambiente en tierra | Presión, temperatura y humedad |

**Decisión tomada:** la estación de tierra no lleva GNSS. No lo necesita, y la hora la
mantiene el reloj de tiempo real.

## El panel de seguimiento

Existe un prototipo de panel de control que ya funciona con datos de un simulador:
fases del vuelo, mapa y trayectoria, actitud, calidad del enlace, altitud y velocidades,
batería y temperatura frente a altitud. Vive en `ground/` y es la referencia de lo que
tiene que verse en campo de un vistazo.

## Cómo empezar

```bash
git clone git@github.com:UCA-Air/telemetry-sw.git
cd telemetry-sw
```

El simulador de `tools/` permite trabajar en el panel sin hardware. Es el primer sitio
por el que entrar si te acabas de incorporar.

## Cómo trabajamos

Ramas, commits, revisiones y documentación: **[CONTRIBUTING.md](CONTRIBUTING.md)**.

## Equipo

- Responsable: Ángel
- Apoyo y coordinación del área: Javi
- Queda una plaza por cubrir: si te interesa, dilo en el grupo.

## Licencia

MIT — ver [LICENSE](LICENSE).
