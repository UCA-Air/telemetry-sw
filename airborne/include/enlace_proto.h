// UCA&Air — Enlace v1 (DX-LR02, 433 MHz): protocolo y cálculos de radio
//
// Portable: no depende de Arduino, así que se compila igual en el ESP32 y en el
// ordenador para las pruebas. Debe coincidir byte a byte con estacion/enlace.py.
//
// Cada trama viaja precedida de una marca de inicio 0xAA 0x55. El LR02 en modo
// transparente entrega los bytes sin separar paquetes, así que la marca, el tipo
// (que fija la longitud) y el CRC permiten encontrar cada trama en el flujo.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

namespace enlace {

// ------------------------------------------------------------------ marcas y tipos
constexpr uint8_t SYNC0 = 0xAA, SYNC1 = 0x55;

constexpr uint8_t T_PLANE   = 0xD2;  // telemetría del avión (misma trama que el protocolo v2)
constexpr uint8_t T_IMU     = 0xD3;  // aceleraciones, giros y presión
constexpr uint8_t T_REPORT  = 0xE1;  // estación → nodo: cómo le llegan los paquetes
constexpr uint8_t T_CHANGE  = 0xE2;  // nodo → estación: propuesta de cambio de canal/nivel
constexpr uint8_t T_ACK     = 0xE3;  // estación → nodo: acepta el cambio

// ------------------------------------------------------------------ CRC-16/MODBUS
inline uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

// ------------------------------------------------------------------ tramas
#pragma pack(push, 1)
struct PlaneFrame {          // 40 B — idéntica a la 0xD2 del protocolo v2
  uint8_t  type, seq;
  int32_t  lat_e7, lon_e7;
  int16_t  alt_baro_dm, alt_gps_m;
  uint16_t gs_cms, cog_cdeg;
  int16_t  roll_cdeg, pitch_cdeg;
  uint16_t hdg_cdeg;
  int16_t  vz_cms;
  uint16_t current_ca, vbat_mv, mah, dist_m;
  int16_t  temp_cc;
  uint8_t  sats, flags;
  uint16_t crc;
};
struct ImuFrame {            // 22 B — nueva: lo que la 0xD2 no lleva
  uint8_t  type, seq;
  int16_t  ax_mg, ay_mg, az_mg;        // aceleración en ejes del avión, milésimas de g
  int16_t  gx_ddps, gy_ddps, gz_ddps;  // velocidad de giro, décimas de °/s (gz = viraje)
  uint16_t gmax_mg;                    // pico de |a| desde la trama anterior
  uint32_t press_pa;                   // presión estática, Pa
  uint16_t crc;
};
struct ReportFrame {         // 12 B — estación → nodo
  uint8_t  type, last_seq;
  uint16_t rx, lost, bad;              // contadores desde el último cambio de canal
  uint8_t  cfg_id, reserved;
  uint16_t crc;
};
struct ChangeFrame {         // 8 B — nodo → estación
  uint8_t  type, change_id, channel, level, reserved;
  uint8_t  pad;
  uint16_t crc;
};
struct AckFrame {            // 6 B — estación → nodo
  uint8_t  type, change_id, accepted, reserved;
  uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(PlaneFrame) == 40, "PlaneFrame");
static_assert(sizeof(ImuFrame) == 22, "ImuFrame");
static_assert(sizeof(ReportFrame) == 12, "ReportFrame");
static_assert(sizeof(ChangeFrame) == 8, "ChangeFrame");
static_assert(sizeof(AckFrame) == 6, "AckFrame");

inline size_t frameLen(uint8_t type) {
  switch (type) {
    case T_PLANE:  return sizeof(PlaneFrame);
    case T_IMU:    return sizeof(ImuFrame);
    case T_REPORT: return sizeof(ReportFrame);
    case T_CHANGE: return sizeof(ChangeFrame);
    case T_ACK:    return sizeof(AckFrame);
    default:       return 0;
  }
}

template <typename T> inline void seal(T &f) {
  f.crc = crc16(reinterpret_cast<const uint8_t *>(&f), sizeof(T) - 2);
}

// Escribe SYNC + trama en buf; devuelve los bytes a enviar.
template <typename T> inline size_t pack(const T &f, uint8_t *buf) {
  buf[0] = SYNC0; buf[1] = SYNC1;
  memcpy(buf + 2, &f, sizeof(T));
  return sizeof(T) + 2;
}

// ------------------------------------------------------------------ separador de tramas
// Se le dan bytes según llegan del UART (en trozos de cualquier tamaño) y devuelve
// tramas completas con CRC correcto. Si algo no cuadra, avanza un byte y vuelve a buscar.
class Deframer {
 public:
  uint32_t badCrc = 0;
  // Devuelve true cuando hay una trama válida en out (longitud en outLen).
  bool push(uint8_t b, uint8_t *out, size_t &outLen) {
    if (n_ < sizeof(buf_)) buf_[n_++] = b;
    else { shift(1); buf_[n_++] = b; }
    while (n_ >= 3) {
      if (buf_[0] != SYNC0 || buf_[1] != SYNC1) { shift(1); continue; }
      size_t L = frameLen(buf_[2]);
      if (!L) { shift(1); continue; }
      if (n_ < L + 2) return false;                       // falta el resto: esperar
      const uint8_t *f = buf_ + 2;
      uint16_t rx = f[L - 2] | (f[L - 1] << 8);
      if (crc16(f, L - 2) == rx) {
        memcpy(out, f, L); outLen = L;
        shift(L + 2);
        return true;
      }
      badCrc++;
      shift(1);
    }
    return false;
  }
  void reset() { n_ = 0; }
 private:
  uint8_t buf_[128];
  size_t n_ = 0;
  void shift(size_t k) { memmove(buf_, buf_ + k, n_ - k); n_ -= k; }
};

// ------------------------------------------------------------------ radio del DX-LR02
// Niveles (AT+LEVEL): todos a 125 kHz, SF = 12 − nivel (guía serie DX-LR03, misma familia).
constexpr float BW_HZ = 125000.0f;
inline int levelSF(int level) { return 12 - level; }

// Canal → frecuencia: 433 MHz + 1,4 MHz × canal (canales 0x00–0x1E). Es una
// estimación para la vista previa: la frecuencia real se lee con AT+HELP.
inline float channelMHz(int ch) { return 433.0f + 1.4f * ch; }

// Tiempo en el aire (nota AN1200.13 de Semtech). cr: 1..4 → 4/5..4/8.
inline float airtimeS(int payload, int sf, int cr = 2, int preamble = 8, bool crc = false) {
  float tsym = powf(2.0f, sf) / BW_HZ;
  int de = (sf >= 11) ? 1 : 0;
  int num = 8 * payload - 4 * sf + 28 + (crc ? 16 : 0);
  int den = 4 * (sf - 2 * de);
  int npay = 8 + ((num > 0) ? ((num + den - 1) / den) * (cr + 4) : 0);
  return (preamble + 4.25f) * tsym + npay * tsym;
}

// Sensibilidad: −174 + 10·log10(BW) + figura de ruido (6 dB) + SNR mínimo del SF
inline float snrMin(int sf) { return -7.5f - 2.5f * (sf - 7); }
inline float sensitivityDbm(int sf) { return -174.0f + 10.0f * log10f(BW_HZ) + 6.0f + snrMin(sf); }

// Alcance estimado en km. Se toma el menor entre espacio libre y tierra plana (dos
// rayos), que es lo que limita cerca del suelo. Margen de 10 dB para desvanecimientos.
inline float rangeKm(float txDbm, int sf, float freqMHz, float hTx, float hRx,
                     float gainsDbi = 4.0f, float marginDb = 10.0f) {
  float pl = txDbm + gainsDbi - sensitivityDbm(sf) - marginDb;     // pérdida admisible
  float dFree = powf(10.0f, (pl - 20.0f * log10f(freqMHz) - 32.44f) / 20.0f);       // km
  float dFlat = powf(10.0f, (pl + 20.0f * log10f(hTx * hRx)) / 40.0f) / 1000.0f;   // km
  return dFree < dFlat ? dFree : dFlat;
}

// Sub-banda de 433 MHz en Europa (ERC/REC 70-03, anexo 1; CNAF en España):
// 433,050–434,790 MHz, 10 mW e.r.p., ciclo de trabajo ≤ 10 %.
constexpr float BAND_LO = 433.050f, BAND_HI = 434.790f;
constexpr float DUTY_LIMIT = 0.10f;
constexpr float ERP_LIMIT_DBM = 10.0f;
inline bool inBand(float fMHz) {
  float half = BW_HZ / 2e6f;
  return (fMHz - half) >= BAND_LO && (fMHz + half) <= BAND_HI;
}

}  // namespace enlace
