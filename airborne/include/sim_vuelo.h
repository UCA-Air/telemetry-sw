// UCA&Air — Vuelo inventado pero coherente, para probar el enlace sin sensores
//
// Un avión de 16 m/s de crucero: 15 s en tierra, ascenso a 80 m y circuito ovalado
// con viento de 4 m/s. Todas las magnitudes salen de la misma física, así que son
// coherentes entre sí: en un viraje de 25° el factor de carga es 1/cos(25°) = 1,10 g,
// la velocidad de giro es g·tan(φ)/V y la presión baja con la altura según la ISA.
// Los valores se meten en las tramas con su formato real (mismas escalas y rangos).
#pragma once
#include <math.h>
#include <stdint.h>
#include "enlace_proto.h"

namespace enlace {

#ifndef HOME_LAT
#define HOME_LAT 36.53190      // cambia por vuestro campo en platformio.ini
#endif
#ifndef HOME_LON
#define HOME_LON -6.19870
#endif

class FlightSim {
 public:
  // Avanza la simulación hasta el instante t (segundos desde el arranque)
  void update(float t) {
    float dt = t - t_;
    if (dt <= 0) return;
    if (dt > 1.0f) dt = 1.0f;                     // evita saltos si el bucle se para
    t_ = t;
    const float G = 9.81f, V = 16.0f, WN = -2.0f, WE = -3.46f;   // viento de 240° a 4 m/s
    float tf = t - 15.0f;                          // tiempo desde el despegue
    float rollCmd = 0, vzCmd = 0;
    if (tf < 0) { tas_ = 0; }
    else {
      tas_ = tf < 5 ? V * tf / 5 : V;
      float target = tf < 40 ? 80.0f : 70.0f + 12.0f * sinf(tf / 20.0f);
      vzCmd = fmaxf(-3.0f, fminf(3.0f, (target - h_) * 0.4f));
      // Circuito: tramos rectos de 15 s y virajes de 180° con 25° de alabeo
      float cyc = fmodf(tf, 2 * 15.0f + 2 * (float)M_PI * V / (G * tanf(25 * M_PI / 180)));
      float straight = 15.0f, turn = (float)M_PI * V / (G * tanf(25 * M_PI / 180));
      rollCmd = (cyc > straight && cyc < straight + turn) || (cyc > 2 * straight + turn) ? 25.0f : 0.0f;
      if (tf < 8) rollCmd = 0;
    }
    rollRate_ = (rollCmd - roll_) * 1.5f;          // el alabeo no cambia de golpe
    roll_ += rollRate_ * dt;
    vz_ = tas_ > 10 ? vzCmd : 0;
    h_ = fmaxf(0, h_ + vz_ * dt);
    float phi = roll_ * (float)M_PI / 180;
    yawRate_ = tas_ > 5 ? (G * tanf(phi) / tas_) * 180 / (float)M_PI : 0;   // °/s
    hdg_ = fmodf(hdg_ + yawRate_ * dt + 360, 360);
    float hr = hdg_ * (float)M_PI / 180;
    float gn = tas_ * cosf(hr) + (h_ > 0.5f ? WN : 0), ge = tas_ * sinf(hr) + (h_ > 0.5f ? WE : 0);
    if (tas_ < 0.5f) gn = ge = 0;
    n_ += gn * dt; e_ += ge * dt;
    gs_ = sqrtf(gn * gn + ge * ge);
    cog_ = gs_ > 0.3f ? fmodf(atan2f(ge, gn) * 180 / (float)M_PI + 360, 360) : hdg_;
    if (h_ > 0.5f) dist_ += gs_ * dt;
    pitch_ = tas_ > 10 ? atan2f(vz_, tas_) * 180 / (float)M_PI + 4.0f : 0;
    current_ = tas_ < 1 ? 0.4f : (vz_ > 0.5f ? 24.0f : 14.0f) + 2.0f * (1 / cosf(phi) - 1) * 10;
    mah_ += current_ * dt / 3.6f;
    vbat_ = 16.8f - 1.3f * (mah_ / 2200) - current_ * 0.012f;
    // Factor de carga y aceleraciones en ejes del avión (z hacia arriba = +1 g en reposo)
    load_ = 1.0f / cosf(phi);
    float vib = tas_ > 1 ? 0.04f : 0.005f;         // vibración del motor
    ax_ = (tas_ < V && tf >= 0 && tf < 5 ? 0.33f : 0.0f) + vib * noise();
    ay_ = vib * noise();                            // viraje coordinado: sin aceleración lateral
    az_ = load_ + vz_ * 0.0f + vib * noise();
    gmax_ = fmaxf(gmax_, sqrtf(ax_ * ax_ + ay_ * ay_ + az_ * az_));
  }

  void fillPlane(PlaneFrame &f, uint8_t seq) {
    double lat, lon;                                // double: un float pierde ~0,3 m en latitud
    toLatLon(lat, lon);
    f = PlaneFrame();
    f.type = T_PLANE; f.seq = seq;
    f.lat_e7 = (int32_t)llround(lat * 1e7); f.lon_e7 = (int32_t)llround(lon * 1e7);
    f.alt_baro_dm = (int16_t)lroundf(h_ * 10 + 1.5f * noise());
    f.alt_gps_m = (int16_t)lroundf(h_ + 12 + 1.5f * noise());
    f.gs_cms = (uint16_t)lroundf(gs_ * 100);
    f.cog_cdeg = (uint16_t)(lroundf(cog_ * 100) % 36000);
    f.roll_cdeg = (int16_t)lroundf(roll_ * 100);
    f.pitch_cdeg = (int16_t)lroundf(pitch_ * 100);
    f.hdg_cdeg = (uint16_t)(lroundf(hdg_ * 100) % 36000);
    f.vz_cms = (int16_t)lroundf(vz_ * 100);
    f.current_ca = (uint16_t)lroundf(current_ * 100);
    f.vbat_mv = (uint16_t)lroundf(vbat_ * 1000);
    f.mah = (uint16_t)fminf(65535, mah_);
    f.dist_m = (uint16_t)fminf(65535, dist_);
    f.temp_cc = (int16_t)lroundf((22.0f - 0.0098f * h_) * 100);
    f.sats = (uint8_t)((12 & 0x3F) | (2 << 6));
    f.flags = 0x3F;                                 // todos los "sensores" bien (son inventados)
    seal(f);
  }

  void fillImu(ImuFrame &f, uint8_t seq) {
    f = ImuFrame();
    f.type = T_IMU; f.seq = seq;
    f.ax_mg = (int16_t)lroundf(ax_ * 1000); f.ay_mg = (int16_t)lroundf(ay_ * 1000); f.az_mg = (int16_t)lroundf(az_ * 1000);
    f.gx_ddps = (int16_t)lroundf(rollRate_ * 10);
    f.gy_ddps = (int16_t)lroundf((load_ - 1) * 9.81f / fmaxf(tas_, 5) * 180 / (float)M_PI * 10);
    f.gz_ddps = (int16_t)lroundf(yawRate_ * 10);
    f.gmax_mg = (uint16_t)lroundf(gmax_ * 1000);
    float hm = 10.0f + h_;                          // campo a 10 m sobre el nivel del mar
    f.press_pa = (uint32_t)lroundf(101325.0f * powf(1 - 2.25577e-5f * hm, 5.25588f));
    seal(f);
    gmax_ = 0;
  }

  float alt() const { return h_; }
  float gs() const { return gs_; }
  float roll() const { return roll_; }
  float pitch() const { return pitch_; }
  float hdg() const { return hdg_; }
  float load() const { return load_; }
  float yawRate() const { return yawRate_; }
  void latLon(double &lat, double &lon) const { toLatLon(lat, lon); }

 private:
  float t_ = 0, h_ = 0, tas_ = 0, roll_ = 0, rollRate_ = 0, yawRate_ = 0, vz_ = 0, hdg_ = 250, gs_ = 0, cog_ = 250;
  float n_ = 0, e_ = 0, dist_ = 0, pitch_ = 0, current_ = 0.4f, mah_ = 0, vbat_ = 16.8f, load_ = 1;
  float ax_ = 0, ay_ = 0, az_ = 1, gmax_ = 0;
  uint32_t rng_ = 12345;
  float noise() { rng_ = rng_ * 1664525u + 1013904223u; return ((rng_ >> 8) / 16777216.0f) * 2 - 1; }
  void toLatLon(double &lat, double &lon) const {
    lat = HOME_LAT + (n_ / 6371000.0) * 180.0 / M_PI;
    lon = HOME_LON + (e_ / (6371000.0 * cos(HOME_LAT * M_PI / 180.0))) * 180.0 / M_PI;
  }
};

}  // namespace enlace
