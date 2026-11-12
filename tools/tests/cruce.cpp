// Imprime tramas y cálculos del lado C++ para compararlos con estacion/enlace.py
#include <cstdio>
#include "sim_vuelo.h"
using namespace enlace;

template <typename T> void dump(const char *name, const T &f) {
  uint8_t b[64]; size_t n = pack(f, b);
  printf("%s ", name);
  for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
  printf("\n");
}

int main() {
  FlightSim s;
  for (int i = 0; i <= 900; i++) s.update(i * 0.1f);        // 90 s: en pleno circuito
  PlaneFrame p; s.fillPlane(p, 7); dump("plane", p);
  ImuFrame m; s.fillImu(m, 8); dump("imu", m);
  ReportFrame r{}; r.type = T_REPORT; r.last_seq = 9; r.rx = 300; r.lost = 12; r.bad = 3; r.cfg_id = 4; seal(r); dump("report", r);
  ChangeFrame c{}; c.type = T_CHANGE; c.change_id = 5; c.channel = 1; c.level = 3; seal(c); dump("change", c);
  AckFrame a{}; a.type = T_ACK; a.change_id = 5; a.accepted = 1; seal(a); dump("ack", a);
  for (int sf = 5; sf <= 12; sf++)
    printf("air %d %.6f %.6f %.4f\n", sf, airtimeS(42, sf), airtimeS(14, sf, 2, 8, true), rangeKm(10, sf, 434.4f, 100, 2));
  printf("band %d %d %d\n", inBand(channelMHz(0)), inBand(channelMHz(1)), inBand(channelMHz(2)));
}
