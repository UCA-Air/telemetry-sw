// UCA&Air — Nodo de vuelo: emisión, estadísticas del enlace y cambio coordinado
//
// Portable (sin Arduino): el mismo código corre en el ESP32-C3 y en el ordenador
// para las pruebas con el emulador del LR02.
#pragma once
#include "enlace_proto.h"
#include "lr02.h"
#include "sim_vuelo.h"

namespace enlace {

struct LinkCfg {
  int channel = 1;          // 434,4 MHz: el único canal del LR02 dentro de 433,05–434,79 MHz
  int level = 5;            // SF7: el más rápido con alcance razonable
  int powerIdx = -1;        // −1: la que tenga el módulo
  bool operator==(const LinkCfg &o) const { return channel == o.channel && level == o.level; }
};
// Canal de encuentro: si se pierde el enlace, los dos extremos vuelven aquí
static const LinkCfg RENDEZVOUS = LinkCfg();

struct Storage {
  virtual bool load(LinkCfg &c) = 0;
  virtual void save(const LinkCfg &c) = 0;
  virtual ~Storage() {}
};

class Node {
 public:
  enum Phase { IDLE, PROPOSE, CONFIRM };

  Node(Port &p, Clock &c, Storage &s) : port_(p), clk_(c), store_(s), lr_(p, c) {}

  // ---------------------------------------------------------------- estado para la pantalla
  ModuleInfo info;
  LinkCfg cfg;
  FlightSim sim;
  bool txEnabled = true;
  uint32_t sent = 0;                  // tramas enviadas en la configuración actual
  bool haveReport = false;
  uint32_t reportAt = 0;
  uint16_t rRx = 0, rLost = 0, rBad = 0;
  uint32_t reportsRx = 0;
  Phase phase = IDLE;
  char status[20] = "";
  uint32_t statusAt = 0;
  bool moduleOk = false;

  void begin() {
    LinkCfg saved;
    if (!store_.load(saved)) saved = RENDEZVOUS;
    applyLocal(saved, "ARRANCANDO");
    next_ = clk_.ms() + 500;
  }

  // ---------------------------------------------------------------- acciones de la interfaz
  // Cambio coordinado de canal y/o nivel (la estación tiene que seguirlo)
  void requestChange(int channel, int level) {
    if (phase != IDLE) { setStatus("ESPERA..."); return; }
    LinkCfg want = cfg; want.channel = channel; want.level = level;
    if (want == cfg) return;
    pending_ = want;
    old_ = cfg;
    changeId_++;
    tries_ = 0;
    phase = PROPOSE;
    sendChange();
    setStatus("PROPONIENDO");
  }
  // La potencia solo afecta al emisor: se cambia sin avisar a la estación
  void setPower(int idx) {
    LinkCfg c = cfg; c.powerIdx = idx;
    applyLocal(c, "POTENCIA");
    if (moduleOk) store_.save(cfg);
  }

  // ---------------------------------------------------------------- cifras derivadas
  float lossPct() const { int t = rRx + rLost; return t ? 100.0f * rLost / t : 0; }
  uint32_t reportAge() const { return haveReport ? clk_.ms() - reportAt : 0xFFFFFFFF; }
  bool stationLost() const { return !haveReport || reportAge() > lostAfterMs(); }
  int sf() const { return info.sf > 0 ? info.sf : levelSF(cfg.level); }
  float airtimeOf(size_t len) const { return airtimeS((int)len + 2, sf(), info.cr, info.preamble, info.crc); }
  // Intervalo entre tramas para no pasar del 10 % de ciclo de trabajo (con 20 % de margen)
  uint32_t intervalFor(size_t len) const {
    uint32_t t = (uint32_t)(airtimeOf(len) * 1.2f / DUTY_LIMIT * 1000);
    return t < 1000 ? 1000 : t;
  }
  uint32_t planeInterval() const { return intervalFor(sizeof(PlaneFrame)); }
  float freqMHz() const { return info.freqHz ? info.freqHz / 1e6f : channelMHz(cfg.channel); }
  uint32_t lostAfterMs() const { uint32_t t = planeInterval() * 4 + 6000; return t < 10000 ? 10000 : t; }

  // ---------------------------------------------------------------- bucle
  void loop() {
    uint32_t now = clk_.ms();
    // 1) lo que llega por radio
    while (port_.available()) {
      int c = port_.read();
      if (c < 0) break;
      uint8_t fr[64]; size_t n;
      if (def_.push((uint8_t)c, fr, n)) onFrame(fr, n);
    }
    now = clk_.ms();          // onFrame puede haber tardado segundos (reconfiguración del módulo)
    // 2) propuesta de cambio pendiente
    if (phase == PROPOSE && now - proposedAt_ > ackTimeout()) {
      if (++tries_ < 3) sendChange();
      else { phase = IDLE; setStatus("SIN RESPUESTA"); }
    }
    // 3) confirmación tras el cambio: si la estación no aparece, vuelta atrás
    if (phase == CONFIRM && now - switchedAt_ > confirmTimeout()) {
      phase = IDLE;
      applyLocal(old_, "VUELTA ATRAS");
    }
    now = clk_.ms();
    // 4) red de seguridad: un minuto sin noticias → canal de encuentro
    if (phase == IDLE && !(cfg == RENDEZVOUS) && now - lastHeard_ > 60000 && now - switchedAt_ > 60000) {
      LinkCfg r = RENDEZVOUS; r.powerIdx = cfg.powerIdx;
      applyLocal(r, "CANAL ENCUENTRO");
    }
    now = clk_.ms();
    // 5) emisión de datos
    sim.update(now / 1000.0f);
    if (txEnabled && phase != PROPOSE && moduleOk && (int32_t)(now - next_) >= 0) {
      uint8_t buf[64]; size_t n;
      if (imuNext_) { ImuFrame f; sim.fillImu(f, seq_++); n = pack(f, buf); next_ = now + intervalFor(sizeof f); }
      else          { PlaneFrame f; sim.fillPlane(f, seq_++); n = pack(f, buf); next_ = now + intervalFor(sizeof f); }
      imuNext_ = !imuNext_;
      port_.write(buf, n);
      sent++;
    }
  }

 private:
  Port &port_;
  Clock &clk_;
  Storage &store_;
  Lr02 lr_;
  Deframer def_;
  LinkCfg pending_, old_;
  uint8_t seq_ = 0, changeId_ = 0;
  int tries_ = 0;
  bool imuNext_ = false;
  uint32_t next_ = 0, proposedAt_ = 0, switchedAt_ = 0, lastHeard_ = 0;

  void setStatus(const char *s) {
    strncpy(status, s, sizeof status - 1); status[sizeof status - 1] = 0;
    statusAt = clk_.ms();
    char l[48]; snprintf(l, sizeof l, "[nodo] %s", s); clk_.log(l);
  }

  uint32_t ackTimeout() const {
    // ida (propuesta) + vuelta (acuse) + margen para que la estación procese
    uint32_t t = (uint32_t)((airtimeOf(sizeof(ChangeFrame)) + airtimeOf(sizeof(AckFrame))) * 1000) + 1500;
    return t;
  }
  uint32_t confirmTimeout() const {
    // tiempo para que la estación se reconfigure (~3 s), oiga dos tramas y responda
    return 6000 + planeInterval() * 3 + (uint32_t)(airtimeOf(sizeof(ReportFrame)) * 1000);
  }

  void sendChange() {
    ChangeFrame f{};
    f.type = T_CHANGE; f.change_id = changeId_;
    f.channel = (uint8_t)pending_.channel; f.level = (uint8_t)pending_.level;
    seal(f);
    uint8_t buf[16]; size_t n = pack(f, buf);
    port_.write(buf, n);
    proposedAt_ = clk_.ms();
  }

  // Reconfigura el módulo propio. Bloquea unos 2–3 s (entrar en AT, ajustar, salir).
  void applyLocal(const LinkCfg &c, const char *why) {
    setStatus(why);
    ModuleInfo mi;
    bool ok = lr_.apply(c.channel, c.level, c.powerIdx, mi);
    if (mi.ok) info = mi;
    moduleOk = mi.ok;
    if (ok) cfg = c;
    else if (mi.ok) { cfg.channel = mi.channel(); cfg.level = mi.level; setStatus("ERROR MODULO"); }
    else setStatus("SIN MODULO");
    def_.reset();
    sent = 0; haveReport = false; rRx = rLost = rBad = 0;
    switchedAt_ = lastHeard_ = clk_.ms();
    next_ = clk_.ms() + 300;
  }

  void onFrame(const uint8_t *fr, size_t n) {
    lastHeard_ = clk_.ms();
    if (fr[0] == T_REPORT && n == sizeof(ReportFrame)) {
      ReportFrame r; memcpy(&r, fr, sizeof r);
      haveReport = true; reportAt = clk_.ms(); reportsRx++;
      rRx = r.rx; rLost = r.lost; rBad = r.bad;
      if (phase == CONFIRM) { phase = IDLE; store_.save(cfg); setStatus("CAMBIO OK"); }
    } else if (fr[0] == T_ACK && n == sizeof(AckFrame)) {
      AckFrame a; memcpy(&a, fr, sizeof a);
      if (phase == PROPOSE && a.change_id == changeId_) {
        if (!a.accepted) { phase = IDLE; setStatus("RECHAZADO"); return; }
        // La estación ya está cambiando: esperamos a que termine de emitir y cambiamos
        clk_.sleep(200);
        LinkCfg target = pending_; target.powerIdx = cfg.powerIdx;
        applyLocal(target, "CAMBIANDO");
        phase = (cfg == target) ? CONFIRM : IDLE;
        if (phase == CONFIRM) setStatus("CONFIRMANDO");
      }
    }
  }
};

}  // namespace enlace
