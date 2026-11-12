// UCA&Air — Menú del nodo para una pantalla de 64×48 (12 columnas × 6 filas)
//
// Tres botones:  ARRIBA / ABAJO  pasan de página (o cambian el valor al editar)
//                CENTRO          entra a editar, confirma, o actúa en la página
// Al editar, la línea del valor parpadea; CENTRO aplica. Si el valor no ha
// cambiado, CENTRO simplemente sale de la edición.
#pragma once
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "nodo.h"

namespace enlace {

class Menu {
 public:
  enum Btn { UP, DOWN, OK };
  enum Page { P_LINK, P_FREQ, P_LEVEL, P_POWER, P_RANGE, P_FLIGHT, P_IMU, P_GPS, P_MODULE, P_TX, P_COUNT };
  static constexpr int ROWS = 6, COLS = 12;

  explicit Menu(Node &n) : node_(n) {}

  char text[ROWS][COLS + 1];
  bool editing = false;
  int page = P_LINK;

  void button(Btn b) {
    if (!editing) {
      if (b == UP)   page = (page + P_COUNT - 1) % P_COUNT;
      if (b == DOWN) page = (page + 1) % P_COUNT;
      if (b == OK) startEdit();
      return;
    }
    int lo = 0, hi = 0;
    limits(lo, hi);
    if (b == UP)   value_ = value_ >= hi ? lo : value_ + 1;
    if (b == DOWN) value_ = value_ <= lo ? hi : value_ - 1;
    if (b == OK)   commit();
  }

  // Rellena text[][] con la página actual. blink alterna cada medio segundo.
  void render(uint32_t nowMs) {
    for (int r = 0; r < ROWS; r++) text[r][0] = 0;
    static const char *names[P_COUNT] = {"ENLACE", "FRECUENCIA", "NIVEL", "POTENCIA", "ALCANCE",
                                         "VUELO", "IMU", "GPS", "MODULO", "EMISION"};
    line(0, "%d %s", page + 1, names[page]);
    const Node &n = node_;
    bool blink = (nowMs / 400) % 2;
    switch (page) {
      case P_LINK:
        if (!n.moduleOk) { line(1, "SIN MODULO"); line(2, "Revisa TX/RX"); break; }
        if (n.stationLost()) {
          line(1, "SIN ESTACION");
          line(2, "Tx %lu", (unsigned long)n.sent);
        } else {
          line(1, "Perd %4.1f%%", n.lossPct());
          line(2, "Rx %u/%u", n.rRx, n.rRx + n.rLost);
          line(3, "Inf hace %lus", (unsigned long)(n.reportAge() / 1000));
        }
        line(4, "%.1f L%d", n.freqMHz(), n.cfg.level);
        break;
      case P_FREQ: {
        int ch = editing ? value_ : n.cfg.channel;
        float f = editing ? channelMHz(ch) : n.freqMHz();
        if (editing && blink) line(1, " "); else line(1, "%s%.3f", editing ? ">" : "", f);
        line(2, "Canal %02X", ch);
        line(3, inBand(f) ? "Banda OK" : "FUERA BANDA");
        line(4, editing ? "OK=aplicar" : "OK=editar");
        break;
      }
      case P_LEVEL: {
        int lv = editing ? value_ : n.cfg.level;
        int sf = levelSF(lv);
        if (editing && blink) line(1, " "); else line(1, "%sNivel %d", editing ? ">" : "", lv);
        line(2, "SF%d %s", sf, rate(sf));
        float at = airtimeS(42, sf, n.info.cr, n.info.preamble, n.info.crc);
        uint32_t iv = (uint32_t)(at * 1.2f / DUTY_LIMIT * 1000); if (iv < 1000) iv = 1000;
        line(3, "1 tr/%.1fs", iv / 1000.0f);
        line(4, editing ? "OK=aplicar" : "OK=editar");
        break;
      }
      case P_POWER: {
        if (editing) { if (blink) line(1, " "); else line(1, ">Indice %d", value_); line(2, "dBm: al apl."); }
        else { line(1, "%d dBm", n.info.powerDbm); line(2, "%s", n.info.powerDbm > (int)ERP_LIMIT_DBM ? "! >10mW" : "Legal"); }
        line(3, "Max leg 10mW");
        line(4, editing ? "OK=aplicar" : "OK=editar");
        break;
      }
      case P_RANGE: {
        float p = n.info.powerDbm > -50 ? n.info.powerDbm : 22;
        line(1, "Suelo %.1fkm", rangeKm(p, n.sf(), n.freqMHz(), 1.5f, 1.5f));
        line(2, "Avion %.0fkm", rangeKm(p, n.sf(), n.freqMHz(), 100.0f, 2.0f));
        line(3, "SF%d %ddBm", n.sf(), (int)p);
        line(4, "Teorico");
        break;
      }
      case P_FLIGHT:
        line(1, "Alt %5.1fm", n.sim.alt());
        line(2, "GS %4.1fm/s", n.sim.gs());
        line(3, "Al%+3.0f Ca%+3.0f", n.sim.roll(), n.sim.pitch());
        line(4, "Rumbo %03.0f", n.sim.hdg());
        break;
      case P_IMU:
        line(1, "n %4.2fg", n.sim.load());
        line(2, "Viraje%+4.0f", n.sim.yawRate());
        line(3, "(grados/s)");
        line(4, "Datos simul.");
        break;
      case P_GPS: {
        double la, lo; n.sim.latLon(la, lo);
        line(1, "%.5f", la);
        line(2, "%.5f", lo);
        line(3, "12 sat 3D");
        line(4, "Datos simul.");
        break;
      }
      case P_MODULE:
        if (!n.moduleOk) { line(1, "No responde"); line(2, "9600 8N1?"); break; }
        line(1, "LR02 %s", n.info.version);
        line(2, "%.3fMHz", n.info.freqHz / 1e6);
        line(3, "SF%d CR4/%d", n.info.sf, n.info.cr + 4);
        line(4, "%ddBm M%d", n.info.powerDbm, n.info.mode);
        break;
      case P_TX:
        line(1, n.txEnabled ? "Emitiendo" : "EN PAUSA");
        line(2, "Tx %lu", (unsigned long)n.sent);
        line(3, "Cada %.1fs", n.planeInterval() / 1000.0f);
        line(4, n.txEnabled ? "OK=pausar" : "OK=reanudar");
        break;
    }
    // Última fila: mensaje de estado durante 4 s, o la fase del cambio
    if (n.statusAt && nowMs - n.statusAt < 4000) line(5, "%s", n.status);
    else if (n.phase == Node::PROPOSE) line(5, "PROPONIENDO");
    else if (n.phase == Node::CONFIRM) line(5, "CONFIRMANDO");
  }

 private:
  Node &node_;
  int value_ = 0;

  void line(int r, const char *fmt, ...) __attribute__((format(printf, 3, 4))) {
    va_list ap; va_start(ap, fmt);
    vsnprintf(text[r], COLS + 1, fmt, ap);
    va_end(ap);
  }
  static const char *rate(int sf) {
    switch (sf) { case 12: return "244b/s"; case 11: return "447b/s"; case 10: return "813b/s";
                  case 9: return "1.5kb/s"; case 8: return "2.6kb/s"; case 7: return "4.6kb/s";
                  case 6: return "7.8kb/s"; default: return "13kb/s"; }
  }
  void limits(int &lo, int &hi) const {
    lo = 0; hi = 0;
    if (page == P_FREQ)  { lo = 0; hi = 0x1E; }
    if (page == P_LEVEL) { lo = 0; hi = 7; }
    if (page == P_POWER) { lo = 0; hi = 7; }
  }
  void startEdit() {
    if (page == P_TX) { node_.txEnabled = !node_.txEnabled; return; }
    if (page == P_FREQ)  { editing = true; value_ = node_.cfg.channel; }
    if (page == P_LEVEL) { editing = true; value_ = node_.cfg.level; }
    if (page == P_POWER) { editing = true; value_ = node_.cfg.powerIdx < 0 ? 0 : node_.cfg.powerIdx; }
  }
  void commit() {
    editing = false;
    if (page == P_FREQ && value_ != node_.cfg.channel) node_.requestChange(value_, node_.cfg.level);
    if (page == P_LEVEL && value_ != node_.cfg.level) node_.requestChange(node_.cfg.channel, value_);
    if (page == P_POWER && value_ != node_.cfg.powerIdx) node_.setPower(value_);
  }
};

}  // namespace enlace
