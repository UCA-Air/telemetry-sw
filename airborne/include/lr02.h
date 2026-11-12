// UCA&Air — Controlador del módulo DX-LR02 (ASR6601, 433 MHz) por UART
//
// Funcionamiento del módulo (guía serie DX-LR03 y repositorio dutchpatriot/lora-dx-lr02):
//   - Arranca en modo transparente: lo que entra por el UART sale por radio y viceversa.
//   - "+++\r\n" alterna entre modo transparente y modo AT ("Entry AT" / "Exit AT").
//   - Al salir del modo AT el módulo se reinicia ("Power on") y aplica lo configurado.
//   - AT+HELP devuelve toda la configuración real: es la única fuente fiable.
//
// Portable: solo usa las interfaces Port y Clock, para compilarlo en el ordenador.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace enlace {

struct Port {                       // el UART que va al módulo
  virtual int available() = 0;
  virtual int read() = 0;
  virtual void write(const uint8_t *d, size_t n) = 0;
  virtual ~Port() {}
};
struct Clock {
  virtual uint32_t ms() = 0;
  virtual void sleep(uint32_t ms) = 0;
  virtual void log(const char *s) = 0;   // depuración por USB
  virtual ~Clock() {}
};

struct ModuleInfo {
  bool ok = false;
  char version[16] = "?";
  int mode = -1, level = -1, sf = -1, cr = 2, preamble = 8, powerDbm = -99;
  bool crc = false;
  long freqHz = 0;
  int channel() const { return freqHz ? (int)((freqHz / 1e6 - 433.0) / 1.4 + 0.5) : -1; }
};

class Lr02 {
 public:
  Lr02(Port &p, Clock &c) : port_(p), clk_(c) {}
  const char *error() const { return err_; }

  // Lee la configuración (entra en AT, AT+HELP, sale).
  bool readInfo(ModuleInfo &info) {
    if (!enterAt()) return false;
    bool ok = help(info);
    exitAt();
    return ok;
  }

  // Aplica canal/nivel/potencia (−1 = no tocar), lee la configuración resultante y sale.
  bool apply(int channel, int level, int powerIdx, ModuleInfo &info) {
    if (!enterAt()) return false;
    bool ok = true;
    char v[12];
    command("MODE", "0");                              // siempre transparente
    if (channel >= 0) { snprintf(v, sizeof v, "%02X", channel); ok &= command("CHANNEL", v); }
    if (level >= 0)   { snprintf(v, sizeof v, "%d", level);     ok &= command("LEVEL", v); }
    if (powerIdx >= 0){ snprintf(v, sizeof v, "%d", powerIdx);  ok &= command("POWE", v); }
    help(info);
    exitAt();
    // Lo que manda es lo que el módulo dice tener, no lo que se le pidió
    if (channel >= 0 && info.channel() != channel) { setErr("canal no aplicado"); ok = false; }
    if (level >= 0 && info.level != level) { setErr("nivel no aplicado"); ok = false; }
    return ok;
  }

 private:
  Port &port_;
  Clock &clk_;
  char err_[40] = "";
  char line_[600];

  void setErr(const char *e) { strncpy(err_, e, sizeof err_ - 1); err_[sizeof err_ - 1] = 0; clk_.log(e); }
  void send(const char *s) { port_.write((const uint8_t *)s, strlen(s)); }
  void drain() { while (port_.available()) port_.read(); }

  // Acumula lo que llega hasta ver `token`, o hasta `timeout`, o hasta `quiet` ms sin datos
  bool waitFor(const char *token, uint32_t timeout, uint32_t quiet = 0) {
    size_t n = 0;
    uint32_t t0 = clk_.ms(), last = t0;
    line_[0] = 0;
    while (clk_.ms() - t0 < timeout) {
      while (port_.available()) {
        int c = port_.read();
        if (c < 0) break;
        if (n < sizeof line_ - 1) { line_[n++] = (char)c; line_[n] = 0; }
        last = clk_.ms();
      }
      if (token && strstr(line_, token)) {
        if (!quiet) return true;
        if (clk_.ms() - last >= quiet) return true;
      }
      if (!token && n && clk_.ms() - last >= quiet) return true;
      clk_.sleep(2);
    }
    return token ? (strstr(line_, token) != nullptr) : n > 0;
  }

  bool enterAt() {
    drain();
    for (int attempt = 0; attempt < 3; attempt++) {
      send("+++\r\n");
      waitFor(nullptr, 1200, 150);
      if (strstr(line_, "Entry AT")) return true;
      if (strstr(line_, "Exit AT")) {          // estaba en AT y lo acabamos de sacar
        waitFor("Power on", 3000);
        clk_.sleep(200);
        drain();
        continue;                              // vuelve a pulsar +++ para entrar
      }
      // ¿Ya estaba en AT sin saberlo? Un "AT" responde OK solo en modo AT
      send("AT\r\n");
      if (waitFor("OK", 600)) return true;
      clk_.sleep(300);
    }
    setErr("el modulo no entra en AT");
    return false;
  }

  void exitAt() {
    send("+++\r\n");
    waitFor("Power on", 3000);
    clk_.sleep(150);
    drain();
  }

  // Prueba "AT+NOMBREvalor" (formato del manual) y, si falla, "AT+NOMBRE=valor"
  bool command(const char *name, const char *value) {
    char buf[40];
    snprintf(buf, sizeof buf, "AT+%s%s\r\n", name, value);
    send(buf);
    waitFor(nullptr, 800, 120);
    if (strstr(line_, "OK")) return true;
    snprintf(buf, sizeof buf, "AT+%s=%s\r\n", name, value);
    send(buf);
    waitFor(nullptr, 800, 120);
    if (strstr(line_, "OK")) return true;
    char e[40];
    snprintf(e, sizeof e, "AT+%s rechazado", name);
    setErr(e);
    return false;
  }

  static const char *after(const char *hay, const char *key) {
    const char *p = strstr(hay, key);
    return p ? p + strlen(key) : nullptr;
  }

  bool help(ModuleInfo &in) {
    send("AT+HELP\r\n");
    waitFor("Power", 1500, 120);
    const char *p;
    in = ModuleInfo();
    if ((p = after(line_, "VERSION="))) { sscanf(p, "%15s", in.version); }
    if ((p = after(line_, "MODE:")))   in.mode = atoi(p);
    if ((p = after(line_, "LEVEL:")))  in.level = atoi(p);
    if ((p = after(line_, "Frequency:"))) in.freqHz = atol(p);
    if ((p = after(line_, "Spreading Factor:"))) in.sf = atoi(p);
    if ((p = after(line_, "Coding rate:"))) in.cr = atoi(p);
    if ((p = after(line_, "CRC:")))    in.crc = atoi(p) != 0;
    if ((p = after(line_, "Preamble:"))) in.preamble = atoi(p);
    if ((p = after(line_, "Power:")))  in.powerDbm = atoi(p);
    in.ok = in.freqHz > 0 && in.level >= 0;
    if (!in.ok) setErr("AT+HELP ilegible");
    return in.ok;
  }
};

}  // namespace enlace
