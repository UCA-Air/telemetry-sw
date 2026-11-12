// UCA&Air — Nodo de vuelo v1: ESP32-C3 SuperMini + DX-LR02 (433 MHz) + OLED 0,66"
//
// Conexiones (ver README):
//   LR02 TXD → GPIO20      LR02 RXD ← GPIO7       LR02 VCC → 5V    GND → GND
//   (GPIO21 no: el C3 escribe ahí sus mensajes de arranque y el LR02 los emitiría)
//   OLED SDA → GPIO5       OLED SCL → GPIO6       OLED VCC → 3V3   GND → GND
//   Botón ARRIBA → GPIO3   Botón CENTRO → GPIO4   Botón ABAJO → GPIO10   (el otro lado a GND)
//   El botón BOOT de la placa (GPIO9) también hace de CENTRO.

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include "menu.h"

using namespace enlace;

// ---------------------------------------------------------------- pines
#ifndef PIN_LR_RX
#define PIN_LR_RX   20     // entra lo que transmite el LR02 (su TXD)
#endif
#ifndef PIN_LR_TX
#define PIN_LR_TX    7     // sale hacia el LR02 (su RXD)
#endif
#ifndef PIN_SDA
#define PIN_SDA      5
#endif
#ifndef PIN_SCL
#define PIN_SCL      6
#endif
#define PIN_BTN_UP   3
#define PIN_BTN_OK   4
#define PIN_BTN_DOWN 10
#define PIN_BTN_BOOT 9
#define PIN_LED      8     // LED azul de la SuperMini (se enciende a nivel bajo)

HardwareSerial lrSerial(1);
U8G2_SSD1306_64X48_ER_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------------------------------------------------------------- adaptadores
struct SerialPort : Port {
  int available() override { return lrSerial.available(); }
  int read() override { return lrSerial.read(); }
  void write(const uint8_t *d, size_t n) override {
    lrSerial.write(d, n);
    lrSerial.flush();          // que salga en un solo bloque: el LR02 lo emite como un paquete
    digitalWrite(PIN_LED, LOW);
    ledAt = millis();
  }
  uint32_t ledAt = 0;
};

void drawScreen();             // se usa durante las esperas largas

struct ArduinoClock : Clock {
  uint32_t ms() override { return millis(); }
  void sleep(uint32_t ms) override {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) { drawScreen(); delay(ms < 20 ? ms : 20); }
  }
  void log(const char *s) override { Serial.println(s); }
};

struct NvsStorage : Storage {
  Preferences prefs;
  bool load(LinkCfg &c) override {
    prefs.begin("enlace", true);
    bool has = prefs.isKey("ch");
    if (has) { c.channel = prefs.getInt("ch", 1); c.level = prefs.getInt("lv", 5); c.powerIdx = prefs.getInt("pw", -1); }
    prefs.end();
    return has;
  }
  void save(const LinkCfg &c) override {
    prefs.begin("enlace", false);
    prefs.putInt("ch", c.channel); prefs.putInt("lv", c.level); prefs.putInt("pw", c.powerIdx);
    prefs.end();
  }
};

SerialPort port;
ArduinoClock clk;
NvsStorage storage;
Node node(port, clk, storage);
Menu menu(node);
bool oledOk = false;

// ---------------------------------------------------------------- pantalla
void drawScreen() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (!oledOk || now - last < 150) return;
  last = now;
  menu.render(now);
  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tf);
  oled.setDrawColor(1);
  oled.drawBox(0, 0, 64, 8);                               // cabecera en vídeo inverso
  oled.setDrawColor(0);
  oled.drawStr(1, 7, menu.text[0]);
  oled.setDrawColor(1);
  for (int r = 1; r < Menu::ROWS; r++) oled.drawStr(0, 7 + 8 * r, menu.text[r]);
  if (menu.editing) oled.drawFrame(0, 8, 64, 9);           // marca la línea que se edita
  oled.sendBuffer();
}

// ---------------------------------------------------------------- botones
struct Button {
  uint8_t pin; Menu::Btn id;
  bool down = false; uint32_t t = 0, rep = 0;
  void poll(bool repeat) {
    bool now = digitalRead(pin) == LOW;
    uint32_t ms = millis();
    if (now && !down && ms - t > 30) { down = true; t = rep = ms; menu.button(id); }
    else if (!now && down && ms - t > 30) { down = false; t = ms; }
    else if (now && down && repeat && ms - t > 500 && ms - rep > 120) { rep = ms; menu.button(id); }  // mantener = repetir
  }
};
Button bUp{PIN_BTN_UP, Menu::UP}, bDown{PIN_BTN_DOWN, Menu::DOWN}, bOk{PIN_BTN_OK, Menu::OK}, bBoot{PIN_BTN_BOOT, Menu::OK};

// ---------------------------------------------------------------- arranque
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, HIGH);
  const uint8_t buttons[] = {PIN_BTN_UP, PIN_BTN_OK, PIN_BTN_DOWN, PIN_BTN_BOOT};
  for (uint8_t p : buttons) pinMode(p, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.beginTransmission(0x3C);
  oledOk = Wire.endTransmission() == 0;
  if (oledOk) { oled.begin(); oled.setContrast(200); }
  else Serial.println("[nodo] no encuentro la pantalla en 0x3C");

  lrSerial.begin(9600, SERIAL_8N1, PIN_LR_RX, PIN_LR_TX);   // 9600 8N1: valor de fábrica del LR02
  delay(800);                                               // el módulo tarda en arrancar
  Serial.println("[nodo] UCA&Air nodo de vuelo v1");
  node.begin();
}

void loop() {
  node.loop();
  if (millis() - port.ledAt > 60) digitalWrite(PIN_LED, HIGH);   // destello corto en cada emisión
  bUp.poll(menu.editing); bDown.poll(menu.editing); bOk.poll(false); bBoot.poll(false);
  drawScreen();
}
