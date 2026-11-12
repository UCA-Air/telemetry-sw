// Nodo de vuelo compilado para el ordenador (Linux/macOS), unido a un puerto serie.
// Sirve para probar el firmware sin ESP32: con el emulador o con un LR02 real
// enchufado por USB-TTL (el ordenador hace de nodo).
//
//   g++ -std=c++17 -O1 -I../firmware/nodo/include host_nodo.cpp -o host_nodo
//   ./host_nodo /dev/pts/5            luego teclas + Enter: u (arriba) d (abajo) o (centro)
//
// Cada segundo, o cuando cambia, imprime la pantalla del nodo entre corchetes.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include "menu.h"

using namespace enlace;

static uint32_t nowMs() {
  using namespace std::chrono;
  static auto t0 = steady_clock::now();
  return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count();
}

struct FdPort : Port {
  int fd;
  uint8_t buf[512]; int n = 0, i = 0;
  explicit FdPort(int f) : fd(f) {}
  void fill() {
    if (i < n) return;
    int r = ::read(fd, buf, sizeof buf);
    n = r > 0 ? r : 0; i = 0;
  }
  int available() override { fill(); return n - i; }
  int read() override { fill(); return i < n ? buf[i++] : -1; }
  void write(const uint8_t *d, size_t len) override {
    size_t off = 0;
    while (off < len) { ssize_t w = ::write(fd, d + off, len - off); if (w > 0) off += w; else usleep(500); }
    tcdrain(fd);
  }
};

struct HostClock : Clock {
  uint32_t ms() override { return nowMs(); }
  void sleep(uint32_t ms) override { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
  void log(const char *s) override { printf("%7.2f LOG %s\n", nowMs() / 1000.0, s); fflush(stdout); }
};

struct FileStorage : Storage {
  std::string path;
  explicit FileStorage(std::string p) : path(std::move(p)) {}
  bool load(LinkCfg &c) override {
    FILE *f = fopen(path.c_str(), "r");
    if (!f) return false;
    bool ok = fscanf(f, "%d %d %d", &c.channel, &c.level, &c.powerIdx) == 3;
    fclose(f);
    return ok;
  }
  void save(const LinkCfg &c) override {
    FILE *f = fopen(path.c_str(), "w");
    if (f) { fprintf(f, "%d %d %d\n", c.channel, c.level, c.powerIdx); fclose(f); }
  }
};

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "uso: %s /dev/ttyX [baudios] [fichero_config]\n", argv[0]); return 1; }
  int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror("open"); return 1; }
  termios t{}; tcgetattr(fd, &t); cfmakeraw(&t);
  speed_t sp = B9600;
  if (argc > 2 && std::string(argv[2]) == "115200") sp = B115200;
  cfsetispeed(&t, sp); cfsetospeed(&t, sp); tcsetattr(fd, TCSANOW, &t);
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

  FdPort port(fd);
  HostClock clk;
  FileStorage store(argc > 3 ? argv[3] : "/tmp/nodo_cfg.txt");
  Node node(port, clk, store);
  Menu menu(node);
  node.begin();

  std::string last;
  uint32_t lastPrint = 0;
  char in[64];
  for (;;) {
    node.loop();
    int r = ::read(0, in, sizeof in);
    for (int k = 0; k < r; k++) {
      if (in[k] == 'u') menu.button(Menu::UP);
      if (in[k] == 'd') menu.button(Menu::DOWN);
      if (in[k] == 'o') menu.button(Menu::OK);
      if (in[k] == 'q') return 0;
    }
    uint32_t now = nowMs();
    menu.render(now - now % 800);              // sin parpadeo en la salida de texto
    std::string s;
    for (int i = 0; i < Menu::ROWS; i++) { s += "["; s += menu.text[i]; s += "]"; }
    if (s != last || now - lastPrint > 1000) {
      printf("%7.2f SCR %s\n", now / 1000.0, s.c_str()); fflush(stdout);
      last = s; lastPrint = now;
    }
    usleep(2000);
  }
}
