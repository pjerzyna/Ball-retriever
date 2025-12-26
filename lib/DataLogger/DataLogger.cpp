#include "DataLogger.h"

// local helper
static void printFileFromFlashImpl(Stream& out, bool fsOk, const String& path) {
  if (!fsOk) { out.println("ERR: LittleFS not mounted"); return; }

  File f = LittleFS.open(path, "r");
  if (!f) { out.println("ERR: file not found"); return; }

  out.println("===FILE_BEGIN===");
  while (f.available()) out.write((uint8_t)f.read());
  out.println("\n===FILE_END===");
  f.close();
}

bool DataLogger::begin(const Config& cfg) {
  _cfg = cfg;

  // RAM buffer
  _max = _cfg.maxSamples;
  if (_max == 0) _max = 1;

  // safety mechanism
  if (_buf) {
    delete[] _buf;
    _buf = nullptr;
  }

  _buf = new Sample[_max];
  if (!_buf) {
    _max = 0;
    _idx = 0;
    _active = false;
    _fsOk = false;
    return false;
  }

  _idx = 0;
  _lastMs = 0;
  _active = _cfg.autoStart;

  _fsOk = LittleFS.begin(false); // false = do not format automatically
  return _fsOk;
}

void DataLogger::start() {
  _idx = 0;
  _lastMs = 0;
  _active = true;
}

void DataLogger::stop() {
  _active = false;
}

bool DataLogger::isActive() const {
  return _active;
}

void DataLogger::tick(uint32_t nowMs,
                           float ax_g, float ay_g, float az_g,
                           float gz_dps,
                           uint32_t pL, uint32_t pR,
                           float vL, float vR) {
  if (!_active) return;
  if (!_buf || _max == 0) return;
  if (_idx >= _max) return;
  if (nowMs - _lastMs < _cfg.periodMs) return;
  _lastMs = nowMs;

  Sample &s = _buf[_idx++];
  s.tMs = nowMs;

  s.ax_g = ax_g;
  s.ay_g = ay_g;
  s.az_g = az_g;
  s.gz_dps = gz_dps;

  s.pL = pL;
  s.pR = pR;
  s.vL = vL;
  s.vR = vR;
}

// new logs unique names
String DataLogger::makeLogPath() {
  int id = 0;

  File root = LittleFS.open("/");
  if (root) {
    File f = root.openNextFile();
    while (f) {
      String name = f.name();
      int p1 = name.indexOf("log_");
      int p2 = name.lastIndexOf(".csv");
      if (p1 >= 0 && p2 > p1) {
        int n = name.substring(p1 + 4, p2).toInt();
        if (n >= id) id = n + 1;
      }
      f = root.openNextFile();
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "/log_%04d.csv", id);
  return String(buf);
}

// "s" command or manual save
bool DataLogger::saveToFlash() {
  if (!_fsOk) return false;
  if (!_buf) return false;

  String path = makeLogPath();
  File f = LittleFS.open(path, "w");
  if (!f) return false;

  f.println("t_ms,ax_g,ay_g,az_g,gz_dps,pL,pR,vL,vR");
  for (size_t i = 0; i < _idx; i++) {
    const auto &s = _buf[i];
    f.printf("%lu,%.6f,%.6f,%.6f,%.6f,%lu,%lu,%.4f,%.4f\n",
             (unsigned long)s.tMs,
             s.ax_g, s.ay_g, s.az_g, s.gz_dps,
             (unsigned long)s.pL, (unsigned long)s.pR,
             s.vL, s.vR);
  }

  f.close();
  return true;
}

// "l" command
void DataLogger::listLogs(Stream& out) {
  if (!_fsOk) { out.println("ERR: LittleFS not mounted"); return; }

  File root = LittleFS.open("/");
  if (!root) { out.println("ERR: open root failed"); return; }

  out.println("===FILES===");
  File f = root.openNextFile();
  while (f) {
    out.print(f.name());
    out.print("  size=");
    out.println((unsigned long)f.size());
    f = root.openNextFile();
  }
  out.println("===FILES_END===");
}

// "A" command
void DataLogger::dumpAllLogs(Stream& out) {
  if (!_fsOk) { out.println("ERR: LittleFS not mounted"); return; }

  File root = LittleFS.open("/");
  if (!root) { out.println("ERR: open root failed"); return; }

  File f = root.openNextFile();
  while (f) {
    String name = f.name();
    size_t size = f.size();
    f.close();

    out.print("===FILE_NAME==="); out.println(name);
    out.print("===FILE_SIZE==="); out.println((unsigned long)size);

    String path = name.startsWith("/") ? name : ("/" + name);
    printFileFromFlashImpl(out, _fsOk, path);

    out.println("===FILE_DONE===");
    f = root.openNextFile();
  }

  out.println("===ALL_DONE===");
}

// "x" command
void DataLogger::eraseAllLogs(Stream& out) {
  if (!_fsOk) { out.println("ERR: LittleFS not mounted"); return; }

  File root = LittleFS.open("/");
  if (!root) { out.println("ERR: open root failed"); return; }

  int removed = 0;
  File f = root.openNextFile();
  while (f) {
    String name = f.name();
    f.close();

    String path = name.startsWith("/") ? name : ("/" + name);

    if (LittleFS.remove(path)) {
      removed++;
      out.print("DEL: "); out.println(path);
    } else {
      out.print("ERR: can't delete "); out.println(path);
    }

    f = root.openNextFile();
  }

  out.print("OK: removed ");
  out.println(removed);
}
