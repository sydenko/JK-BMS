/*
 * ============================================================================
 *  JK24.ino  —  V11.1-XDUMP  |  ESP32-C6-WROOM-1  (COM7)
 *  BLE-монітор JK-BMS + Wi-Fi точка доступу з HTML-дашбордом
 *  C:\Arduino\JK-BMS\JK24\
 *  Ядро ESP32 3.3.12 (новий BLE, NimBLE-обгортка)
 *
 *  НОВЕ У V11.1 (проміжна дослідницька, над V11-DASHBOARD):
 *   - ОДНОРАЗОВИЙ HEX-дамп першого кадру 0x02 (174 байт, офсети зліва)
 *   - Автоматичний пошук орієнтирів (landmarkScan):
 *       DD 79          -> U-середня ~31197 мВ
 *       EB 00 ED 00    -> T1/T2 = 23.5 / 23.7 °C
 *       CE 3A 00 00    -> Cycle Cap = 15054 мА·г
 *       88 13 00 00    -> повна ємність 5.0 А·г
 *       64 00          -> константа 100 (загадка off107)
 *       0B 01          -> кандидат CMOS-температури
 *     Вивід: Serial 115200. Дамп один за бут — для повтору натисни RST.
 *   - Решта: V11-DASHBOARD без змін (AP BMS_24V, /api/json, canvas-графіки)
 *
 *  КАРТА КАДРУ 0x02 (робоча BMS_24V, 8S Li-ion) — стан ДО виправлення:
 *   5        лічильник кадру
 *   6..21    комірки 1..8       uint16 LE, мВ            (✓ працює)
 *   59..60   U батареї (сер.)   uint16 LE, мВ            (✗ ЗСУНЕНА)
 *   73..74   T1                 int16  LE, 0.1°C         (✗ ЗСУНЕНА)
 *   75..76   T2                 int16  LE, 0.1°C         (✗ ЗСУНЕНА)
 *   87..88   U миттєва          uint16 LE, мВ            (✓ xRAY)
 *   91..92   U фільтрована      uint16 LE, мВ            (✓ xRAY)
 *   95..98   СТРУМ              int32  LE, mA, мінус=розряд (✓✓ верифіковано)
 *   103..106 Cycle Cap          uint32 LE, mAh           (✗ ЗСУНЕНО)
 *   107..108 загадка "64 00"    u16                      (✗ ЗСУНЕНО)
 *   136      повільний тикер                             (✓)
 *   173      контрольна сума                            (✓)
 *
 *  Міграція у режим станції (вдома, до роутера) = замінити
 *  WiFi.softAP(...) на WiFi.begin(STAPSK...) — одна функція.
 * ==========================================================================
 */

#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ctype.h>

// ===================== WI-FI (точка доступу) ================================
#define AP_SSID   "BMS_24V"
#define AP_PASS   "aaabbbccc"      // як CNC_sVital — твоя конвенція

// ===================== ВИБІР ЦІЛІ ===========================================
#define BMS_WORK          // робоча  "BMS_24V"     8S  Li-ion
//#define BMS_HOME          // домашня "JK_BD4A17S4P" 16S LiFePO4 (карта інша!)

#ifdef BMS_WORK
  static BLEAddress targetAddr("c8:47:80:40:e1:0d");
  #define TARGET_NAME "BMS_24V (робоча)"
  #define N_CELLS     8
#else
  static BLEAddress targetAddr("c8:47:80:40:89:b3");
  #define TARGET_NAME "JK_BD4A17S4P (домашня)"
  #define N_CELLS     16
#endif

// ===================== КОНСТАНТИ ПРОТОКОЛУ ==================================
#define JK_SERVICE_UUID "ffe0"
#define JK_CHAR_UUID    "ffe1"

static const uint8_t CMD_DEVINFO[20] = {
  0xAA, 0x55, 0x90, 0xEB, 0x97,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x11 };

static const uint8_t CMD_TELEMETRY[20] = {
  0xAA, 0x55, 0x90, 0xEB, 0x96,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x10 };

#define WATCHDOG_MS  6000
#define SCAN_SECONDS 5

// ===================== ЗНІМОК ТЕЛЕМЕТРІЇ ====================================
struct Snapshot {
  bool     valid   = false;
  uint32_t ms      = 0;
  uint16_t cell[16]= {0};
  uint16_t uAvg    = 0;   // офсет 59 (ЗАРАЗ НЕВІРНИЙ — чекаємо дамп)
  uint16_t uFast   = 0;   // офсет 87
  uint16_t uFil    = 0;   // офсет 91
  int32_t  mA      = 0;   // офсет 95 — СТРУМ
  int16_t  t1      = 0;   // офсет 73 (ЗАРАЗ НЕВІРНИЙ)
  int16_t  t2      = 0;   // офсет 75 (ЗАРАЗ НЕВІРНИЙ)
  uint32_t cyc     = 0;   // офсет 103, mAh (ЗАРАЗ НЕВІРНИЙ)
  uint16_t r107    = 0;   // загадка (ЗАРАЗ НЕВІРНА)
  uint32_t frames  = 0;
  int      rssi    = 0;
};
static Snapshot snap;          // пишеться в loop() -> без гонок з JSON
static bool bleConnected = false;

// ===================== BLE СТАН ==============================================
static BLEClient*      pClient = nullptr;
static BLERemoteCharacteristic* jkChar = nullptr;

static uint8_t  acc[4096];
static size_t   accLen   = 0;
static volatile uint32_t lastDataRx = 0;
static uint32_t frameCnt = 0;

// ===================== ДРУК БАЙТІВ / ЧИТАННЯ ================================
uint16_t rdU16(const uint8_t* f, size_t o) { return f[o] | ((uint16_t)f[o+1] << 8); }
int16_t  rdI16(const uint8_t* f, size_t o) { return (int16_t)rdU16(f, o); }
int32_t  rdI32(const uint8_t* f, size_t o) {
  return (int32_t)((uint32_t)f[o] | ((uint32_t)f[o+1]<<8) |
                   ((uint32_t)f[o+2]<<16) | ((uint32_t)f[o+3]<<24));
}

void printHexLine(const uint8_t* d, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02X ", d[i]);
}

// ===================== ДІАГНОСТИКА: ДАМП + ОРІЄНТИРИ ========================
static bool dumpDone = false;   // дамп — один раз за бут; повтор = RST

void dumpFrame0x02(const uint8_t* f, size_t len) {
  Serial.printf("\n===== DUMP кадру 0x02 | len=%u =====\n", (unsigned)len);
  for (size_t i = 0; i < len; i += 16) {
    Serial.printf("%03u: ", (unsigned)i);
    for (size_t j = 0; j < 16; j++) {
      if (i + j < len) Serial.printf("%02X ", f[i + j]);
      else             Serial.print("   ");
    }
    Serial.println();
  }
  Serial.println("===== КІНЕЦЬ DUMP =====");
}

void landmarkScan(const uint8_t* f, size_t len) {
  struct Pat { const char* name; uint8_t n; uint8_t b[4]; };
  const Pat pats[] = {
    { "U-avg   (DD 79)",         2, {0xDD, 0x79, 0, 0} },
    { "T1/T2   (EB 00 ED 00)",   4, {0xEB, 0x00, 0xED, 0x00} },
    { "Cycle   (CE 3A 00 00)",   4, {0xCE, 0x3A, 0x00, 0x00} },
    { "FullCap (88 13 00 00)",   4, {0x88, 0x13, 0x00, 0x00} },
    { "конст   (64 00)",         2, {0x64, 0x00, 0, 0} },
    { "CMOS    (0B 01)",         2, {0x0B, 0x01, 0, 0} },
  };
  Serial.println("----- ОРІЄНТИРИ (офсети від 0x55 заголовка) -----");
  for (const Pat& p : pats) {
    Serial.printf("%-24s:", p.name);
    int hits = 0;
    for (size_t i = 0; i + p.n <= len; i++) {
      bool ok = true;
      for (uint8_t k = 0; k < p.n; k++)
        if (f[i + k] != p.b[k]) { ok = false; break; }
      if (ok) {
        hits++;
        if (p.n == 2)
          // для дворічних патернів показуємо ще 2 наступні байти,
          // щоб відрізнити справжній влучання від випадкового
          Serial.printf("  +%u [%02X %02X | %02X %02X]",
                        (unsigned)i, f[i], f[i+1], f[i+2], f[i+3]);
        else
          Serial.printf("  +%u", (unsigned)i);
      }
    }
    if (!hits) Serial.print("  НЕ ЗНАЙДЕНО");
    Serial.println();
  }
  Serial.println("--------------------------------------------------");
}

// ===================== TX ====================================================
void txFrame(const uint8_t* frame, size_t len, const char* tag) {
  if (!jkChar) return;
  Serial.printf("[TX] %s\n", tag);
  jkChar->writeValue((uint8_t*)frame, len, /*response=*/false);
}

// ===================== ПАРСЕР КАДРІВ ========================================
int findHeader(const uint8_t* buf, size_t len) {
  if (len < 4) return -1;
  for (size_t i = 0; i + 4 <= len; i++) {
    if (buf[i] == 0x55 && buf[i+1] == 0xAA &&
        buf[i+2] == 0xEB && buf[i+3] == 0x90) return (int)i;
  }
  return -1;
}

// --- декодер кадру 0x02 (тільки робоча 8S карта!) ---
// УВАГА: офсети 59/73/75/103/107 — СТАРІ, після дампу будуть виправлені (V11.2)
void parse0x02(const uint8_t* f, size_t len) {
  if (len < 174) return;
  snap.valid  = true;
  snap.ms     = millis();
  for (int i = 0; i < N_CELLS; i++) snap.cell[i] = rdU16(f, 6 + 2*i);
  snap.uAvg   = rdU16(f, 59);
  snap.t1     = rdI16(f, 73);
  snap.t2     = rdI16(f, 75);
  snap.uFast  = rdU16(f, 87);
  snap.uFil   = rdU16(f, 91);
  snap.mA     = rdI32(f, 95);
  snap.cyc    = (uint32_t)rdI32(f, 103);
  snap.r107   = rdU16(f, 107);
}

void drainFrames() {
  while (accLen >= 4) {
    int s = findHeader(acc, accLen);
    if (s == -1) {
      size_t keep = (accLen > 3) ? 3 : accLen;
      if (accLen > keep) memmove(acc, acc + accLen - keep, keep);
      accLen = keep;
      return;
    }
    if (s > 0) { memmove(acc, acc + s, accLen - s); accLen -= s; }

    int n = findHeader(acc + 4, accLen - 4);
    if (n < 0) {
      if (accLen > 1024) { memmove(acc, acc + 4, accLen - 4); accLen -= 4; }
      return;
    }
    size_t frameLen = 4 + (size_t)n;
    frameCnt++;
    lastDataRx = millis();

    while (frameLen >= 4 && acc[frameLen-4]==0x41 && acc[frameLen-3]==0x54 &&
           acc[frameLen-2]==0x0D && acc[frameLen-1]==0x0A) frameLen -= 4;

    uint8_t flag = acc[4];
    if (flag == 0x02) {
      if (!dumpDone) {                    // одноразовий дамп першого 0x02
        dumpFrame0x02(acc, frameLen);
        landmarkScan(acc, frameLen);
        dumpDone = true;
      }
      parse0x02(acc, frameLen);
      snap.frames = frameCnt;
    }

    memmove(acc, acc + frameLen, accLen - frameLen);
    accLen -= frameLen;
  }
}

// ===================== NOTIFY CALLBACK ======================================
void jkNotifyCB(BLERemoteCharacteristic* chr,
                uint8_t* data, size_t len, bool isNotify) {
  (void)chr; (void)isNotify;

  // серцебиття AT\r\n — пропускаємо
  if (len == 4 && data[0]==0x41 && data[1]==0x54 &&
                  data[2]==0x0D && data[3]==0x0A) return;
  size_t off = 0;
  while (off + 4 <= len && data[off]==0x41 && data[off+1]==0x54 &&
         data[off+2]==0x0D && data[off+3]==0x0A) off += 4;

  size_t payload = len - off;
  if (payload > 0) {
    if (accLen + payload > sizeof(acc)) accLen = 0;
    memcpy(acc + accLen, data + off, payload);
    accLen += payload;
  }
}

// ===================== BLE: СКАН + ПІДКЛЮЧЕННЯ ==============================
bool scanForTarget() {
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setActiveScan(true);
  BLEScanResults* results = pScan->start(SCAN_SECONDS, false);
  for (int i = 0; i < results->getCount(); i++) {
    BLEAdvertisedDevice d = results->getDevice(i);
    if (memcmp(d.getAddress().getNative(), targetAddr.getNative(), 6) == 0) {
      snap.rssi = d.getRSSI();
      Serial.printf("[SCAN] Ціль: RSSI=%d\n", snap.rssi);
      return true;
    }
  }
  return false;
}

bool connectAndArm() {
  Serial.printf("[BLE] Підключення до %s ...\n", targetAddr.toString().c_str());
  pClient = BLEDevice::createClient();
  if (!pClient->connect(targetAddr)) {
    Serial.println("[BLE] Connect FAIL");
    delete pClient; pClient = nullptr;
    return false;
  }
  Serial.println("[BLE] Connected");
  bleConnected = true;

  pClient->setMTU(83);
  Serial.printf("[MTU] negotiated = %u\n", pClient->getMTU());

  BLERemoteService* svc = pClient->getService(JK_SERVICE_UUID);
  if (!svc) { Serial.println("[ERR] Сервіс ffe0 не знайдено!"); return false; }
  jkChar = svc->getCharacteristic(JK_CHAR_UUID);
  if (!jkChar) { Serial.println("[ERR] Характеристика ffe1 не знайдена!"); return false; }

  jkChar->registerForNotify(jkNotifyCB);
  delay(300);
  txFrame(CMD_DEVINFO,  sizeof(CMD_DEVINFO),  "0x97 devinfo");
  delay(300);
  txFrame(CMD_TELEMETRY, sizeof(CMD_TELEMETRY), "0x96 tele");
  lastDataRx = millis();
  return true;
}

void ensureConnected() {
  while (!connectAndArm()) {
    bleConnected = false;
    delay(3000);
    while (!scanForTarget()) delay(3000);
  }
}

// ===================== HTML СТОРІНКА (canvas, без CDN) =======================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="uk"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>JK-BMS 24V</title>
<style>
:root{--p:#6d4aff;--bg:#14141c;--card:#1e1e2a;--tx:#e8e8f2;--dim:#8888a0}
*{box-sizing:border-box;margin:0}
body{background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,sans-serif;padding:12px}
h1{font-size:18px;color:var(--p);margin-bottom:10px;display:flex;gap:10px;align-items:center}
.dot{width:10px;height:10px;border-radius:50%;background:#c33;display:inline-block}
.ok{background:#3c3}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-bottom:12px}
.kpi{background:var(--card);border-radius:8px;padding:10px 12px}
.kpi b{display:block;font-size:20px;color:#fff;margin-top:2px}
.kpi span{font-size:11px;color:var(--dim)}
.panel{background:var(--card);border-radius:8px;padding:10px;margin-bottom:12px}
.panel h2{font-size:13px;color:var(--dim);font-weight:400;margin-bottom:6px}
canvas{width:100%;height:140px;display:block}
#hm{height:120px}
.neg{color:#ff8080}.pos{color:#8ce99a}
table{width:100%;border-collapse:collapse;font-size:12px}
td{padding:2px 6px;border-bottom:1px solid #2a2a3a}
td:last-child{text-align:right}
</style></head><body>
<h1><span class="dot" id="led"></span> JK-BMS 24V — 8S <span id="rssi" style="font-size:12px;color:var(--dim)"></span></h1>
<div class="grid">
 <div class="kpi"><span>Напруга</span><b id="u">—</b></div>
 <div class="kpi"><span>Струм</span><b id="i">—</b></div>
 <div class="kpi"><span>Δ комірок</span><b id="d">—</b></div>
 <div class="kpi"><span>T1 / T2</span><b id="t">—</b></div>
 <div class="kpi"><span>Cycle Cap</span><b id="cyc">—</b></div>
 <div class="kpi"><span>R? (off107)</span><b id="r">—</b></div>
</div>
<div class="panel"><h2>Комірки, мВ (останній кадр)</h2><canvas id="bars"></canvas></div>
<div class="panel"><h2>Струм, А (історія)</h2><canvas id="cur"></canvas></div>
<div class="panel"><h2>Напруга: миттєва / середня, В (історія)</h2><canvas id="volt"></canvas></div>
<div class="panel"><h2>Теплокарта комірок у часі (ліворуч — старіше)</h2><canvas id="hm"></canvas></div>
<script>
const hI=[],hF=[],hA=[];const MAX=240;
const $=id=>document.getElementById(id);
const led=$('led');
function sc(c,w,h){const r=c.getBoundingClientRect();c.width=r.width;c.height=h;return c.getContext('2d')}
function poll(){
 fetch('/api/json').then(r=>r.json()).then(j=>{
  led.className='dot '+(j.on?'ok':'');
  $('rssi').textContent='RSSI '+j.rssi+' dBm · кадр #'+j.frames;
  $('u').textContent=(j.ufast/1000).toFixed(2)+' В';
  const amp=j.i/1000;
  const el=$('i');el.textContent=(amp>0?'+':'')+amp.toFixed(2)+' А';
  el.className=amp>0.005?'pos':(amp<-0.005?'neg':'');
  const cs=j.cells;let mn=Math.min(...cs),mx=Math.max(...cs);
  $('d').textContent=(mx-mn)+' мВ';
  $('t').textContent=(j.t1/10).toFixed(1)+' / '+(j.t2/10).toFixed(1)+' °C';
  $('cyc').textContent=(j.cyc/1000).toFixed(1)+' А·г';
  $('r').textContent=j.r;
  hI.push(amp);hF.push(j.ufast/1000);hA.push(j.uavg/1000);
  if(hI.length>MAX){hI.shift();hF.shift();hA.shift()}
  bars(cs,mn,mx);cur();volt();heat(cs,mn,mx);
 }).catch(()=>{led.className='dot'});
}
function bars(cs,mn,mx){
 const c=$('bars');const x=sc(c,0,140);const W=c.width;
 const pad=Math.max(mn-5,0),top=mx+5,sp=top-pad||1;
 x.clearRect(0,0,W,140);
 const bw=W/cs.length;
 cs.forEach((v,i)=>{
  const hh=(v-pad)/sp*130;
  x.fillStyle='#6d4aff';
  x.fillRect(i*bw+4,135-hh,bw-8,hh);
  x.fillStyle='#bbb';x.font='10px sans-serif';x.textAlign='center';
  x.fillText(v,i*bw+bw/2,135);
 });
}
function cur(){
 const c=$('cur');const x=sc(c,0,140);const W=c.width;
 x.clearRect(0,0,W,140);
 let m=Math.max(1,...hI.map(Math.abs));
 x.strokeStyle='#333';x.beginPath();x.moveTo(0,70);x.lineTo(W,70);x.stroke();
 x.strokeStyle='#e8b4b4';
 x.beginPath();hI.forEach((v,i)=>{const px=i/MAX*W,py=70-v/m*65;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
 x.fillStyle='#888';x.font='10px sans-serif';
 x.fillText('±'+m.toFixed(1)+' А',4,12);
}
function volt(){
 const c=$('volt');const x=sc(c,0,140);const W=c.width;
 x.clearRect(0,0,W,140);
 if(!hF.length)return;
 const mn=Math.min(...hF,...hA)-0.05,mx=Math.max(...hF,...hA)+0.05,sp=mx-mn||1;
 [['#8ce99a',hF],['#7f6cff',hA]].forEach(([col,arr])=>{
  x.strokeStyle=col;x.beginPath();
  arr.forEach((v,i)=>{const px=i/MAX*W,py=135-(v-mn)/sp*125;i?x.lineTo(px,py):x.moveTo(px,py)});
  x.stroke();
 });
 x.fillStyle='#888';x.font='10px sans-serif';
 x.fillText(mn.toFixed(2)+'–'+mx.toFixed(2)+' В',4,12);
}
const hmC=[];
function heat(cs,mn,mx){
 const c=$('hm');const x=sc(c,0,120);const W=c.width;
 hmC.push(cs.slice());if(hmC.length>W)hmC.shift();
 x.fillStyle='#14141c';x.fillRect(0,0,W,120);
 const rh=120/cs.length,sp=(mx-mn)||1;
 hmC.forEach((row,k)=>{
  row.forEach((v,i)=>{
   const n=(v-mn)/sp;
   const hue=260-260*n;   // фіолетовий -> червоний
   x.fillStyle='hsl('+hue+',70%,'+(30+35*n)+'%)';
   x.fillRect(k,i*rh,1,rh);
  });
 });
}
setInterval(poll,1000);poll();
</script></body></html>
)rawliteral";

// ===================== ВЕБ-СЕРВЕР ===========================================
static WebServer server(80);

void handleRoot()  { server.send_P(200, "text/html", INDEX_HTML); }

void handleJson() {
  String j = "{";
  j += "\"on\":"    + String(bleConnected ? 1 : 0);
  j += ",\"frames\":" + String(snap.frames);
  j += ",\"rssi\":"   + String(snap.rssi);
  j += ",\"ufast\":"  + String(snap.uFast);
  j += ",\"uavg\":"   + String(snap.uAvg);
  j += ",\"ufil\":"   + String(snap.uFil);
  j += ",\"i\":"      + String(snap.mA);
  j += ",\"t1\":"     + String(snap.t1);
  j += ",\"t2\":"     + String(snap.t2);
  j += ",\"cyc\":"    + String(snap.cyc);
  j += ",\"r\":"      + String(snap.r107);
  j += ",\"cells\":[";
  for (int i = 0; i < N_CELLS; i++) {
    j += String(snap.cell[i]);
    if (i < N_CELLS - 1) j += ",";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleNotFound() { server.send(404, "text/plain", "404"); }

// ===================== SETUP / LOOP =========================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n==== JK24 V11.1-XDUMP | %s ====\n", TARGET_NAME);

  // --- Wi-Fi AP ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP] SSID %s | http://%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/",     handleRoot);
  server.on("/api/json", handleJson);
  server.onNotFound(handleNotFound);
  server.begin();

  // --- BLE ---
  BLEDevice::init("");
  BLEDevice::setOwnAddrType(0x01);
  while (!scanForTarget()) delay(3000);
  ensureConnected();
}

void loop() {
  server.handleClient();

  if (!pClient || !pClient->isConnected()) {
    Serial.println("[BLE] Розрив! Перепідключення...");
    bleConnected = false;
    jkChar = nullptr;
    if (pClient) { delete pClient; pClient = nullptr; }
    delay(2000);
    while (!scanForTarget()) delay(3000);
    ensureConnected();
    return;
  }

  if ((millis() - lastDataRx) > WATCHDOG_MS) {
    Serial.println("[WD ] Потік молчить > 6 c -> resend 0x96");
    txFrame(CMD_TELEMETRY, sizeof(CMD_TELEMETRY), "0x96 tele (сторож)");
    lastDataRx = millis() - (WATCHDOG_MS / 2);
  }

  drainFrames();
  delay(10);
}