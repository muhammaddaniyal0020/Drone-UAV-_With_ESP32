/* ===================================================================
 *  ESP32-S3 DevKitC-1 N16R8 Quadcopter UAV - All-in-One Controller
 * ===================================================================
 *  Board   : ESP32-S3 DevKitC-1 N16R8 (16MB Flash / 8MB PSRAM)
 *  WiFi    : AP Hotspot  SSID: DroneAP-ESP32S3  PASS: drone1234
 *  GCS URL : http://192.168.4.1  (open in browser after WiFi connect)
 *  Control : WebSocket  ws://192.168.4.1/ws
 *
 *  REQUIRED LIBRARIES (Arduino Library Manager):
 *    1. ESPAsyncWebServer  by me-no-dev
 *    2. AsyncTCP           by me-no-dev
 *    3. ArduinoJson        by Benoit Blanchon (v6 or v7)
 *
 *  ARDUINO IDE BOARD SETTINGS:
 *    Board            : ESP32S3 Dev Module
 *    Flash Size       : 16MB (128Mb)
 *    Partition Scheme : Default 4MB with spiffs
 *    PSRAM            : OPI PSRAM
 *    Upload Speed     : 921600
 *
 *  X-FRAME MOTOR LAYOUT (top view, props on):
 *    M4(FL,CCW) -------- M1(FR,CW)    <-- Front
 *        \             /
 *    M3(RL,CW) -------- M2(RR,CCW)    <-- Rear
 *
 *  SAFETY: ALWAYS REMOVE PROPELLERS DURING DEVELOPMENT!
 * =================================================================== */

// =============================================================
// SECTION 1 - INCLUDES
// =============================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>

// =============================================================
// SECTION 2 - PIN DEFINITIONS
// =============================================================
#define PIN_SDA         8       // I2C SDA (MPU-6050 + BMP280)
#define PIN_SCL         9       // I2C SCL
#define PIN_M1          4       // ESC Front-Right (CW)
#define PIN_M2          5       // ESC Rear-Right  (CCW)
#define PIN_M3          6       // ESC Rear-Left   (CW)
#define PIN_M4          7       // ESC Front-Left  (CCW)
#define PIN_VBAT        1       // Battery ADC: VBAT--[47k]--ADC--[10k]--GND
#define VBAT_RATIO      5.70f   // (47k+10k)/10k
#define PIN_LED         48      // On-board LED

// =============================================================
// SECTION 3 - WIFI HOTSPOT CONFIG
// =============================================================
static const char* AP_SSID = "DroneAP-ESP32S3";
static const char* AP_PASS = "drone1234";
static const IPAddress AP_IP    (192,168,4,1);
static const IPAddress AP_GW    (192,168,4,1);
static const IPAddress AP_SUBNET(255,255,255,0);

// =============================================================
// SECTION 4 - MPU-6050 REGISTERS & SCALE
// =============================================================
#define MPU_ADDR        0x68
#define MPU_SMPLRT_DIV  0x19
#define MPU_CONFIG_REG  0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_XOUT  0x3B    // 14 bytes: AX AY AZ TEMP GX GY GZ
#define MPU_PWR_MGMT1   0x6B
#define MPU_WHO_AM_I    0x75
#define GYRO_LSB        65.5f   // LSB per deg/s (+-500 dps)
#define ACCEL_LSB       16384.0f // LSB per g    (+-2g)

// =============================================================
// SECTION 5 - BMP280 REGISTERS
// =============================================================
#define BMP_ADDR        0x76
#define BMP_REG_ID      0xD0
#define BMP_REG_CTRL    0xF4
#define BMP_REG_CFG     0xF5
#define BMP_REG_PRESS   0xF7    // 3 bytes raw pressure
#define BMP_REG_TEMP    0xFA    // 3 bytes raw temperature
#define BMP_REG_CALIB   0x88    // 24 bytes calibration data

// =============================================================
// SECTION 6 - ESC / MOTOR CONFIG
// =============================================================
#define ESC_FREQ_HZ     50
#define ESC_BITS        16
#define ESC_PERIOD_US   20000   // 1/50Hz = 20ms
#define ESC_MIN_US      1000
#define ESC_MAX_US      2000
#define ESC_IDLE_US     1000    // motor-off (disarmed) pulse

// =============================================================
// SECTION 7 - PID INITIAL GAINS (tune these for your frame)
// =============================================================
#define INIT_KP_ROLL    1.40f
#define INIT_KI_ROLL    0.03f
#define INIT_KD_ROLL    18.0f
#define INIT_KP_PITCH   1.40f
#define INIT_KI_PITCH   0.03f
#define INIT_KD_PITCH   18.0f
#define INIT_KP_YAW     3.50f
#define INIT_KI_YAW     0.02f
#define INIT_KD_YAW     0.00f

// =============================================================
// SECTION 8 - FLIGHT PARAMETERS
// =============================================================
#define MAX_ANGLE_DEG   30.0f   // max roll/pitch setpoint (degrees)
#define MAX_YAW_RATE    150.0f  // max yaw rate (deg/s)
#define THROTTLE_IDLE   1050    // minimum armed throttle (us)
#define THROTTLE_MAX    1920    // hard safety cap (us)
#define CF_ALPHA        0.98f   // complementary filter gyro weight
#define FLIGHT_LOOP_US  4000    // 250 Hz flight loop period
#define BARO_DIVIDER    10      // sample baro every N flight loops
#define TELEM_MS        100     // telemetry send period (10 Hz)
#define SIGNAL_TO_MS    2000    // auto-disarm on no-signal timeout
#define PID_OUTPUT_LIM  300.0f  // PID output clamp per axis
#define PID_I_LIMIT     100.0f  // integral anti-windup clamp
#define GYRO_CAL_N      200     // gyro calibration sample count

// =============================================================
// SECTION 9 - EMBEDDED GCS DASHBOARD (HTML/CSS/JS)
//             Served at http://192.168.4.1
// =============================================================
static const char GCS_HTML[] PROGMEM = R"EOHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<meta name="description" content="ESP32-S3 Drone Ground Control Station">
<title>Drone GCS - ESP32-S3</title>
<style>
:root{
  --bg:#07080f;--bg2:#0c0f1e;--card:#10152a;--bdr:#1a2540;
  --cyan:#00e5d4;--blue:#3a8fff;--grn:#00e676;--yel:#ffd740;
  --red:#ff3d57;--txt:#c5cef0;--dim:#4a5475;
  --font:system-ui,-apple-system,'Segoe UI',sans-serif;
  --mono:'Courier New','Lucida Console',monospace;
}
*{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;background:var(--bg);color:var(--txt);font-family:var(--font);overflow-x:hidden}
header{display:flex;align-items:center;justify-content:space-between;
  padding:10px 18px;background:var(--bg2);border-bottom:1px solid var(--bdr);
  position:sticky;top:0;z-index:99;box-shadow:0 2px 20px rgba(0,0,0,.5)}
.logo{display:flex;align-items:center;gap:10px;font-size:1rem;font-weight:700;letter-spacing:.04em}
.rotor{font-size:1.4rem;display:inline-block;animation:spin 2.5s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
.hdr-r{display:flex;align-items:center;gap:12px;font-family:var(--mono);font-size:.72rem}
.pill{padding:3px 11px;border-radius:20px;font-weight:700;font-size:.68rem;
  border:1px solid var(--bdr);background:#111828;transition:all .3s}
.pill.on {border-color:var(--grn);color:var(--grn)}
.pill.off{border-color:var(--red);color:var(--red)}
.pill.arm{border-color:var(--yel);color:var(--yel);background:#1a1400}
.pill.dis{border-color:var(--dim);color:var(--dim)}
.grid{display:grid;grid-template-columns:1fr 175px 1fr;
  grid-template-rows:auto auto auto;gap:10px;padding:10px 14px;
  max-width:1060px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--bdr);border-radius:10px;padding:12px;
  box-shadow:0 4px 20px rgba(0,0,0,.3)}
.ct{font-family:var(--mono);font-size:.62rem;font-weight:700;color:var(--dim);
  letter-spacing:.14em;text-transform:uppercase;margin-bottom:9px}
.ahi-w{position:relative;width:160px;height:160px;margin:0 auto 8px;
  border-radius:50%;overflow:hidden;border:2px solid var(--bdr)}
.drum{position:absolute;left:0;width:100%;height:300%;top:-75%;transition:transform .07s linear}
.sky{height:50%;background:linear-gradient(180deg,#062050,#1660a8)}
.gnd{height:50%;background:linear-gradient(180deg,#5c3a15,#3a2208)}
.hl{position:absolute;top:50%;left:0;width:100%;height:2px;
  background:rgba(255,220,0,.6);transform:translateY(-1px)}
.ret{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
  width:130px;height:2px;pointer-events:none}
.ret::before,.ret::after{content:'';position:absolute;height:2px;background:var(--yel);width:40px;top:0}
.ret::before{right:8px}.ret::after{left:8px}
.dot{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
  width:7px;height:7px;border-radius:50%;background:var(--yel)}
.av{display:grid;grid-template-columns:1fr 1fr 1fr;text-align:center;
  font-family:var(--mono);font-size:.68rem;gap:4px}
.av span{color:var(--dim)}.av b{color:var(--cyan);display:block}
.cmp-hdg{text-align:center;font-family:var(--mono);font-size:.88rem;
  color:var(--cyan);font-weight:700;margin-top:5px}
.vrow{display:flex;gap:14px;align-items:flex-end}
.vg{flex:1;display:flex;flex-direction:column;align-items:center;gap:5px}
.vglbl{font-family:var(--mono);font-size:.6rem;color:var(--dim)}
.vgtrk{width:26px;height:106px;background:#080d1a;border:1px solid var(--bdr);
  border-radius:4px;position:relative;overflow:hidden}
.vgf{position:absolute;bottom:0;width:100%;border-radius:4px;transition:height .2s}
.fc{background:linear-gradient(0deg,#004d45,var(--cyan))}
.fb{background:linear-gradient(0deg,#002860,var(--blue))}
.vgval{font-family:var(--mono);font-size:.75rem}
.m4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px}
.mi{display:flex;flex-direction:column;gap:4px;align-items:center}
.mn{font-family:var(--mono);font-size:.58rem;color:var(--dim)}
.mtrk{width:100%;height:8px;background:#080d1a;border:1px solid var(--bdr);
  border-radius:4px;overflow:hidden}
.mf{height:100%;border-radius:4px;
  background:linear-gradient(90deg,var(--blue),var(--cyan));transition:width .1s}
.mv{font-family:var(--mono);font-size:.7rem}
.brow{display:flex;align-items:center;gap:10px;margin-top:6px}
.bico{font-size:2.2rem}
.btrk{height:12px;background:#080d1a;border:1px solid var(--bdr);
  border-radius:6px;overflow:hidden;margin-bottom:5px;width:100%}
.bf{height:100%;border-radius:6px;transition:width .5s,background .5s}
.bv{font-family:var(--mono);font-size:1rem;font-weight:700}
.bp{font-family:var(--mono);font-size:.7rem;color:var(--dim)}
.ctlrow{display:flex;align-items:center;justify-content:space-around;flex-wrap:wrap;gap:14px}
.jsw{display:flex;flex-direction:column;align-items:center;gap:5px}
.jslbl{font-family:var(--mono);font-size:.6rem;color:var(--dim)}
.jsa{width:128px;height:128px;border-radius:50%;background:#090d1c;
  border:2px solid var(--bdr);position:relative;cursor:grab;touch-action:none;
  box-shadow:inset 0 2px 10px rgba(0,0,0,.5)}
.jsa:active{cursor:grabbing}
.jsk{width:38px;height:38px;border-radius:50%;
  background:radial-gradient(circle at 35% 35%,var(--cyan),#003c35);
  border:2px solid var(--cyan);box-shadow:0 0 10px rgba(0,229,212,.3);
  position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none}
.jcx{position:absolute;top:0;left:50%;width:1px;height:100%;
  background:rgba(255,255,255,.05);transform:translateX(-.5px)}
.jcy{position:absolute;left:0;top:50%;width:100%;height:1px;
  background:rgba(255,255,255,.05);transform:translateY(-.5px)}
.jv{font-family:var(--mono);font-size:.66rem;color:var(--dim);text-align:center}
.jv b{color:var(--cyan)}
.arms{display:flex;flex-direction:column;align-items:center;gap:10px}
.abtn{padding:13px 30px;font-size:.9rem;font-weight:700;border:none;
  border-radius:50px;cursor:pointer;letter-spacing:.07em;
  text-transform:uppercase;font-family:var(--font);transition:all .2s}
.abtn.d{background:linear-gradient(135deg,#003520,#005030);
  border:2px solid var(--grn);color:var(--grn);box-shadow:0 0 16px rgba(0,230,118,.2)}
.abtn.d:hover{box-shadow:0 0 26px rgba(0,230,118,.45);transform:scale(1.04)}
.abtn.a{background:linear-gradient(135deg,#380000,#580000);
  border:2px solid var(--red);color:var(--red);box-shadow:0 0 16px rgba(255,61,87,.2)}
.abtn.a:hover{box-shadow:0 0 26px rgba(255,61,87,.45);transform:scale(1.04)}
.athr{font-family:var(--mono);font-size:.75rem;color:var(--dim)}
.aw{font-family:var(--mono);font-size:.6rem;color:var(--dim);text-align:center;
  max-width:145px;line-height:1.5;padding:6px;border:1px solid #ff3d5722;
  border-radius:6px;background:#200000}
.pid{grid-column:1/4}
details summary{cursor:pointer;font-family:var(--mono);font-size:.66rem;color:var(--dim);
  letter-spacing:.1em;list-style:none;padding:5px 0;user-select:none}
details summary::-webkit-details-marker{display:none}
details summary::before{content:"\25B6  PID TUNING PANEL"}
details[open] summary::before{content:"\25BC  PID TUNING PANEL"}
.pgrid{display:grid;grid-template-columns:55px repeat(9,1fr);
  gap:5px;margin-top:10px;align-items:center}
.prl{font-family:var(--mono);font-size:.64rem;color:var(--cyan);font-weight:700}
.pg{display:flex;flex-direction:column;gap:2px}
.pg label{font-family:var(--mono);font-size:.57rem;color:var(--dim)}
.pg input{width:100%;background:#080d1a;border:1px solid var(--bdr);color:var(--cyan);
  font-family:var(--mono);font-size:.67rem;padding:3px;border-radius:3px;
  text-align:center;outline:none}
.pg input:focus{border-color:var(--cyan)}
.papply{grid-column:span 10;justify-self:end;padding:5px 16px;
  border:1px solid var(--cyan);color:var(--cyan);background:transparent;
  font-family:var(--mono);font-size:.67rem;border-radius:5px;
  cursor:pointer;letter-spacing:.07em}
.papply:hover{background:rgba(0,229,212,.1)}
#lg{background:#060810;border:1px solid var(--bdr);border-radius:7px;
  padding:7px 10px;height:70px;overflow-y:auto;font-family:var(--mono);
  font-size:.61rem;color:var(--dim);grid-column:1/4;margin-top:2px}
.lok{color:var(--grn)}.lerr{color:var(--red)}.linf{color:var(--cyan)}
@keyframes ag{0%,100%{border-bottom-color:var(--bdr)}50%{border-bottom-color:var(--yel)}}
.is-armed header{animation:ag 1.2s ease-in-out infinite}
::-webkit-scrollbar{width:3px}::-webkit-scrollbar-thumb{background:var(--bdr);border-radius:2px}
@media(max-width:660px){
  .grid{grid-template-columns:1fr 1fr}
  .att,.alt{grid-column:1}.cmp{grid-column:2}
  .mot,.bat,.ctl,.pid{grid-column:1/3}#lg{grid-column:1/3}
}
</style>
</head>
<body>
<header>
  <div class="logo">
    <span class="rotor">&#x1F681;</span>
    <span>ESP32-S3 DRONE GCS</span>
  </div>
  <div class="hdr-r">
    <span id="rssi" style="color:var(--dim);font-family:var(--mono);font-size:.7rem">RSSI --dBm</span>
    <span id="cpill" class="pill off">&#x25CF; OFFLINE</span>
    <span id="apill" class="pill dis">DISARMED</span>
  </div>
</header>

<div class="grid">
<!-- ATTITUDE INDICATOR -->
<div class="card att" style="grid-column:1;grid-row:1">
  <div class="ct">Attitude Indicator</div>
  <div class="ahi-w">
    <div class="drum" id="drum">
      <div class="sky"></div>
      <div class="gnd"></div>
      <div class="hl"></div>
    </div>
    <div class="ret"></div>
    <div class="dot"></div>
  </div>
  <div class="av">
    <div><span>ROLL</span><b id="vR">0.0&#xB0;</b></div>
    <div><span>PITCH</span><b id="vP">0.0&#xB0;</b></div>
    <div><span>YAW</span><b id="vY">0.0&#xB0;</b></div>
  </div>
</div>

<!-- COMPASS -->
<div class="card cmp" style="grid-column:2;grid-row:1">
  <div class="ct">Heading</div>
  <svg width="114" height="114" viewBox="0 0 114 114" style="display:block;margin:0 auto">
    <defs>
      <radialGradient id="cg">
        <stop offset="0%" stop-color="#111828"/>
        <stop offset="100%" stop-color="#0a0f1e"/>
      </radialGradient>
    </defs>
    <circle cx="57" cy="57" r="55" fill="url(#cg)" stroke="#1a2540" stroke-width="1.5"/>
    <text x="57" y="13"  text-anchor="middle" fill="#c5cef0" font-size="9" font-family="Courier New,monospace" font-weight="bold">N</text>
    <text x="102" y="61" text-anchor="middle" fill="#4a5475" font-size="9" font-family="Courier New,monospace">E</text>
    <text x="57"  y="107" text-anchor="middle" fill="#4a5475" font-size="9" font-family="Courier New,monospace">S</text>
    <text x="12"  y="61" text-anchor="middle" fill="#4a5475" font-size="9" font-family="Courier New,monospace">W</text>
    <g stroke="#1a2540" stroke-width="1">
      <line x1="57" y1="4" x2="57" y2="17"/>
      <line x1="57" y1="97" x2="57" y2="110"/>
      <line x1="4"  y1="57" x2="17" y2="57"/>
      <line x1="97" y1="57" x2="110" y2="57"/>
    </g>
    <g id="needle" transform="rotate(0,57,57)">
      <polygon points="57,14 51.5,57 57,52 62.5,57" fill="#ff3d57"/>
      <polygon points="57,100 51.5,57 57,62 62.5,57" fill="#4a5475"/>
    </g>
    <circle cx="57" cy="57" r="4" fill="#0a0f1e" stroke="#1a2540"/>
  </svg>
  <div class="cmp-hdg" id="vHdg">000&#xB0;</div>
</div>

<!-- ALTITUDE & TEMP -->
<div class="card alt" style="grid-column:3;grid-row:1">
  <div class="ct">Altitude &amp; Temp</div>
  <div class="vrow">
    <div class="vg">
      <div class="vglbl">ALT</div>
      <div class="vgtrk"><div class="vgf fc" id="altF" style="height:0%"></div></div>
      <div class="vgval" id="vAlt">0.0m</div>
    </div>
    <div class="vg">
      <div class="vglbl">TEMP</div>
      <div class="vgtrk"><div class="vgf fb" id="tmpF" style="height:40%"></div></div>
      <div class="vgval" id="vTmp">25&#xB0;C</div>
    </div>
  </div>
</div>

<!-- MOTORS -->
<div class="card mot" style="grid-column:1/3;grid-row:2">
  <div class="ct">Motor Outputs (&#x03BC;s)</div>
  <div class="m4">
    <div class="mi"><div class="mn">M1 FR CW</div>
      <div class="mtrk"><div class="mf" id="mb1" style="width:0%"></div></div>
      <div class="mv" id="mv1">1000</div></div>
    <div class="mi"><div class="mn">M2 RR CCW</div>
      <div class="mtrk"><div class="mf" id="mb2" style="width:0%"></div></div>
      <div class="mv" id="mv2">1000</div></div>
    <div class="mi"><div class="mn">M3 RL CW</div>
      <div class="mtrk"><div class="mf" id="mb3" style="width:0%"></div></div>
      <div class="mv" id="mv3">1000</div></div>
    <div class="mi"><div class="mn">M4 FL CCW</div>
      <div class="mtrk"><div class="mf" id="mb4" style="width:0%"></div></div>
      <div class="mv" id="mv4">1000</div></div>
  </div>
</div>

<!-- BATTERY -->
<div class="card bat" style="grid-column:3;grid-row:2">
  <div class="ct">Battery (3S LiPo)</div>
  <div class="brow">
    <div class="bico" id="bico">&#x1F50B;</div>
    <div style="flex:1">
      <div class="btrk"><div class="bf" id="bfill" style="width:100%;background:var(--grn)"></div></div>
      <div class="bv" id="vBat">--.-V</div>
      <div class="bp" id="vBpct">-- %</div>
    </div>
  </div>
</div>

<!-- CONTROLS -->
<div class="card ctl" style="grid-column:1/4;grid-row:3">
  <div class="ct">Flight Controls</div>
  <div class="ctlrow">
    <div class="jsw">
      <div class="jslbl">THROTTLE / YAW</div>
      <div class="jsa" id="jsL">
        <div class="jcx"></div><div class="jcy"></div>
        <div class="jsk" id="jkL"></div>
      </div>
      <div class="jv">T:<b id="dT">0%</b> &nbsp; Y:<b id="dY">0%</b></div>
    </div>
    <div class="arms">
      <button class="abtn d" id="armBtn" onclick="toggleArm()">&#x26A1; ARM</button>
      <div class="athr">THR: <span id="thrUs">1000</span>&#x03BC;s</div>
      <div class="aw">&#x26A0; Remove propellers<br>before arming!</div>
    </div>
    <div class="jsw">
      <div class="jslbl">PITCH / ROLL</div>
      <div class="jsa" id="jsR">
        <div class="jcx"></div><div class="jcy"></div>
        <div class="jsk" id="jkR"></div>
      </div>
      <div class="jv">P:<b id="dP">0%</b> &nbsp; R:<b id="dRo">0%</b></div>
    </div>
  </div>
</div>

<!-- PID TUNING -->
<div class="card pid">
  <details>
    <summary></summary>
    <div class="pgrid">
      <span class="prl">ROLL</span>
      <div class="pg"><label>Kp</label><input id="kpR" type="number" value="1.40" step="0.05" min="0" max="10"></div>
      <div class="pg"><label>Ki</label><input id="kiR" type="number" value="0.03" step="0.005" min="0" max="1"></div>
      <div class="pg"><label>Kd</label><input id="kdR" type="number" value="18.0" step="0.5" min="0" max="100"></div>
      <span class="prl">PITCH</span>
      <div class="pg"><label>Kp</label><input id="kpP" type="number" value="1.40" step="0.05" min="0" max="10"></div>
      <div class="pg"><label>Ki</label><input id="kiP" type="number" value="0.03" step="0.005" min="0" max="1"></div>
      <div class="pg"><label>Kd</label><input id="kdP" type="number" value="18.0" step="0.5" min="0" max="100"></div>
      <span class="prl">YAW</span>
      <div class="pg"><label>Kp</label><input id="kpY" type="number" value="3.50" step="0.1" min="0" max="20"></div>
      <div class="pg"><label>Ki</label><input id="kiY" type="number" value="0.02" step="0.005" min="0" max="1"></div>
      <div class="pg"><label>Kd</label><input id="kdY" type="number" value="0.0" step="0.1" min="0" max="20"></div>
      <button class="papply" onclick="sendPID()">APPLY &#x2192;</button>
    </div>
  </details>
</div>
<div id="lg"></div>
</div><!-- /grid -->

<script>
'use strict';
const WS_URL='ws://'+location.host+'/ws';
let ws=null,alive=false,armed=false,cThr=0,cYaw=0,cPit=0,cRol=0,itvl=null;
function connect(){
  lg('inf','Connecting to '+WS_URL);
  ws=new WebSocket(WS_URL);
  ws.onopen=()=>{alive=true;conn(true);lg('ok','Connected');
    if(itvl)clearInterval(itvl);itvl=setInterval(send,80);};
  ws.onclose=()=>{alive=false;conn(false);if(itvl)clearInterval(itvl);
    lg('err','Disconnected - retry 2s');setTimeout(connect,2000);};
  ws.onerror=()=>ws.close();
  ws.onmessage=e=>{try{ui(JSON.parse(e.data));}catch(x){}};
}
function send(){
  if(!alive)return;
  try{ws.send(JSON.stringify({cmd:'ctrl',thr:Math.round(Math.max(0,cThr)),
    roll:+cRol.toFixed(1),pitch:+cPit.toFixed(1),yaw:+cYaw.toFixed(1)}));}catch(e){}
}
function toggleArm(){
  if(!alive){lg('err','Not connected');return;}
  armed=!armed;
  try{ws.send(JSON.stringify({cmd:armed?'arm':'disarm'}));}catch(e){}
  armBtn();lg(armed?'ok':'inf',armed?'ARMED - BE CAREFUL!':'Disarmed');
  if(!armed)cThr=0;
}
function armBtn(){
  const b=document.getElementById('armBtn'),p=document.getElementById('apill');
  if(armed){b.textContent='DISARM';b.className='abtn a';
    p.textContent='ARMED';p.className='pill arm';
    document.body.classList.add('is-armed');}
  else{b.textContent='ARM';b.className='abtn d';
    p.textContent='DISARMED';p.className='pill dis';
    document.body.classList.remove('is-armed');}
}
function sendPID(){
  if(!alive){lg('err','Not connected');return;}
  const g=id=>+document.getElementById(id).value;
  try{ws.send(JSON.stringify({cmd:'pid',kp_r:g('kpR'),ki_r:g('kiR'),kd_r:g('kdR'),
    kp_p:g('kpP'),ki_p:g('kiP'),kd_p:g('kdP'),
    kp_y:g('kpY'),ki_y:g('kiY'),kd_y:g('kdY')}));
    lg('ok','PID gains sent');}catch(e){}
}
function ui(d){
  const roll=d.roll||0,pitch=d.pitch||0,yaw=d.yaw||0;
  document.getElementById('drum').style.transform='rotate('+((-roll).toFixed(1))+'deg) translateY('+(pitch*1.4).toFixed(1)+'px)';
  document.getElementById('vR').textContent=roll.toFixed(1)+'deg';
  document.getElementById('vP').textContent=pitch.toFixed(1)+'deg';
  document.getElementById('vY').textContent=yaw.toFixed(1)+'deg';
  const hdg=((yaw%360)+360)%360;
  document.getElementById('needle').setAttribute('transform','rotate('+hdg.toFixed(0)+',57,57)');
  document.getElementById('vHdg').textContent=hdg.toFixed(0).padStart(3,'0')+'deg';
  const alt=d.alt||0;
  document.getElementById('altF').style.height=Math.min(100,alt/50*100)+'%';
  document.getElementById('vAlt').textContent=alt.toFixed(1)+'m';
  const tmp=d.temp||25;
  document.getElementById('tmpF').style.height=Math.min(100,tmp/80*100)+'%';
  document.getElementById('vTmp').textContent=tmp.toFixed(1)+'C';
  for(let i=1;i<=4;i++){
    const us=d['m'+i]||1000,p=Math.max(0,Math.min(100,(us-1000)/1000*100));
    document.getElementById('mb'+i).style.width=p+'%';
    document.getElementById('mv'+i).textContent=us;
  }
  const v=d.vbat||0,bp=Math.max(0,Math.min(100,(v-9.6)/3.0*100));
  document.getElementById('vBat').textContent=v.toFixed(2)+'V';
  document.getElementById('vBpct').textContent=bp.toFixed(0)+'%';
  const bf=document.getElementById('bfill');
  bf.style.width=bp+'%';
  bf.style.background=bp>40?'var(--grn)':bp>20?'var(--yel)':'var(--red)';
  document.getElementById('bico').textContent=bp>60?'&#x1F50B;':bp>20?'&#x1FAAB;':'&#x26A0;';
  if(d.armed!==undefined&&d.armed!==armed){armed=d.armed;armBtn();}
  if(d.rssi!==undefined)document.getElementById('rssi').textContent='RSSI '+d.rssi+'dBm';
  document.getElementById('thrUs').textContent=Math.round(1000+Math.max(0,cThr)*9.2);
}
function conn(ok){const p=document.getElementById('cpill');
  p.textContent=ok?'ONLINE':'OFFLINE';p.className='pill '+(ok?'on':'off');}
function lg(t,m){const el=document.getElementById('lg'),
  ts=new Date().toTimeString().slice(0,8),cls={ok:'lok',err:'lerr',inf:'linf'}[t]||'';
  el.innerHTML+='<span class="'+cls+'">['+ts+'] '+m+'</span><br>';el.scrollTop=el.scrollHeight;}
function joystick(aId,kId,cb,snapY){
  const area=document.getElementById(aId),knob=document.getElementById(kId);
  let active=false;
  const KS='position:absolute;pointer-events:none;width:38px;height:38px;border-radius:50%;background:radial-gradient(circle at 35% 35%,var(--cyan),#003c35);border:2px solid var(--cyan);box-shadow:0 0 10px rgba(0,229,212,.3);';
  function mv(px,py){
    const r=area.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
    let dx=px-cx,dy=py-cy,d=Math.sqrt(dx*dx+dy*dy),mx=r.width/2-20;
    if(d>mx){dx=dx/d*mx;dy=dy/d*mx;}
    knob.style.cssText=KS+'left:'+(r.width/2+dx)+'px;top:'+(r.height/2+dy)+'px;transform:none';
    cb(dx/mx*100,-dy/mx*100);
  }
  function up(){active=false;
    knob.style.cssText=KS+'top:50%;left:50%;transform:translate(-50%,-50%)';
    cb(0,snapY?0:null);}
  area.addEventListener('pointerdown',e=>{active=true;area.setPointerCapture(e.pointerId);mv(e.clientX,e.clientY);});
  area.addEventListener('pointermove',e=>{if(active)mv(e.clientX,e.clientY);});
  area.addEventListener('pointerup',up);area.addEventListener('pointercancel',up);
}
joystick('jsL','jkL',(x,y)=>{
  if(x!=null){cYaw=x;document.getElementById('dY').textContent=x.toFixed(0)+'%';}
  if(y!=null){cThr=Math.max(0,y);document.getElementById('dT').textContent=cThr.toFixed(0)+'%';}
},false);
joystick('jsR','jkR',(x,y)=>{
  if(x!=null){cRol=x;document.getElementById('dRo').textContent=x.toFixed(0)+'%';}
  if(y!=null){cPit=y;document.getElementById('dP').textContent=y.toFixed(0)+'%';}
},true);
connect();
lg('inf','GCS ready - ESP32-S3 Drone UAV v1.0');
</script>
</body>
</html>
)EOHTML";

// =============================================================
// SECTION 10 - GLOBAL STATE (shared between cores via mutex)
// =============================================================
struct DroneState {
  volatile float roll, pitch, yaw_accum;
  volatile float altitude, temperature, vbat;
  volatile uint16_t m1, m2, m3, m4;
  volatile float sp_roll, sp_pitch, sp_yaw_rate;
  volatile int   sp_throttle;
  volatile bool  armed, gcs_connected;
  volatile unsigned long last_cmd_ms;
  float kp_r, ki_r, kd_r;
  float kp_p, ki_p, kd_p;
  float kp_y, ki_y, kd_y;
};
static DroneState drone;
static SemaphoreHandle_t mtx = NULL;
static struct {
  uint16_t T1; int16_t T2,T3,P2,P3,P4,P5,P6,P7,P8,P9; uint16_t P1;
} bmpC;
static int32_t bmpTFine  = 0;
static float   seaLevelP = 101325.0f;
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

// =============================================================
// SECTION 11 - I2C HELPERS
// =============================================================
static void i2cW(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
static uint8_t i2cR(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(dev,(uint8_t)1); return Wire.available() ? Wire.read() : 0xFF;
}
static void i2cRB(uint8_t dev, uint8_t reg, uint8_t* b, uint8_t n) {
  Wire.beginTransmission(dev); Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom(dev,n);
  for(uint8_t i=0; i<n && Wire.available(); i++) b[i]=Wire.read();
}

// =============================================================
// SECTION 12 - MPU-6050
// =============================================================
static bool mpuInit() {
  i2cW(MPU_ADDR, MPU_PWR_MGMT1, 0x00); delay(100);
  uint8_t who = i2cR(MPU_ADDR, MPU_WHO_AM_I);
  if(who != 0x68 && who != 0x72) return false;
  i2cW(MPU_ADDR, MPU_SMPLRT_DIV, 0x07); // 1kHz sample rate
  i2cW(MPU_ADDR, MPU_CONFIG_REG, 0x03); // DLPF 44Hz
  i2cW(MPU_ADDR, MPU_GYRO_CFG,   0x08); // +-500 dps
  i2cW(MPU_ADDR, MPU_ACCEL_CFG,  0x00); // +-2g
  return true;
}
static void mpuRead(float &ax,float &ay,float &az,float &gx,float &gy,float &gz) {
  uint8_t b[14]; i2cRB(MPU_ADDR, MPU_ACCEL_XOUT, b, 14);
  ax=(int16_t)((b[0]<<8)|b[1])/ACCEL_LSB; ay=(int16_t)((b[2]<<8)|b[3])/ACCEL_LSB;
  az=(int16_t)((b[4]<<8)|b[5])/ACCEL_LSB;
  gx=(int16_t)((b[8]<<8)|b[9])/GYRO_LSB; gy=(int16_t)((b[10]<<8)|b[11])/GYRO_LSB;
  gz=(int16_t)((b[12]<<8)|b[13])/GYRO_LSB;
}

// =============================================================
// SECTION 13 - BMP280 (Bosch compensation formulas)
// =============================================================
static bool bmpInit() {
  uint8_t id = i2cR(BMP_ADDR, BMP_REG_ID);
  if(id != 0x58 && id != 0x60) return false;
  uint8_t c[24]; i2cRB(BMP_ADDR, BMP_REG_CALIB, c, 24);
  bmpC.T1=(uint16_t)(c[1]<<8|c[0]); bmpC.T2=(int16_t)(c[3]<<8|c[2]);
  bmpC.T3=(int16_t)(c[5]<<8|c[4]);  bmpC.P1=(uint16_t)(c[7]<<8|c[6]);
  bmpC.P2=(int16_t)(c[9]<<8|c[8]);  bmpC.P3=(int16_t)(c[11]<<8|c[10]);
  bmpC.P4=(int16_t)(c[13]<<8|c[12]);bmpC.P5=(int16_t)(c[15]<<8|c[14]);
  bmpC.P6=(int16_t)(c[17]<<8|c[16]);bmpC.P7=(int16_t)(c[19]<<8|c[18]);
  bmpC.P8=(int16_t)(c[21]<<8|c[20]);bmpC.P9=(int16_t)(c[23]<<8|c[22]);
  i2cW(BMP_ADDR, BMP_REG_CFG,  0x08);
  i2cW(BMP_ADDR, BMP_REG_CTRL, 0x57);
  delay(50); return true;
}
static void bmpReadRaw(float &pres, float &temp) {
  uint8_t b[6]; i2cRB(BMP_ADDR, BMP_REG_PRESS, b, 6);
  int32_t adcP=((int32_t)b[0]<<12)|((int32_t)b[1]<<4)|(b[2]>>4);
  int32_t adcT=((int32_t)b[3]<<12)|((int32_t)b[4]<<4)|(b[5]>>4);
  int32_t v1=(((adcT>>3)-((int32_t)bmpC.T1<<1))*(int32_t)bmpC.T2)>>11;
  int32_t v2=((((adcT>>4)-(int32_t)bmpC.T1)*((adcT>>4)-(int32_t)bmpC.T1))>>12)*(int32_t)bmpC.T3>>14;
  bmpTFine=v1+v2; temp=(float)((bmpTFine*5+128)>>8)/100.0f;
  int64_t pv1=(int64_t)bmpTFine-128000;
  int64_t pv2=pv1*pv1*(int64_t)bmpC.P6;
  pv2+=(pv1*(int64_t)bmpC.P5)<<17; pv2+=((int64_t)bmpC.P4)<<35;
  pv1=((pv1*pv1*(int64_t)bmpC.P3)>>8)+((pv1*(int64_t)bmpC.P2)<<12);
  pv1=((((int64_t)1<<47)+pv1)*(int64_t)bmpC.P1)>>33;
  if(pv1==0){pres=0;return;}
  int64_t p=1048576-adcP; p=(((p<<31)-pv2)*3125)/pv1;
  pv1=((int64_t)bmpC.P9*(p>>13)*(p>>13))>>25;
  pv2=((int64_t)bmpC.P8*p)>>19;
  p=((p+pv1+pv2)>>8)+((int64_t)bmpC.P7<<4);
  pres=(float)p/256.0f;
}
static float bmpToAlt(float p){return 44330.0f*(1.0f-powf(p/seaLevelP,0.19029f));}

// =============================================================
// SECTION 14 - ESC MOTOR DRIVER (LEDC PWM)
// =============================================================
static uint32_t usToDuty(uint16_t us){
  return (uint32_t)(constrain((float)us,(float)ESC_MIN_US,(float)ESC_MAX_US)
                    /(float)ESC_PERIOD_US*65535.0f);
}
static void motorsInit(){
  ledcAttach(PIN_M1,ESC_FREQ_HZ,ESC_BITS); ledcAttach(PIN_M2,ESC_FREQ_HZ,ESC_BITS);
  ledcAttach(PIN_M3,ESC_FREQ_HZ,ESC_BITS); ledcAttach(PIN_M4,ESC_FREQ_HZ,ESC_BITS);
}
static void motorsWrite(uint16_t m1,uint16_t m2,uint16_t m3,uint16_t m4){
  ledcWrite(PIN_M1,usToDuty(m1)); ledcWrite(PIN_M2,usToDuty(m2));
  ledcWrite(PIN_M3,usToDuty(m3)); ledcWrite(PIN_M4,usToDuty(m4));
}
static void motorsIdle(){motorsWrite(ESC_IDLE_US,ESC_IDLE_US,ESC_IDLE_US,ESC_IDLE_US);}
static float readBattery(){return ((float)analogRead(PIN_VBAT)/4095.0f*3.3f)*VBAT_RATIO;}

// =============================================================
// SECTION 15 - PID CONTROLLER
// =============================================================
struct PIDCtrl {
  float kp,ki,kd,err0,intg;
  void set(float p,float i,float d){kp=p;ki=i;kd=d;}
  void reset(){err0=0;intg=0;}
  float compute(float sp,float pv,float dt){
    float e=sp-pv;
    intg=constrain(intg+e*dt,-PID_I_LIMIT,PID_I_LIMIT);
    float dv=(dt>0)?((e-err0)/dt):0; err0=e;
    return constrain(kp*e+ki*intg+kd*dv,-PID_OUTPUT_LIM,PID_OUTPUT_LIM);
  }
};
static PIDCtrl pidR,pidP,pidY;

// =============================================================
// SECTION 16 - FLIGHT LOOP TASK (Core 0, 250 Hz)
// =============================================================
static void flightTask(void*){
  xSemaphoreTake(mtx,portMAX_DELAY);
    pidR.set(drone.kp_r,drone.ki_r,drone.kd_r);
    pidP.set(drone.kp_p,drone.ki_p,drone.kd_p);
    pidY.set(drone.kp_y,drone.ki_y,drone.kd_y);
  xSemaphoreGive(mtx);
  // Gyro calibration
  Serial.println("[Flight] Calibrating gyro (keep still ~1s)...");
  float gx0=0,gy0=0,gz0=0;
  for(int i=0;i<GYRO_CAL_N;i++){
    float ax,ay,az,gx,gy,gz; mpuRead(ax,ay,az,gx,gy,gz);
    gx0+=gx;gy0+=gy;gz0+=gz; delay(5);
  }
  gx0/=GYRO_CAL_N; gy0/=GYRO_CAL_N; gz0/=GYRO_CAL_N;
  Serial.printf("[Flight] Gyro offsets: GX=%.3f GY=%.3f GZ=%.3f\n",gx0,gy0,gz0);
  // Baro reference
  {float p,t; bmpReadRaw(p,t); if(p>50000&&p<120000)seaLevelP=p;
   Serial.printf("[Flight] Sea level: %.2f Pa\n",seaLevelP);}
  float cfR=0,cfP=0,yawA=0;
  unsigned long prevUs=micros();
  uint8_t baroCnt=0; uint32_t ledTk=0;
  for(;;){
    unsigned long now=micros();
    float dt=(float)(now-prevUs)*1e-6f;
    if(dt<=0||dt>0.05f)dt=0.004f;
    prevUs=now;
    float ax,ay,az,gx,gy,gz; mpuRead(ax,ay,az,gx,gy,gz);
    gx-=gx0; gy-=gy0; gz-=gz0;
    float aR=atan2f(ay,az)*57.29578f;
    float aP=atan2f(-ax,sqrtf(ay*ay+az*az))*57.29578f;
    cfR=CF_ALPHA*(cfR+gx*dt)+(1.0f-CF_ALPHA)*aR;
    cfP=CF_ALPHA*(cfP+gy*dt)+(1.0f-CF_ALPHA)*aP;
    yawA+=gz*dt;
    if(yawA>180.0f)yawA-=360.0f; if(yawA<-180.0f)yawA+=360.0f;
    xSemaphoreTake(mtx,portMAX_DELAY);
      pidR.set(drone.kp_r,drone.ki_r,drone.kd_r);
      pidP.set(drone.kp_p,drone.ki_p,drone.kd_p);
      pidY.set(drone.kp_y,drone.ki_y,drone.kd_y);
      bool armd=drone.armed;
      float spR=constrain(drone.sp_roll,-MAX_ANGLE_DEG,MAX_ANGLE_DEG);
      float spP=constrain(drone.sp_pitch,-MAX_ANGLE_DEG,MAX_ANGLE_DEG);
      float spY=constrain(drone.sp_yaw_rate,-MAX_YAW_RATE,MAX_YAW_RATE);
      int   spT=drone.sp_throttle;
      if(armd&&(millis()-drone.last_cmd_ms)>SIGNAL_TO_MS){
        drone.armed=false;armd=false;
        Serial.println("[Flight] Signal timeout - disarmed!");}
      drone.roll=cfR; drone.pitch=cfP; drone.yaw_accum=yawA;
    xSemaphoreGive(mtx);
    if(armd&&spT>=THROTTLE_IDLE){
      float oR=pidR.compute(spR,cfR,dt);
      float oP=pidP.compute(spP,cfP,dt);
      float oY=pidY.compute(spY,gz,dt);
      // X-frame mixing
      uint16_t o1=(uint16_t)constrain(spT+oR-oP+oY,THROTTLE_IDLE,THROTTLE_MAX);
      uint16_t o2=(uint16_t)constrain(spT+oR+oP-oY,THROTTLE_IDLE,THROTTLE_MAX);
      uint16_t o3=(uint16_t)constrain(spT-oR+oP+oY,THROTTLE_IDLE,THROTTLE_MAX);
      uint16_t o4=(uint16_t)constrain(spT-oR-oP-oY,THROTTLE_IDLE,THROTTLE_MAX);
      motorsWrite(o1,o2,o3,o4);
      xSemaphoreTake(mtx,portMAX_DELAY);
        drone.m1=o1;drone.m2=o2;drone.m3=o3;drone.m4=o4;
      xSemaphoreGive(mtx);
    } else {
      pidR.reset(); pidP.reset(); pidY.reset();
      motorsIdle();
      xSemaphoreTake(mtx,portMAX_DELAY);
        drone.m1=drone.m2=drone.m3=drone.m4=ESC_IDLE_US;
      xSemaphoreGive(mtx);
    }
    if(++baroCnt>=BARO_DIVIDER){
      baroCnt=0;
      float pres,tmp; bmpReadRaw(pres,tmp);
      float alt=bmpToAlt(pres),vb=readBattery();
      xSemaphoreTake(mtx,portMAX_DELAY);
        drone.altitude=alt;drone.temperature=tmp;drone.vbat=vb;
      xSemaphoreGive(mtx);
    }
    if(++ledTk%(armd?31:125)==0){static bool ls=false;ls=!ls;digitalWrite(PIN_LED,ls);}
    long el=(long)(micros()-now);
    if(el<(long)FLIGHT_LOOP_US)delayMicroseconds((uint32_t)((long)FLIGHT_LOOP_US-el));
  }
}

// =============================================================
// SECTION 17 - WEBSOCKET HANDLER
// =============================================================
static void onWsEvent(AsyncWebSocket*,AsyncWebSocketClient* client,
                      AwsEventType type,void* arg,uint8_t* data,size_t len){
  if(type==WS_EVT_CONNECT){
    Serial.printf("[WS] Client #%u connected\n",client->id());
    xSemaphoreTake(mtx,portMAX_DELAY);
      drone.gcs_connected=true;drone.last_cmd_ms=millis();
    xSemaphoreGive(mtx);
  } else if(type==WS_EVT_DISCONNECT){
    Serial.printf("[WS] Client #%u disconnected\n",client->id());
    xSemaphoreTake(mtx,portMAX_DELAY);
      if(ws.count()==0)drone.gcs_connected=false;
    xSemaphoreGive(mtx);
  } else if(type==WS_EVT_DATA){
    AwsFrameInfo* info=(AwsFrameInfo*)arg;
    if(info->opcode!=WS_TEXT||len==0||len>511)return;
    char buf[512]; memcpy(buf,data,len); buf[len]='\0';
    StaticJsonDocument<384> doc;
    if(deserializeJson(doc,buf)!=DeserializationError::Ok)return;
    const char* cmd=doc["cmd"]|"";
    xSemaphoreTake(mtx,portMAX_DELAY);
      drone.last_cmd_ms=millis();
      if(!strcmp(cmd,"arm")){drone.armed=true;Serial.println("[WS] ARMED");}
      else if(!strcmp(cmd,"disarm")){drone.armed=false;drone.sp_throttle=ESC_IDLE_US;Serial.println("[WS] Disarmed");}
      else if(!strcmp(cmd,"ctrl")){
        int tp=doc["thr"]|0;
        drone.sp_throttle=THROTTLE_IDLE+(int)((float)tp/100.0f*(THROTTLE_MAX-THROTTLE_IDLE));
        drone.sp_roll     =(float)(doc["roll"]|0)/100.0f*MAX_ANGLE_DEG;
        drone.sp_pitch    =(float)(doc["pitch"]|0)/100.0f*MAX_ANGLE_DEG;
        drone.sp_yaw_rate =(float)(doc["yaw"]|0)/100.0f*MAX_YAW_RATE;
      } else if(!strcmp(cmd,"pid")){
        if(doc.containsKey("kp_r"))drone.kp_r=(float)doc["kp_r"];
        if(doc.containsKey("ki_r"))drone.ki_r=(float)doc["ki_r"];
        if(doc.containsKey("kd_r"))drone.kd_r=(float)doc["kd_r"];
        if(doc.containsKey("kp_p"))drone.kp_p=(float)doc["kp_p"];
        if(doc.containsKey("ki_p"))drone.ki_p=(float)doc["ki_p"];
        if(doc.containsKey("kd_p"))drone.kd_p=(float)doc["kd_p"];
        if(doc.containsKey("kp_y"))drone.kp_y=(float)doc["kp_y"];
        if(doc.containsKey("ki_y"))drone.ki_y=(float)doc["ki_y"];
        if(doc.containsKey("kd_y"))drone.kd_y=(float)doc["kd_y"];
        Serial.println("[WS] PID updated");
      }
    xSemaphoreGive(mtx);
  }
}

// =============================================================
// SECTION 18 - COMMS TASK (Core 1, 10 Hz telemetry)
// =============================================================
static void commsTask(void*){
  unsigned long lastT=0;
  for(;;){
    unsigned long now=millis();
    if(now-lastT>=TELEM_MS){
      lastT=now;
      xSemaphoreTake(mtx,portMAX_DELAY);
        float roll=drone.roll,pitch=drone.pitch,yaw=drone.yaw_accum;
        float alt=drone.altitude,tmp=drone.temperature,vb=drone.vbat;
        uint16_t m1=drone.m1,m2=drone.m2,m3=drone.m3,m4=drone.m4;
        bool armd=drone.armed;
      xSemaphoreGive(mtx);
      StaticJsonDocument<256> doc;
      doc["roll"]=round(roll*10)/10.0; doc["pitch"]=round(pitch*10)/10.0;
      doc["yaw"]=round(yaw*10)/10.0;   doc["alt"]=round(alt*10)/10.0;
      doc["temp"]=round(tmp*10)/10.0;  doc["vbat"]=round(vb*100)/100.0;
      doc["m1"]=m1;doc["m2"]=m2;doc["m3"]=m3;doc["m4"]=m4;
      doc["armed"]=armd;doc["rssi"]=WiFi.RSSI();
      char tb[256]; size_t n=serializeJson(doc,tb,sizeof(tb));
      if(ws.count()>0)ws.textAll(tb,n);
    }
    ws.cleanupClients(4);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =============================================================
// SECTION 19 - SETUP
// =============================================================
void setup(){
  Serial.begin(115200); delay(600);
  Serial.println("\n+------------------------------------------+");
  Serial.println("|  ESP32-S3 DRONE UAV - Flight Controller  |");
  Serial.println("|  Board: ESP32-S3 DevKitC-1 N16R8         |");
  Serial.println("+------------------------------------------+\n");
  pinMode(PIN_LED,OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Wire.begin(PIN_SDA,PIN_SCL,400000UL);
  Serial.print("[Init] MPU-6050 ... ");
  Serial.println(mpuInit()?"OK":"FAIL - check wiring!");
  Serial.print("[Init] BMP280   ... ");
  Serial.println(bmpInit()?"OK":"FAIL - check wiring!");
  Serial.println("[Init] ESC arm pulse (2s)...");
  motorsInit(); motorsIdle(); delay(2000);
  Serial.println("[Init] ESC ready");
  memset(&drone,0,sizeof(drone));
  drone.sp_throttle=ESC_IDLE_US;
  drone.m1=drone.m2=drone.m3=drone.m4=ESC_IDLE_US;
  drone.vbat=readBattery(); drone.temperature=25.0f;
  drone.last_cmd_ms=millis();
  drone.kp_r=INIT_KP_ROLL;  drone.ki_r=INIT_KI_ROLL;  drone.kd_r=INIT_KD_ROLL;
  drone.kp_p=INIT_KP_PITCH; drone.ki_p=INIT_KI_PITCH; drone.kd_p=INIT_KD_PITCH;
  drone.kp_y=INIT_KP_YAW;   drone.ki_y=INIT_KI_YAW;   drone.kd_y=INIT_KD_YAW;
  mtx=xSemaphoreCreateMutex();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP,AP_GW,AP_SUBNET);
  WiFi.softAP(AP_SSID,AP_PASS,6,false,4);
  Serial.printf("[WiFi] Hotspot SSID: %s  IP: %s\n",AP_SSID,WiFi.softAPIP().toString().c_str());
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/",HTTP_GET,[](AsyncWebServerRequest* r){r->send_P(200,"text/html",GCS_HTML);});
  server.on("/health",HTTP_GET,[](AsyncWebServerRequest* r){r->send(200,"text/plain","OK");});
  server.onNotFound([](AsyncWebServerRequest* r){r->redirect("/");});
  server.begin();
  Serial.println("[HTTP] GCS at http://192.168.4.1");
  xTaskCreatePinnedToCore(flightTask,"FlightTask",8192,NULL,5,NULL,0);
  xTaskCreatePinnedToCore(commsTask, "CommsTask", 8192,NULL,2,NULL,1);
  Serial.println("\n[Boot] System ready!");
  Serial.printf("[Boot] Connect WiFi: %s  Pass: %s\n",AP_SSID,AP_PASS);
  Serial.println("[Boot] Open browser: http://192.168.4.1\n");
}

// =============================================================
// SECTION 20 - LOOP (idle - all work in FreeRTOS tasks)
// =============================================================
void loop(){ vTaskDelay(pdMS_TO_TICKS(1000)); }
