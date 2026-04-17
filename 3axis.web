/*
  ESP32 + ICM-20948 (SPI MODE 3) — 3D Orientation + 9-axis
  
  O sensor deste utilizador requer SPI Mode 3 (CPOL=1, CPHA=1),
    spi = SPI(1, baudrate=1000000, polarity=1, phase=1, ...)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPI.h>
#include <math.h>

// ==================== SPI CONFIG (Mode 3!) ====================
static constexpr int SPI_SCK_PIN  = 18;
static constexpr int SPI_MISO_PIN = 19;
static constexpr int SPI_MOSI_PIN = 23;
static constexpr int SPI_CS_PIN   =  5;
static constexpr uint32_t SPI_CLK = 1000000;  // 1 MHz (como no Python)

// ==================== WiFi ====================
const char* AP_SSID = "ICM20948-3D";
const char* AP_PASS = "12345678";

// ==================== Servers ====================
WebServer http(80);
WebSocketsServer ws(81);

// ==================== Timing ====================
uint32_t lastSendMs = 0;
const uint32_t SEND_MS = 20; // 50 Hz

// ==================== Sensor data ====================
float gAx=0, gAy=0, gAz=0;
float gGx=0, gGy=0, gGz=0;
float gMx=0, gMy=0, gMz=0;
double gQ0=1, gQ1=0, gQ2=0, gQ3=0;

// Complementary filter state
float filtRoll  = 0;
float filtPitch = 0;
float filtYaw   = 0;
uint32_t lastIMUus = 0;
const float ALPHA = 0.98; // gyro weight

bool sensorOk = false;

// ==========================================================
//  MINIMAL ICM-20948 DRIVER (SPI Mode 3)
// ==========================================================

void icm_write(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);  // bit7=0 → write
  SPI.transfer(val);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t icm_read(uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);  // bit7=1 → read
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void icm_readBytes(uint8_t reg, uint8_t* buf, uint8_t count) {
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE3));
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < count; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.endTransaction();
}

void icm_selectBank(uint8_t bank) {
  // Reg 0x7F = REG_BANK_SEL, bank in bits [5:4]
  icm_write(0x7F, (bank & 0x03) << 4);
}

int16_t toInt16(uint8_t hi, uint8_t lo) {
  return (int16_t)((hi << 8) | lo);
}

// ---- Magnetometer (AK09916 inside ICM-20948, via I2C master) ----

void icm_i2cMasterEnable() {
  icm_selectBank(0);
  // USER_CTRL (0x03): enable I2C master
  uint8_t uc = icm_read(0x03);
  icm_write(0x03, uc | 0x20);  // I2C_MST_EN
  
  icm_selectBank(3);
  // I2C_MST_CTRL (0x01): 400 kHz
  icm_write(0x01, 0x07);  // clock divider for ~400kHz
}

void icm_magWrite(uint8_t reg, uint8_t val) {
  icm_selectBank(3);
  icm_write(0x03, 0x0C);          // I2C_SLV0_ADDR = AK09916 addr (0x0C), write
  icm_write(0x04, reg);           // I2C_SLV0_REG
  icm_write(0x05, val);           // I2C_SLV0_DO → data to write
  icm_write(0x06, 0x81);          // I2C_SLV0_CTRL: enable, 1 byte
  icm_selectBank(0);
  delay(10);
}

void icm_magSetup() {
  // Reset magnetometer
  icm_magWrite(0x31, 0x01);  // CNTL3 = soft reset
  delay(100);
  // Set continuous measurement mode 4 (100 Hz)
  icm_magWrite(0x31, 0x00);  // CNTL3 clear
  delay(10);
  icm_magWrite(0x32, 0x08);  // CNTL2 = continuous mode 4 (100Hz)
  delay(10);
  
  // Setup SLV0 for auto-read: read 8 bytes from AK09916 starting at 0x10
  icm_selectBank(3);
  icm_write(0x03, 0x8C);  // I2C_SLV0_ADDR = 0x0C | 0x80 (read)
  icm_write(0x04, 0x10);  // I2C_SLV0_REG = start reading at ST1 (0x10)
  icm_write(0x06, 0x89);  // I2C_SLV0_CTRL: enable, read 9 bytes (ST1 + 6 data + ST2 + dummy)
  icm_selectBank(0);
}

bool icm_readMag(float &mx, float &my, float &mz) {
  // External sensor data is auto-read to EXT_SLV_SENS_DATA_00 (0x3B) in Bank 0
  icm_selectBank(0);
  uint8_t buf[9];
  icm_readBytes(0x3B, buf, 9);
  
  uint8_t st1 = buf[0];
  if (!(st1 & 0x01)) return false; // DRDY not set
  
  // AK09916: data is little-endian
  int16_t rx = (int16_t)(buf[2] << 8 | buf[1]);
  int16_t ry = (int16_t)(buf[4] << 8 | buf[3]);
  int16_t rz = (int16_t)(buf[6] << 8 | buf[5]);
  
  // Sensitivity: 0.15 µT/LSB
  mx = rx * 0.15;
  my = ry * 0.15;
  mz = rz * 0.15;
  return true;
}

// ---- Init ----

bool icm_init() {
  Serial.println("\n[ICM] Init (SPI Mode 3)...");
  
  pinMode(SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  delay(10);
  
  SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

  // Select Bank 0
  icm_selectBank(0);
  delay(10);
  
  // Read WHO_AM_I
  uint8_t whoami = icm_read(0x00);
  Serial.printf("[ICM] WHO_AM_I = 0x%02X", whoami);
  if (whoami == 0xEA) Serial.println(" → ICM-20948 OK!");
  else { Serial.printf(" → ERRADO (esperava 0xEA)\n"); return false; }

  // Software reset
  icm_write(0x06, 0x80);
  delay(200);
  
  // Select Bank 0 again after reset
  icm_selectBank(0);
  delay(10);
  
  // Wake up + auto clock
  icm_write(0x06, 0x01);
  delay(50);
  
  // ODR_ALIGN enable
  icm_write(0x09, 0x01);
  
  // Disable low power mode, enable all accel+gyro axes
  icm_write(0x05, 0x00);  // LP_CONFIG: disable duty cycle modes
  icm_write(0x07, 0x00);  // PWR_MGMT_2: all sensors on
  delay(50);
  
  // ---- Bank 2: Configure accel + gyro ----
  icm_selectBank(2);
  
  // GYRO_CONFIG_1 (0x01): ±250 dps, DLPF enabled, BW ~196 Hz
  icm_write(0x01, 0x01);  // GYRO_FS_SEL=0 (250dps) | GYRO_FCHOICE=1
  
  // GYRO_SMPLRT_DIV (0x00): divider=4 → ~225 Hz
  icm_write(0x00, 0x04);
  
  // ACCEL_CONFIG (0x14): ±2g, DLPF enabled
  icm_write(0x14, 0x01);  // ACCEL_FS_SEL=0 (2g) | ACCEL_FCHOICE=1
  
  // ACCEL_SMPLRT_DIV (0x10-0x11): divider=4
  icm_write(0x10, 0x00);
  icm_write(0x11, 0x04);
  
  icm_selectBank(0);
  delay(50);
  
  // ---- Setup I2C master for magnetometer ----
  icm_i2cMasterEnable();
  delay(20);
  icm_magSetup();
  delay(50);
  
  // Test read
  uint8_t buf[12];
  icm_readBytes(0x2D, buf, 12);
  int16_t ax = toInt16(buf[0], buf[1]);
  int16_t ay = toInt16(buf[2], buf[3]);
  int16_t az = toInt16(buf[4], buf[5]);
  int16_t gx = toInt16(buf[6], buf[7]);
  int16_t gy = toInt16(buf[8], buf[9]);
  int16_t gz = toInt16(buf[10], buf[11]);
  
  Serial.printf("[ICM] Test: Acc=[%d,%d,%d] Gyr=[%d,%d,%d]\n", ax, ay, az, gx, gy, gz);
  Serial.println("[ICM] Sensor ready!");
  
  return true;
}

// ---- Read accel+gyro (Bank 0, regs 0x2D-0x38) ----
void icm_readAG() {
  icm_selectBank(0);
  uint8_t buf[12];
  icm_readBytes(0x2D, buf, 12);
  
  // Accel: sensitivity 16384 LSB/g for ±2g
  gAx = toInt16(buf[0], buf[1]) / 16384.0;
  gAy = toInt16(buf[2], buf[3]) / 16384.0;
  gAz = toInt16(buf[4], buf[5]) / 16384.0;
  
  // Gyro: sensitivity 131 LSB/(°/s) for ±250 dps
  gGx = toInt16(buf[6], buf[7]) / 131.0;
  gGy = toInt16(buf[8], buf[9]) / 131.0;
  gGz = toInt16(buf[10], buf[11]) / 131.0;
}

// ==========================================================
//  COMPLEMENTARY FILTER (accel+gyro → quaternion)
// ==========================================================

void updateOrientation() {
  uint32_t now = micros();
  float dt = (now - lastIMUus) / 1000000.0;
  lastIMUus = now;
  
  if (dt <= 0 || dt > 0.5) {
    // First call or overflow
    filtRoll  = atan2(gAy, gAz);
    filtPitch = atan2(-gAx, sqrt(gAy*gAy + gAz*gAz));
    filtYaw   = 0;
    goto toQuat;
  }
  
  {
    // Accel angles
    float accelRoll  = atan2(gAy, gAz);
    float accelPitch = atan2(-gAx, sqrt(gAy*gAy + gAz*gAz));
    
    // Gyro integration (degrees/s → radians)
    float gxRad = gGx * (M_PI / 180.0) * dt;
    float gyRad = gGy * (M_PI / 180.0) * dt;
    float gzRad = gGz * (M_PI / 180.0) * dt;
    
    // Complementary filter: gyro (fast) + accel (drift correction)
    filtRoll  = ALPHA * (filtRoll  + gxRad) + (1.0 - ALPHA) * accelRoll;
    filtPitch = ALPHA * (filtPitch + gyRad) + (1.0 - ALPHA) * accelPitch;
    filtYaw  += gzRad;  // No mag correction yet (can add later)
  }

toQuat:
  // Euler → Quaternion
  float cr = cos(filtRoll/2),  sr = sin(filtRoll/2);
  float cp = cos(filtPitch/2), sp = sin(filtPitch/2);
  float cy = cos(filtYaw/2),   sy = sin(filtYaw/2);
  
  gQ0 = cr*cp*cy + sr*sp*sy;
  gQ1 = sr*cp*cy - cr*sp*sy;
  gQ2 = cr*sp*cy + sr*cp*sy;
  gQ3 = cr*cp*sy - sr*sp*cy;
  
  double n = sqrt(gQ0*gQ0 + gQ1*gQ1 + gQ2*gQ2 + gQ3*gQ3);
  if (n > 0) { gQ0/=n; gQ1/=n; gQ2/=n; gQ3/=n; }
}

// ==========================================================
//  HTML
// ==========================================================

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pt">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ICM-20948 • 3D 9-Axis</title>
<style>
*{box-sizing:border-box}
html,body{margin:0;height:100%;background:#0b1220;color:#e7eefc;font-family:system-ui,sans-serif;overflow:hidden}
#hud{position:fixed;left:8px;top:8px;z-index:10;display:flex;flex-direction:column;gap:6px;max-width:280px}
#rp{position:fixed;right:8px;top:8px;z-index:10}
.p{background:rgba(255,255,255,.07);border:1px solid rgba(255,255,255,.12);border-radius:12px;padding:8px 12px;font-size:12px}
.m{font-family:monospace;font-size:11px;line-height:1.6}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px;background:#ff4a4a;transition:background .3s}
.ok{background:#3cff7a}
button{padding:4px 8px;border-radius:6px;border:1px solid rgba(255,255,255,.15);background:rgba(255,255,255,.07);color:#e7eefc;cursor:pointer;font-size:11px;margin:2px}
button:hover{background:rgba(255,255,255,.15)}
button.on{background:rgba(60,255,122,.15)}
h4{margin:0 0 4px;font-size:11px;opacity:.5;text-transform:uppercase;letter-spacing:.5px}
.bar{display:flex;align-items:center;gap:4px;margin:1px 0}
.bl{width:14px;font-weight:bold;font-size:10px}
.bg{flex:1;height:6px;background:rgba(255,255,255,.06);border-radius:3px;overflow:hidden}
.bf{height:100%;border-radius:3px;transition:width .08s}
.bv{width:52px;font-size:10px;text-align:right}
#c{width:100%;height:100%;display:block}
</style>
</head>
<body>
<div id="hud">
  <div class="p">
    <div><b>ESP32 + ICM-20948</b></div>
    <div><span id="led" class="dot"></span><span id="st">A ligar…</span></div>
    <div class="m" id="md"></div>
    <div class="m" id="q"></div>
    <div class="m" id="fps"></div>
    <div style="margin-top:4px">
      <button id="bR">Reconnect</button>
      <button id="bS">Simulação</button>
      <button id="bZ">Reset</button>
    </div>
  </div>
  <div class="p"><h4>Acelerómetro (g)</h4><div id="da" class="m"></div></div>
  <div class="p"><h4>Giroscópio (°/s)</h4><div id="dg" class="m"></div></div>
  <div class="p"><h4>Magnetómetro (µT)</h4><div id="dm" class="m"></div></div>
</div>
<div id="rp" class="p">
  <div style="color:#ff4040"><b>━━</b> X</div>
  <div style="color:#40ff70"><b>━━</b> Y</div>
  <div style="color:#4080ff"><b>━━</b> Z</div>
  <div style="opacity:.4;font-size:10px;margin-top:4px">Complementary filter</div>
</div>
<canvas id="c"></canvas>
<script>
(()=>{
const $=id=>document.getElementById(id);
const cv=$('c'),ctx=cv.getContext('2d');
let W=0,H=0;
function resize(){const d=devicePixelRatio||1;W=innerWidth;H=innerHeight;cv.width=Math.floor(W*d);cv.height=Math.floor(H*d);cv.style.width=W+'px';cv.style.height=H+'px';ctx.setTransform(d,0,0,d,0,0);}
addEventListener('resize',resize);resize();

let tW=1,tX=0,tY=0,tZ=0,cW=1,cX=0,cY=0,cZ=0;
let sim=false,simT=0;
let ax=0,ay=0,az=0,gx=0,gy=0,gz=0,mx=0,my=0,mz=0;
let fc=0,ft=performance.now(),mc=0;

function setC(ok){$('led').classList.toggle('ok',ok);}
function nrm(){const n=Math.sqrt(cW*cW+cX*cX+cY*cY+cZ*cZ)||1;cW/=n;cX/=n;cY/=n;cZ/=n;}
function lerp(f){cW+=(tW-cW)*f;cX+=(tX-cX)*f;cY+=(tY-cY)*f;cZ+=(tZ-cZ)*f;nrm();}
function q2m(w,x,y,z){const xx=x*x,yy=y*y,zz=z*z,xy=x*y,xz=x*z,yz=y*z,wx=w*x,wy=w*y,wz=w*z;return[1-2*(yy+zz),2*(xy-wz),2*(xz+wy),2*(xy+wz),1-2*(xx+zz),2*(yz-wx),2*(xz-wy),2*(yz+wx),1-2*(xx+yy)];}
function rv(m,a,b,c){return{x:m[0]*a+m[1]*b+m[2]*c,y:m[3]*a+m[4]*b+m[5]*c,z:m[6]*a+m[7]*b+m[8]*c};}

const crx=-30*Math.PI/180,cry=40*Math.PI/180;
const c1=Math.cos(crx),s1=Math.sin(crx),c2=Math.cos(cry),s2=Math.sin(cry);
const cam=[c2,s2*s1,s2*c1,0,c1,-s1,-s2,c2*s1,c2*c1];
function proj(v,cx,cy,sc){const u=rv(cam,v.x,v.y,v.z);const d=5+u.z,p=sc/d;return{x:cx+u.x*p,y:cy-u.y*p,z:u.z};}

function arrow(p0,p1,col,zv){
  const al=Math.max(0.4,Math.min(1,0.5+0.5*((zv+2)/4)));
  ctx.globalAlpha=al;ctx.strokeStyle=col;ctx.fillStyle=col;
  ctx.lineWidth=4;ctx.lineCap='round';
  ctx.beginPath();ctx.moveTo(p0.x,p0.y);ctx.lineTo(p1.x,p1.y);ctx.stroke();
  const dx=p1.x-p0.x,dy=p1.y-p0.y,a=Math.atan2(dy,dx),h=14;
  ctx.beginPath();ctx.moveTo(p1.x,p1.y);
  ctx.lineTo(p1.x+h*Math.cos(a+2.6),p1.y+h*Math.sin(a+2.6));
  ctx.lineTo(p1.x+h*Math.cos(a-2.6),p1.y+h*Math.sin(a-2.6));
  ctx.closePath();ctx.fill();ctx.globalAlpha=1;
}

function bars(el,vals,mx2,cols){
  const n=['X','Y','Z'];let h='';
  for(let i=0;i<3;i++){const pct=Math.min(100,Math.abs(vals[i])/mx2*100);const s=vals[i]>=0?'+':'';
  h+='<div class="bar"><div class="bl" style="color:'+cols[i]+'">'+n[i]+'</div><div class="bg"><div class="bf" style="width:'+pct+'%;background:'+cols[i]+'"></div></div><div class="bv">'+s+vals[i].toFixed(2)+'</div></div>';}
  el.innerHTML=h;
}
const ac=['#ff4040','#40ff70','#4080ff'];

function draw(){
  if(sim){simT+=.015;tW=Math.cos(simT);tX=Math.sin(simT)*.5;tY=Math.sin(simT*.7)*.5;tZ=Math.sin(simT*1.3)*.5;const n=Math.sqrt(tW*tW+tX*tX+tY*tY+tZ*tZ)||1;tW/=n;tX/=n;tY/=n;tZ/=n;}
  lerp(0.3);
  ctx.clearRect(0,0,W,H);ctx.fillStyle='#0b1220';ctx.fillRect(0,0,W,H);
  const cx=W*.52,cy=H*.5,sc=Math.min(W,H)*1.2,m=q2m(cW,cX,cY,cZ),L=1.8;

  ctx.strokeStyle='rgba(255,255,255,0.04)';ctx.lineWidth=1;
  for(let i=-3;i<=3;i++){let a=proj({x:i*.5,y:0,z:-1.5},cx,cy,sc),b=proj({x:i*.5,y:0,z:1.5},cx,cy,sc);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();a=proj({x:-1.5,y:0,z:i*.5},cx,cy,sc);b=proj({x:1.5,y:0,z:i*.5},cx,cy,sc);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();}

  const eX=rv(m,L,0,0),eY=rv(m,0,L,0),eZ=rv(m,0,0,L);
  const nX=rv(m,-.5,0,0),nY=rv(m,0,-.5,0),nZ=rv(m,0,0,-.5);
  const o=proj({x:0,y:0,z:0},cx,cy,sc);
  const px=proj(eX,cx,cy,sc),py=proj(eY,cx,cy,sc),pz=proj(eZ,cx,cy,sc);
  const npx=proj(nX,cx,cy,sc),npy=proj(nY,cx,cy,sc),npz=proj(nZ,cx,cy,sc);

  ctx.setLineDash([3,3]);ctx.lineWidth=1.5;
  [{p:npx,c:'#ff4040'},{p:npy,c:'#40ff70'},{p:npz,c:'#4080ff'}].forEach(a=>{ctx.globalAlpha=.2;ctx.strokeStyle=a.c;ctx.beginPath();ctx.moveTo(o.x,o.y);ctx.lineTo(a.p.x,a.p.y);ctx.stroke();});
  ctx.setLineDash([]);ctx.globalAlpha=1;

  const axes=[{p:px,c:'#ff4040',z:px.z,l:'X'},{p:py,c:'#40ff70',z:py.z,l:'Y'},{p:pz,c:'#4080ff',z:pz.z,l:'Z'}].sort((a,b)=>a.z-b.z);
  axes.forEach(a=>arrow(o,a.p,a.c,a.z));

  ctx.fillStyle='rgba(255,255,255,0.4)';ctx.beginPath();ctx.arc(o.x,o.y,4,0,Math.PI*2);ctx.fill();
  ctx.font='bold 18px system-ui';axes.forEach(a=>{ctx.fillStyle=a.c;ctx.fillText(a.l,a.p.x+8,a.p.y-4);});

  bars($('da'),[ax,ay,az],4,ac);bars($('dg'),[gx,gy,gz],500,ac);bars($('dm'),[mx,my,mz],100,ac);

  fc++;const now=performance.now();
  if(now-ft>=1000){$('fps').textContent='Render: '+fc+' fps | WS: '+mc+'/s';fc=0;mc=0;ft=now;}
  $('q').textContent='q={ w:'+tW.toFixed(3)+' x:'+tX.toFixed(3)+' y:'+tY.toFixed(3)+' z:'+tZ.toFixed(3)+' }';
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

let sock=null,rt=null;
function connect(){
  if(sock&&sock.readyState<2)sock.close();if(rt){clearTimeout(rt);rt=null;}
  const url='ws://'+location.hostname+':81/';$('st').textContent='A ligar…';setC(false);
  sock=new WebSocket(url);
  sock.onopen=()=>{$('st').textContent='Ligado';setC(true);};
  sock.onclose=()=>{$('st').textContent='Desligado';setC(false);rt=setTimeout(connect,2000);};
  sock.onerror=()=>{$('st').textContent='Erro WS';setC(false);};
  sock.onmessage=(ev)=>{mc++;try{const d=JSON.parse(ev.data);
    if(d.w!==undefined&&!sim){const w=+d.w,x=+d.x,y=+d.y,z=+d.z;if(isFinite(w)&&isFinite(x)&&isFinite(y)&&isFinite(z)){const n=Math.sqrt(w*w+x*x+y*y+z*z)||1;tW=w/n;tX=x/n;tY=y/n;tZ=z/n;}}
    if(d.ax!==undefined){ax=+d.ax;ay=+d.ay;az=+d.az;}
    if(d.gx!==undefined){gx=+d.gx;gy=+d.gy;gz=+d.gz;}
    if(d.mx!==undefined){mx=+d.mx;my=+d.my;mz=+d.mz;}
    if(d.md!==undefined)$('md').textContent='Modo: '+d.md;
  }catch(e){}};
}
$('bR').onclick=()=>connect();
$('bS').onclick=function(){sim=!sim;this.className=sim?'on':'';this.textContent=sim?'Sim: ON':'Simulação';};
$('bZ').onclick=()=>{tW=1;tX=0;tY=0;tZ=0;cW=1;cX=0;cY=0;cZ=0;};
connect();
})();
</script>
</body>
</html>
)HTML";

// ==========================================================
//  HTTP / WS / WiFi
// ==========================================================

void sendHTML() {
  const size_t len = strlen_P(INDEX_HTML);
  http.setContentLength(len);
  http.send(200, "text/html", "");
  const size_t CS = 1024;
  char buf[CS + 1];
  size_t s = 0;
  while (s < len) {
    size_t n = min(CS, len - s);
    memcpy_P(buf, INDEX_HTML + s, n);
    buf[n] = '\0';
    http.sendContent(buf);
    s += n;
    yield();
  }
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED)       Serial.printf("[WS] #%u connected\n", num);
  else if (type == WStype_DISCONNECTED) Serial.printf("[WS] #%u disconnected\n", num);
}

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  Serial.printf("[WIFI] AP  SSID: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ==========================================================
//  SETUP
// ==========================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n\n=== ICM-20948 3D (SPI Mode 3) ===\n");

  setupWiFi();

  http.on("/", HTTP_GET, sendHTML);
  http.on("/favicon.ico", HTTP_GET, []() { http.send(204); });
  http.onNotFound([]() { http.send(404, "text/plain", "404"); });
  http.begin();
  Serial.println("[HTTP] :80 OK");

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("[WS]   :81 OK");

  sensorOk = icm_init();
  
  if (sensorOk) {
    lastIMUus = micros();
    Serial.println("\n=== SENSOR OK — SPI Mode 3 ===");
  } else {
    Serial.println("\n=== SENSOR FAIL ===");
  }
  Serial.println("Abre http://192.168.4.1/\n");
}

// ==========================================================
//  LOOP
// ==========================================================

void loop() {
  // Sempre servir HTTP e WS primeiro
  http.handleClient();
  ws.loop();

  if (sensorOk) {
    // Ler accel + gyro
    icm_readAG();
    
    // Ler magnetómetro (pode falhar, não é crítico)
    icm_readMag(gMx, gMy, gMz);
    
    // Complementary filter → quaternion
    updateOrientation();
  }

  // Enviar via WS
  uint32_t now = millis();
  if (now - lastSendMs >= SEND_MS) {
    lastSendMs = now;

    char msg[300];
    snprintf(msg, sizeof(msg),
      "{\"w\":%.5f,\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,"
      "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
      "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
      "\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,"
      "\"md\":\"%s\"}",
      gQ0, gQ1, gQ2, gQ3,
      gAx, gAy, gAz,
      gGx, gGy, gGz,
      gMx, gMy, gMz,
      sensorOk ? "SPI Mode3 CF" : "NO SENSOR");

    ws.broadcastTXT(msg);

    // Debug serial
    static uint32_t lastDbg = 0;
    if (now - lastDbg >= 500) {
      lastDbg = now;
      Serial.printf("A[%.2f %.2f %.2f]g  G[%.0f %.0f %.0f]  M[%.0f %.0f %.0f]  Q[%.3f %.3f %.3f %.3f]\n",
        gAx,gAy,gAz, gGx,gGy,gGz, gMx,gMy,gMz, gQ0,gQ1,gQ2,gQ3);
    }
  }

  delay(1);
}
