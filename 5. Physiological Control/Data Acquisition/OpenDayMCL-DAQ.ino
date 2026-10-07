#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MPRLS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <driver/twai.h> 

// ============================================================
// CONFIGURATION
// ============================================================

#define I2C_SDA 48
#define I2C_SCL 47
#define I2C_FREQ 400000
#define MUX_ADDR 0x70
#define MPR_ADDR 0x18
#define PRELOAD_ADDR 0x28
#define CAN_TX_PIN 42 
#define CAN_RX_PIN 41
#define CAN_ID_MOTOR 0x201 
const uint32_t CAN_TIMEOUT_MS = 1000; 


const uint8_t FLOW_PIN[2] = {4, 5};
const uint8_t MPR_CH[2] = {0, 1};
const uint8_t PRELOAD_CH[2] = {4, 5};

const float PULSES_PER_LITRE = 1875.0;
const uint32_t FLOW_TIMEOUT_MS = 5000;

// WiFi credentials. If connection fails, ESP32 creates "UBH-MCL".
const char* WIFI_SSID = "REPLACE_WITH_YOUR_SSID";
const char* WIFI_PASSWORD = "REPLACE_WITH_YOUR_PASSWORD";
const char* AP_SSID = "UBH-MCL";

// ============================================================
// OBJECTS / SENSOR DATA
// ============================================================

WebServer server(80);
Adafruit_MPRLS mpr;
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

struct Sensor {
  float value = NAN;
  bool connected = false;
};

Sensor preload[2];
Sensor pressure[2];

volatile uint32_t flowPulses[2] = {0, 0};

float flowRate[2] = {0, 0};
float mprZero[2] = {0, 0};

bool flowSeen[2] = {false, false};
bool flowConnected[2] = {false, false};
bool mprZeroValid[2] = {false, false};

uint32_t lastFlowPulse[2] = {0, 0};
uint32_t packetId = 0;
unsigned long previousTime = 0;

struct MotorData { 
  float voltage = NAN; 
  float current = NAN; 
  float rpm = NAN; 
  bool connected = false; 
  bool seen = false; 
  uint32_t lastRx = 0; 
}; 

MotorData motor; 
// ============================================================
// WEB PAGE
// ============================================================

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UNSW Bionic Hearts - MCL</title>

<style>
*{box-sizing:border-box}
body{margin:0;font-family:Arial,sans-serif;background:#f8f8f8;color:#222}
.container{width:min(1150px,94%);margin:30px auto}
.header{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:28px}
.brand{color:#b00014;font-size:14px;font-weight:700;letter-spacing:1.5px}
h1{font-size:48px;margin:14px 0}
.status{text-align:right;color:#999;font-size:13px}
.live{color:#269b32;font-weight:bold}
.dot{display:inline-block;width:11px;height:11px;background:#7de36a;border-radius:50%;margin-right:6px}
.offline{color:#b00014}.offline .dot{background:#b00014}

.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:15px}
.card{background:#fff;border:1px solid #ddd;border-top:3px solid #b00014;border-radius:15px;padding:25px 22px;min-height:130px}
.title{text-align:center;color:#888;font-size:13px;letter-spacing:1px}
.value{font-size:39px;margin-top:13px}
.unit{font-size:14px;color:#999}
.warn{display:none}
.disconnected{background:#fff5f5}
.disconnected .value{color:#aaa}
.disconnected .warn{display:inline-block;margin-top:8px;padding:6px 10px;border-radius:20px;background:#ffe3e3;color:#b00014;font-size:12px;font-weight:bold}

.chart{background:#fff;border:1px solid #ddd;border-radius:15px;padding:22px;margin-top:20px}
.charthead{display:flex;justify-content:space-between}
.badge{background:#fff0f0;color:#b00014;padding:8px 12px;border-radius:20px;font-size:12px}
canvas{width:100%;height:280px;margin-top:20px}
.legend{display:flex;gap:20px;flex-wrap:wrap;font-size:13px;color:#666;margin-top:10px}
.line{display:inline-block;width:25px;height:3px;margin-right:6px}
.footer{text-align:center;color:#aaa;font-size:13px;padding:20px}

@media(max-width:800px){
  .cards{grid-template-columns:1fr}
  .header{flex-direction:column;gap:15px}
  .status{text-align:left}
  h1{font-size:34px}
}
</style>
</head>

<body>
<div class="container">

<div class="header">
  <div>
    <div class="brand">UNSW BIONIC HEARTS</div>
    <h1>Live MCL Telemetry</h1>
  </div>
  <div class="status">
    <div id="live" class="live"><span class="dot"></span>LIVE</div>
    <div id="time">Waiting for telemetry...</div>
    <div>Packet: <span id="packet">0</span></div>
  </div>
</div>

<div class="cards" id="cards"></div>

<div class="chart">
  <div class="charthead"><b>Blood Flow</b><span class="badge">Last 60 readings</span></div>
  <canvas id="flowChart"></canvas>
  <div class="legend">
    <span><i class="line" style="background:#b00014"></i>Left Flow</span>
    <span><i class="line" style="background:#222"></i>Right Flow</span>
  </div>
</div>

<div class="chart">
  <div class="charthead"><b>Pressure</b><span class="badge">Last 60 readings</span></div>
  <canvas id="pressureChart"></canvas>
  <div class="legend">
    <span><i class="line" style="background:#b00014"></i>Left Preload</span>
    <span><i class="line" style="background:#222"></i>Right Preload</span>
    <span><i class="line" style="background:#2878c8"></i>Pressure 1</span>
    <span><i class="line" style="background:#239b56"></i>Pressure 2</span>
  </div>
</div>

<div class="footer">
  Device UBH_TAH001 • Live demonstration data from the Mock Circulatory Loop
</div>
</div>

<script>
const N=60;

const info={
  left_flow_lpm:["Left Flow","L/min"],
  right_flow_lpm:["Right Flow","L/min"],
  left_inlet_pressure_mmHg:["Left Preload","mmHg"],
  right_inlet_pressure_mmHg:["Right Preload","mmHg"],
  left_upstream_pressure_mmHg:["Left Upstream Pressure","mmHg"],
  right_upstream_pressure_mmHg:["Right Upstream Pressure","mmHg"],
  pump_rpm:["Pump RPM","RPM"], 

  pump_voltage:["Pump Voltage","V"],
  pump_current:["Pump Current","A"]
};

const hist={
  left_flow_lpm:[],right_flow_lpm:[],
  left_inlet_pressure_mmHg:[],right_inlet_pressure_mmHg:[],
  left_upstream_pressure_mmHg:[],right_upstream_pressure_mmHg:[],
  pump_rpm:[], 
  pump_voltage:[],
  pump_current:[]
};

// Create sensor cards
document.getElementById("cards").innerHTML=Object.entries(info).map(([id,x])=>
  `<div class="card" id="${id}Card">
    <div class="title">${x[0].toUpperCase()}</div>
    <div class="value"><span id="${id}Value">--</span> <span class="unit">${x[1]}</span></div>
    <div class="warn">NOT CONNECTED</div>
  </div>`
).join("");

function setCard(id,s){
  const card=document.getElementById(id+"Card");
  const value=document.getElementById(id+"Value");
  const ok=s.connected&&s.value!==null;

  value.innerText=ok?Number(s.value).toFixed(2):"--";
  card.classList.toggle("disconnected",!ok);
}

function addHistory(id,value){
  hist[id].push(value);
  if(hist[id].length>N) hist[id].shift();
}

function draw(id,series,unit){
  const canvas=document.getElementById(id);
  const ctx=canvas.getContext("2d");
  const ratio=window.devicePixelRatio||1;
  const w=canvas.clientWidth,h=canvas.clientHeight;

  canvas.width=w*ratio;
  canvas.height=h*ratio;
  ctx.setTransform(ratio,0,0,ratio,0,0);
  ctx.clearRect(0,0,w,h);

  let values=[];
  series.forEach(([key])=>hist[key].forEach(v=>{
    if(v!==null&&Number.isFinite(v)) values.push(v);
  }));

  if(!values.length){
    ctx.fillStyle="#aaa";
    ctx.font="16px Arial";
    ctx.textAlign="center";
    ctx.fillText("Waiting for sensor readings...",w/2,h/2);
    return;
  }

  let min=Math.min(...values),max=Math.max(...values);
  if(min===max){min--;max++;}

  const range=max-min;
  min-=range*0.1;
  max+=range*0.1;

  const L=55,R=20,T=20,B=35;
  const pw=w-L-R,ph=h-T-B;

  // Grid and vertical scale
  ctx.font="11px Arial";
  ctx.strokeStyle="#eee";
  ctx.fillStyle="#999";

  for(let i=0;i<=5;i++){
    const y=T+ph*i/5;
    ctx.beginPath();
    ctx.moveTo(L,y);
    ctx.lineTo(w-R,y);
    ctx.stroke();

    ctx.textAlign="right";
    ctx.fillText((max-(max-min)*i/5).toFixed(1),L-8,y+4);
  }

  ctx.textAlign="left";
  ctx.fillText(unit,5,12);

  // Data series
  series.forEach(([key,color])=>{
    ctx.strokeStyle=color;
    ctx.lineWidth=2.5;
    ctx.beginPath();

    let started=false;
    const data=hist[key];

    data.forEach((v,i)=>{
      if(v===null||!Number.isFinite(v)){started=false;return;}

      const x=L+i/Math.max(N-1,1)*pw;
      const y=T+(max-v)/(max-min)*ph;

      if(!started){ctx.moveTo(x,y);started=true;}
      else ctx.lineTo(x,y);
    });

    ctx.stroke();
  });
}

function redraw(){
  draw("flowChart",[
    ["left_flow_lpm","#b00014"],
    ["right_flow_lpm","#222"]
  ],"L/min");

  draw("pressureChart",[
    ["left_inlet_pressure_mmHg","#b00014"],
    ["right_inlet_pressure_mmHg","#222"],
    ["left_upstream_pressure_mmHg","#2878c8"],
    ["right_upstream_pressure_mmHg","#239b56"]
  ],"mmHg");

}

async function update(){
  try{
    const r=await fetch("/api/data",{cache:"no-store"});
    if(!r.ok) throw new Error();

    const d=await r.json();

    const live=document.getElementById("live");
    live.className="live";
    live.innerHTML='<span class="dot"></span>LIVE';

    document.getElementById("time").innerText="Last update: "+new Date().toLocaleTimeString();
    document.getElementById("packet").innerText=d.packet;

    Object.keys(info).forEach(id=>{
      setCard(id,d[id]);
      addHistory(id,d[id].connected?d[id].value:null);
    });

    redraw();
  }
  catch{
    const live=document.getElementById("live");
    live.className="live offline";
    live.innerHTML='<span class="dot"></span>OFFLINE';
  }
}

setInterval(update,1000);
update();
window.addEventListener("resize",redraw);
</script>
</body>
</html>
)rawliteral";

// ============================================================
// I2C SENSOR FUNCTIONS
// ============================================================

void selectMux(uint8_t channel){
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

bool i2cPresent(uint8_t channel,uint8_t address){
  selectMux(channel);
  delay(1);
  Wire.beginTransmission(address);
  return Wire.endTransmission()==0;
}

// Read ±1 PSI preload sensor and convert to mmHg.
float readPreload(uint8_t channel){
  constexpr float PMIN=-1.0,PMAX=1.0;
  constexpr uint32_t OUT_MIN=1677722,OUT_MAX=15099494;

  if(!i2cPresent(channel,PRELOAD_ADDR)) return NAN;

  selectMux(channel);
  delay(2);

  Wire.beginTransmission(PRELOAD_ADDR);
  Wire.write(0xAA);
  Wire.write(0x00);
  Wire.write(0x00);
  if(Wire.endTransmission()!=0) return NAN;

  delay(10);
  Wire.requestFrom((uint8_t)PRELOAD_ADDR,(uint8_t)7);
  if(Wire.available()<7) return NAN;

  uint8_t data[7];
  for(uint8_t &b:data) b=Wire.read();

  uint32_t counts=((uint32_t)data[1]<<16)|((uint32_t)data[2]<<8)|data[3];
  float psi=((float)counts-OUT_MIN)*(PMAX-PMIN)/(OUT_MAX-OUT_MIN)+PMIN;

  return psi*51.715;
}

// Read MPRLS and convert hPa to mmHg.
float readMPR(uint8_t channel){
  if(!i2cPresent(channel,MPR_ADDR)) return NAN;

  selectMux(channel);
  delay(2);

  float hPa=mpr.readPressure();
  return isnan(hPa)?NAN:hPa*0.750062;
}

// ============================================================
// FLOW SENSOR INTERRUPTS
// ============================================================

void IRAM_ATTR flow1ISR(){ flowPulses[0]++; }
void IRAM_ATTR flow2ISR(){ flowPulses[1]++; }

// ============================================================
// SENSOR UPDATES
// ============================================================

void updateFlow(uint32_t now,float dt){
  uint32_t pulses[2];

  noInterrupts();
  pulses[0]=flowPulses[0];
  pulses[1]=flowPulses[1];
  flowPulses[0]=flowPulses[1]=0;
  interrupts();

  for(int i=0;i<2;i++){
    if(pulses[i]){
      flowSeen[i]=true;
      lastFlowPulse[i]=now;
    }

    // No pulses for 5 s is treated as "not connected".
    // This cannot distinguish disconnection from genuine zero flow.
    flowConnected[i]=flowSeen[i]&&(now-lastFlowPulse[i]<FLOW_TIMEOUT_MS);
    flowRate[i]=(pulses[i]/PULSES_PER_LITRE)/dt*60.0;
  }
}

void updatePressure(){
  for(int i=0;i<2;i++){
    float v=readPreload(PRELOAD_CH[i]);
    preload[i].connected=!isnan(v);
    preload[i].value=preload[i].connected?v:NAN;

    v=readMPR(MPR_CH[i]);

    if(!isnan(v)){
      // Auto-zero if sensor was not present at startup.
      if(!mprZeroValid[i]){
        mprZero[i]=v;
        mprZeroValid[i]=true;
      }

      pressure[i].value=v-mprZero[i];
      pressure[i].connected=true;
    }else{
      pressure[i].value=NAN;
      pressure[i].connected=false;
    }
  }
}

// ============================================================
// OLED / SERIAL
// ============================================================

void formatLine(char* out,size_t size,const char* label,float value,bool connected,const char* unit){
  if(connected) snprintf(out,size,"%s %.2f %s",label,value,unit);
  else snprintf(out,size,"%s NC",label);
}

void updateOLED(){
  char line[6][26];

  formatLine(line[0],sizeof(line[0]),"Flow1",flowRate[0],flowConnected[0],"L/m");
  formatLine(line[1],sizeof(line[1]),"Flow2",flowRate[1],flowConnected[1],"L/m");
  formatLine(line[2],sizeof(line[2]),"Pre L",preload[0].value,preload[0].connected,"mmHg");
  formatLine(line[3],sizeof(line[3]),"Pre R",preload[1].value,preload[1].connected,"mmHg");
  formatLine(line[4],sizeof(line[4]),"Pres1",pressure[0].value,pressure[0].connected,"mmHg");
  formatLine(line[5],sizeof(line[5]),"Pres2",pressure[1].value,pressure[1].connected,"mmHg");

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tr);

  for(int i=0;i<6;i++) oled.drawStr(0,8+i*10,line[i]);

  oled.sendBuffer();
}

void printSensor(const char* name,float value,bool connected,const char* unit){
  Serial.print(name);
  Serial.print(": ");

  if(connected){
    Serial.print(value,2);
    Serial.print(" ");
    Serial.println(unit);
  }else{
    Serial.println("NOT CONNECTED");
  }
}

void printSensorData(){
  Serial.println("----------------------------------------");
  printSensor("Flow 1",flowRate[0],flowConnected[0],"L/min");
  printSensor("Flow 2",flowRate[1],flowConnected[1],"L/min");
  printSensor("Left Preload",preload[0].value,preload[0].connected,"mmHg");
  printSensor("Right Preload",preload[1].value,preload[1].connected,"mmHg");
  printSensor("MPR 1",pressure[0].value,pressure[0].connected,"mmHg");
  printSensor("MPR 2",pressure[1].value,pressure[1].connected,"mmHg");
}

// ============================================================
// WEB API
// ============================================================

String jsonFloat(float value,unsigned int decimals=2){
  return (isnan(value)||isinf(value))?"null":String(value,decimals);
}

void addSensorJSON(String &json,const char* name,float value,bool connected){
  json+="\""+String(name)+"\":{\"value\":";
  json+=connected?jsonFloat(value):"null";
  json+=",\"connected\":";
  json+=connected?"true":"false";
  json+="}";
}

void handleTelemetry(){
  String json;
  json.reserve(900);

  json="{\"packet\":"+String(packetId)+",\"uptime\":"+String(millis())+",";

  addSensorJSON(json,"left_flow_lpm",flowRate[0],flowConnected[0]); json+=",";
  addSensorJSON(json,"right_flow_lpm",flowRate[1],flowConnected[1]); json+=",";
  addSensorJSON(json,"left_inlet_pressure_mmHg",preload[0].value,preload[0].connected); json+=",";
  addSensorJSON(json,"right_inlet_pressure_mmHg",preload[1].value,preload[1].connected); json+=",";
  addSensorJSON(json,"left_upstream_pressure_mmHg",pressure[0].value,pressure[0].connected); json+=",";
  addSensorJSON(json,"right_upstream_pressure_mmHg",pressure[1].value,pressure[1].connected); json+=","; 
  addSensorJSON(json, "pump_rpm", motor.rpm, motor.connected); json+=","; 
  addSensorJSON(json, "pump_voltage", motor.voltage, motor.connected); json+=",";
  addSensorJSON(json, "pump_current", motor.current, motor.connected);
  json+="}";


  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json",json);
}

// ============================================================
// NETWORK
// ============================================================

void startNetwork(){
  bool configured=strcmp(WIFI_SSID,"REPLACE_WITH_YOUR_SSID")!=0;

  if(configured){
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID,WIFI_PASSWORD);

    uint32_t start=millis();

    while(WiFi.status()!=WL_CONNECTED&&millis()-start<15000){
      Serial.print(".");
      delay(500);
    }

    if(WiFi.status()==WL_CONNECTED){
      Serial.print("\nWiFi connected. Dashboard: http://");
      Serial.println(WiFi.localIP());
      return;
    }

    Serial.println("\nWiFi failed - starting local access point.");
  }

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);

  Serial.print("Connect to WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Dashboard: http://");
  Serial.println(WiFi.softAPIP());
}

void startWebServer(){
  server.on("/",HTTP_GET,[](){
    server.send_P(200,"text/html",PAGE);
  });

  server.on("/api/data",HTTP_GET,handleTelemetry);

  server.onNotFound([](){
    server.send(404,"text/plain","404 - Not Found");
  });

  server.begin();
  Serial.println("Web server started.");

  if(MDNS.begin("mcl")) Serial.println("Also try: http://mcl.local");
}

// ============================================================
// STARTUP
// ============================================================

void initMPR(){
  for(int i=0;i<2;i++){
    selectMux(MPR_CH[i]);

    bool ok=mpr.begin();
    Serial.printf("MPR%d initialisation: %s\n",i+1,ok?"OK":"FAILED");

    float v=readMPR(MPR_CH[i]);

    if(!isnan(v)){
      mprZero[i]=v;
      mprZeroValid[i]=true;
      pressure[i].connected=true;
      Serial.printf("MPR%d zero: %.2f mmHg\n",i+1,v);
    }
  }
}

void showIPAddress(){
  String ip=(WiFi.getMode()==WIFI_MODE_AP)
              ?WiFi.softAPIP().toString()
              :WiFi.localIP().toString();

  oled.clearBuffer();
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(0,10,"Telemetry server:");
  oled.drawStr(0,25,ip.c_str());
  oled.drawStr(0,40,"Open IP in browser");
  oled.sendBuffer();
}

void initCAN() { 
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 64; 

  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  bool ok = twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
  Serial.printf("CAN bus initialisation: %s\n", ok ? "OK" : "FAILED");
}

void readCAN() { 
  twai_message_t m; 

  while(twai_receive(&m , 0) == ESP_OK) { 
    if (m.extd || m.rtr) continue; 

    if (m.identifier == CAN_ID_MOTOR && m.data_length_code == 6) {
      uint16_t v = (uint16_t) (m.data[0] | (m.data[1] << 8));
      int16_t c = (int16_t) (m.data[2] | (m.data[3] << 8));
      uint16_t r = (uint16_t) (m.data[4] | (m.data[5] << 8));

      motor.voltage = v / 100.0f; 
      motor.current = c / 100.0f;
      motor.rpm = r; 
      motor.seen = true; 
      motor.lastRx = millis(); 
    }
  }

  motor.connected = motor.seen && (millis() - motor.lastRx < CAN_TIMEOUT_MS);
}
// ============================================================
// SETUP
// ============================================================

void setup(){
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("UNSW BIONIC HEARTS - MCL TELEMETRY");
  Serial.println("========================================");

  Wire.begin(I2C_SDA,I2C_SCL,I2C_FREQ);

  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_ncenB08_tr);
  oled.drawStr(0,15,"MCL Data");
  oled.drawStr(0,35,"Starting...");
  oled.sendBuffer();

  pinMode(FLOW_PIN[0],INPUT_PULLUP);
  pinMode(FLOW_PIN[1],INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FLOW_PIN[0]),flow1ISR,RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN[1]),flow2ISR,RISING);

  initMPR();
  initCAN(); 
  startNetwork();
  startWebServer();
  showIPAddress();

  delay(3000);
  previousTime=millis();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop(){
  server.handleClient();
  readCAN(); 

  uint32_t now=millis();
  if(now-previousTime<1000) return;

  float dt=(now-previousTime)/1000.0;
  previousTime=now;

  updateFlow(now,dt);
  updatePressure();

  packetId++;
  printSensorData();
  updateOLED();
}
