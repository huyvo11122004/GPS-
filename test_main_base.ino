#include <HardwareSerial.h>
#include <TinyGPS++.h>

// -------------------------------------------------
// CẤU HÌNH CHÂN
// -------------------------------------------------
#define RX_SIM     16
#define TX_SIM     17
#define RX_GPS     26
#define TX_GPS     27
#define RELAY      36
#define PWRKEY     4
#define LED_PIN    18
#define BUZZER_PIN 19
#define WATER_PIN  RELAY
#define POWER_PIN 32
// -------------------------------------------------
// BIẾN TOÀN CỤC
// -------------------------------------------------
HardwareSerial moduleSim(2);
HardwareSerial gpsSerial(1);
TinyGPSPlus    gps;

String bufferModuleSim = "";
String lastLBSData = "";

String phoneNumber = "+84379316901";

//String phoneNumber = "+84787525329";

bool waitingForLBS  = false;
String waitingPhone = "";
bool waitingForDebug = false;

const unsigned long GPS_TIMEOUT = 10000;

// === CẢNH BÁO ===
bool alertMode = false;
unsigned long previousMillis = 0;
const long interval = 500;
bool buzzerState = false;

// === CỨU HỘ TỰ ĐỘNG ===
bool rescueMode = false;
unsigned long lastSOSTime = 0;
const unsigned long SOS_INTERVAL = 5UL * 60UL * 1000UL;

// === CHỐNG NHẦU & TỰ ĐỘNG RESET ===
unsigned long waterStartTime = 0;
unsigned long waterDryTime = 0;
const unsigned long WATER_DEBOUNCE = 2000;   // 2 giây chạm nước
const unsigned long DRY_RESET_TIME = 7000;  // 10 giây khô → reset

bool pendingSOS = false;
String sosLat = "", sosLng = "";

// -------------------------------------------------
// SETUP
// -------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(POWER_PIN,OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pinMode(WATER_PIN, INPUT);  // GPIO36 + 10kΩ pull-up

  pinMode(PWRKEY, OUTPUT);
  digitalWrite(PWRKEY, HIGH); delay(1000);
  digitalWrite(PWRKEY, LOW);  delay(1500);
  digitalWrite(PWRKEY, HIGH); delay(5000);

  moduleSim.begin(115200, SERIAL_8N1, RX_SIM, TX_SIM);
  gpsSerial.begin(9600,  SERIAL_8N1, RX_GPS, TX_GPS);
  delay(2000);

  setupModuleSim();
  checkNetwork();

  Serial.println("=== AO PHAO CUU HO ===");
}

// -------------------------------------------------
// LOOP
// -------------------------------------------------
void loop() {
  digitalWrite(POWER_PIN, HIGH);
  delay(10);
  
  // === 1. ĐÈN + CÒI ===
  if (alertMode) {
    digitalWrite(LED_PIN, HIGH);
    unsigned long current = millis();
    if (current - previousMillis >= interval) {
      previousMillis = current;
      buzzerState = !buzzerState;
      buzzerState ? tone(BUZZER_PIN, 1000) : noTone(BUZZER_PIN);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }

  // === 2. ĐỌC CẢM BIẾN NƯỚC ===
  bool waterDetected = (digitalRead(WATER_PIN) == LOW);

  Serial.print("Cam bien nuoc: ");
  Serial.println(waterDetected ? "CO NUOC" : "KHO");

  // === 3. XỬ LÝ CHẠM NƯỚC (DEBOUNCE) ===
  if (waterDetected) {
    if (waterStartTime == 0) {
      waterStartTime = millis();
      waterDryTime = 0;
    } else if (millis() - waterStartTime >= WATER_DEBOUNCE && !rescueMode) {
      Serial.println(">>> CO NUOC XAC NHAN! KICH HOAT CUU HO");
      rescueMode = true;
      alertMode = true;
      pendingSOS = true;

      String lat, lng;
      if (readGPS(lat, lng)) {
        sendSOSMessage(lat, lng);
        pendingSOS = false;
      } else {
        Serial.println("GPS chua fix -> chuyen LBS");
        getLBSLocation();
      }
    }
  } else {
    // CẢM BIẾN KHÔ
    waterStartTime = 0;
    if (rescueMode && waterDryTime == 0) {
      waterDryTime = millis();
    } else if (rescueMode && millis() - waterDryTime >= DRY_RESET_TIME) {
      // KHÔ 10 GIÂY → TỰ ĐỘNG TẮT
      Serial.println(">>> CAM BIEN KHO 7 GIAY - TAT HE THONG!");
      rescueMode = false;
      alertMode = false;
      pendingSOS = false;
      waterDryTime = 0;
    }
  }

  // === 4. GỬI CẬP NHẬT 5 PHÚT (CHỈ KHI ĐANG CỨU HỘ) ===
  if (rescueMode && (millis() - lastSOSTime >= SOS_INTERVAL)) {
    sendLocationUpdate();
  }

  // === 5. XỬ LÝ SMS & LBS ===
  if (moduleSim.available()) {
    bufferModuleSim = "";
    unsigned long start = millis();
    while (millis() - start < 2000 && moduleSim.available()) {
      bufferModuleSim += (char)moduleSim.read();
      delay(1);
    }
    bufferModuleSim.trim();
    Serial.println("SIM: [" + bufferModuleSim + "]");

    if (bufferModuleSim.startsWith("+CLBS:")) {
      lastLBSData = bufferModuleSim;
      String data = parseLBSData(lastLBSData);

      if (pendingSOS && data.startsWith("Toa do")) {
        int c1 = data.indexOf(": ") + 2;
        int c2 = data.indexOf(",");
        String lat = data.substring(c1, c2);
        String lng = data.substring(c2 + 1);
        sendSOSMessage(lat, lng);
        pendingSOS = false;
      }

      if (waitingForLBS || waitingForDebug) {
        if (waitingForLBS) {
          sendSMS(waitingPhone, data);
          waitingForLBS = false;  waitingPhone = "";
        }
        if (waitingForDebug) {
          Serial.println("LBS Debug: " + data);
          waitingForDebug = false;
        }
      }
    }

    if (bufferModuleSim.indexOf("+CMT:") >= 0) {
      int p1 = bufferModuleSim.indexOf("\"");
      int p2 = bufferModuleSim.indexOf("\"", p1 + 1);
      String from = (p1 != -1 && p2 != -1) ? bufferModuleSim.substring(p1 + 1, p2) : "";
      int nl = bufferModuleSim.lastIndexOf("\r\n") + 2;
      String msg = (nl >= 2) ? bufferModuleSim.substring(nl) : "";
      msg.trim();

      if (from == phoneNumber) {
        msg.toLowerCase();
        if (msg == "chk") {
          waitingForLBS = true;  waitingPhone = from;
          getLocation();
        }
        else if (msg == "lbs") {
          waitingForDebug = true;
          getLocation();
        }
        else if (msg == "del") {
          deleteAllSMS(); sendSMS(from, "Deleted all SMS");
        }
        else if (msg == "sos1") {
          alertMode = true;
          sendSMS(from, "SOS_ON: LED + Buzzer dang hoat dong!");
        }
        else if (msg == "sos0") {
          alertMode = false;
          digitalWrite(LED_PIN, LOW);
          noTone(BUZZER_PIN);
          sendSMS(from, "SOS_OFF: LED + Buzzer da tat!");
        }
        else if (msg == "hello") {
          sendSMS(from, "Hello User!");
        }
      }
    }
  }

  while (Serial.available()) moduleSim.write(Serial.read());

  delay(100);
}

// -------------------------------------------------
// GỬI SOS + TỌA ĐỘ
// -------------------------------------------------
void sendSOSMessage(String lat, String lng) {
  String msg = "SOS! NGUOI DA ROI XUONG NUOC!\n"
               "Toa do: " + lat + "," + lng;
  sendSMS(phoneNumber, msg);
  lastSOSTime = millis();
  Serial.println("DA GUI SOS: " + lat + "," + lng);
}

// -------------------------------------------------
// GỬI CẬP NHẬT 5 PHÚT
// -------------------------------------------------
void sendLocationUpdate() {
  Serial.println("=== CAP NHAT VI TRI (5 PHUT) ===");
  String lat, lng;

  if (readGPS(lat, lng)) {
    String msg = "CAP NHAT VI TRI (5 PHUT):\nToa do: " + lat + "," + lng;
    sendSMS(phoneNumber, msg);
  } else {
    String msg = "CAP NHAT VI TRI:\nGPS chua fix. Dang lay LBS...";
    sendSMS(phoneNumber, msg);
    getLBSLocation();
  }
  lastSOSTime = millis();
}

// -------------------------------------------------
// ĐỌC GPS
// -------------------------------------------------
bool readGPS(String &lat, String &lng) {
  unsigned long start = millis();
  while (millis() - start < GPS_TIMEOUT) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.location.isValid()) {
        lat = String(gps.location.lat(), 6);
        lng = String(gps.location.lng(), 6);
        return true;
      }
    }
  }
  return false;
}

// -------------------------------------------------
// LẤY LBS
// -------------------------------------------------
void getLBSLocation() {
  if (!isNetworkReady()) return;
  configureLBS();
  sendATCommand("AT+CLBS=4", "OK", 5000);
}

// -------------------------------------------------
// LẤY VỊ TRÍ CHO LỆNH
// -------------------------------------------------
void getLocation() {
  if (!isNetworkReady()) {
    String err = "Loi: Mang khong san sang";
    if (waitingForLBS)  { sendSMS(waitingPhone, err); waitingForLBS = false; }
    if (waitingForDebug) Serial.println(err);
    return;
  }

  String lat, lng;
  if (readGPS(lat, lng)) {
    String data = "Toa do: " + lat + "," + lng;
    if (waitingForLBS)  { sendSMS(waitingPhone, data); waitingForLBS = false; waitingPhone = ""; }
    if (waitingForDebug) Serial.println("GPS Debug: " + data);
    return;
  }

  getLBSLocation();
}

// -------------------------------------------------
// HÀM HỖ TRỢ (GIỮ NGUYÊN)
// -------------------------------------------------
bool isNetworkReady() {
  String creg = sendATCommand("AT+CREG?", "OK", 2000);
  if (creg.indexOf("+CREG: 0,1") == -1 && creg.indexOf("+CREG: 0,5") == -1 &&
      creg.indexOf("+CREG: 1,1") == -1 && creg.indexOf("+CREG: 1,5") == -1)
    return false;

  String csq = sendATCommand("AT+CSQ", "OK", 2000);
  int rssi = parseCSQ(csq);
  if (rssi < 10 || rssi == 99) return false;

  return true;
}

String parseLBSData(String d) {
  if (d.startsWith("+CLBS: 0")) {
    int c1 = d.indexOf(","), c2 = d.indexOf(",", c1 + 1);
    if (c1 != -1 && c2 != -1) {
      String lat = d.substring(c1 + 1, c2);
      String lng = d.substring(c2 + 1, d.indexOf(",", c2 + 1));
      return "Toa do: " + lat + "," + lng;
    }
  }
  return "Loi LBS";
}

void deleteAllSMS() { sendATCommand("AT+CMGD=1,4", "OK", 5000); }

String sendATCommand(String cmd, const String &exp, unsigned long t) {
  while (moduleSim.available()) moduleSim.read();
  moduleSim.println(cmd);
  String r = "";
  unsigned long s = millis();
  while (millis() - s < t) {
    while (moduleSim.available()) r += (char)moduleSim.read();
    if (r.indexOf(exp) != -1) break;
    delay(10);
  }
  Serial.println("AT: " + cmd + " -> [" + r + "]");
  return r;
}

void configureLBS() {
  sendATCommand("AT+CGDCONT=1,\"IP\",\"v-internet\"", "OK", 2000);
  sendATCommand("AT+CNMP=38", "OK", 2000);
  sendATCommand("AT+CGACT=1,1", "OK", 5000);
  sendATCommand("AT+CLBSCFG=0,2", "OK", 2000);
}

int parseCSQ(String r) {
  int i = r.indexOf("+CSQ: ");
  if (i == -1) return 99;
  i += 6;
  int c = r.indexOf(",", i);
  return (c == -1) ? 99 : r.substring(i, c).toInt();
}

void setupModuleSim() {
  sendATCommand("ATE0", "OK", 2000);
  sendATCommand("AT+IPR=115200", "OK", 2000);
  sendATCommand("AT+CMGF=1", "OK", 2000);
  sendATCommand("AT+CLIP=1", "OK", 2000);
  sendATCommand("AT+CNMI=2,2,0,0,0", "OK", 2000);
  sendATCommand("AT+CFUN=1", "OK", 3000);
}

void checkNetwork() {
  sendATCommand("AT", "OK", 2000);
  sendATCommand("AT+CPIN?", "READY", 2000);
}

void sendSMS(String to, String txt) {
  Serial.println("SMS -> " + to + ": " + txt);
  sendATCommand("AT+CMGF=1", "OK", 2000);
  moduleSim.println("AT+CMGS=\"" + to + "\"");
  delay(500);
  moduleSim.print(txt);
  delay(500);
  moduleSim.write(26);
  sendATCommand("", "OK", 20000);
}