#ifndef ZIGBEE_MODE_ED
#error "Zigbee End Device mode is not selected (Tools -> Zigbee mode -> End Device)."
#endif

#include <Wire.h>
#include <Zigbee.h>
#include <math.h>

#include <Adafruit_NeoPixel.h>
#include <SensirionI2cScd4x.h>
#include <SensirionCore.h>   // для errorToString()

// ---------- Pins ----------
#define I2C_SDA 2
#define I2C_SCL 3

#define WS2812_GPIO 27
#define WS2812_LEDS 1

// ---------- SCD4x ----------
static constexpr uint8_t SCD4X_I2C_ADDR = 0x62;

// ---------- Zigbee endpoints ----------
#define EP_TEMP_HUM 10
#define EP_CO2      11
#define EP_LED_DIM  12
#define EP_ALARM    13

// ------------ Common -------------------
static bool g_zclReady = false;
static uint32_t g_connectedAt = 0;

// ------------ LED ----------------------
static bool g_ledEnabled = true;
static uint8_t g_ledLevel100 = 40;  // стартовая яркость в %, 0..100

// ------------- Binary ------------------
static bool g_alarm = false; 


// ------------- For blink ---------------
static uint16_t g_lastCO2 = 0;
static bool g_hasCO2 = false;

// ---------- Timing ----------
static constexpr uint32_t SENSOR_POLL_MS = 1000;   // опрашиваем чаще, но читаем только когда ready
static constexpr uint32_t ZB_REPORT_MS   = 30000;

// ---------- Objects ----------
Adafruit_NeoPixel pixels(WS2812_LEDS, WS2812_GPIO, NEO_GRB + NEO_KHZ800);

SensirionI2cScd4x scd4x;

ZigbeeTempSensor zbTempHum(EP_TEMP_HUM);
ZigbeeCarbonDioxideSensor zbCO2(EP_CO2);
ZigbeeDimmableLight zbLedDim = ZigbeeDimmableLight(EP_LED_DIM);
ZigbeeBinary zbAlarm(EP_ALARM);


// ---------- State ----------
static uint32_t lastPoll = 0;
static uint32_t lastReport = 0;

static const uint8_t button = BOOT_PIN;

// ---------- LED ----------
static void setLedRGB(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

static void setLedByCO2(uint16_t ppm) {
  if (ppm < 800)   { setLedRGB(0, 80, 0); return; }     // green
  if (ppm < 1200)  { setLedRGB(80, 80, 0); return; }    // yellow
  if (ppm < 2000)  { setLedRGB(120, 40, 0); return; }   // orange
  if (ppm < 5000)  { setLedRGB(140, 0, 0); return; }    // red
  setLedRGB(80, 0, 120);                                // purple
}


// LED с “тревогой” (мигание при очень плохом CO2) и повышением яркости
static void updateLedByCO2(uint16_t ppm) {
  if (!g_ledEnabled) {
    pixels.clear();
    pixels.show();
    return;
  }

  // --- 1) максимальная яркость из HA (0..100% -> 0..255) ---
  uint8_t maxBr = (uint8_t)((uint16_t)g_ledLevel100 * 255 / 100);

  // --- 2) экспоненциальная шкала яркости по CO2 ---
  //  - ниже ~800 ppm почти темно
  //  - около 1500 ppm резкий рост
  //  - >= 2000 ppm почти максимум
  const float CO2_MIN = 400.0f;
  const float CO2_MAX = 2000.0f;

  float x;
  if (ppm <= CO2_MIN) {
    x = 0.0f;
  } else if (ppm >= CO2_MAX) {
    x = 1.0f;
  } else {
    x = (float)(ppm - CO2_MIN) / (CO2_MAX - CO2_MIN); // 0..1
  }

  // Экспонента: чем больше gamma, тем резче "вспышка"
  const float gamma = 3.0f;   // 2.0 мягче, 3.0 резко, 4.0 очень резко
  float k = powf(x, gamma);  // 0..1

  // минимальная подсветка, чтобы LED не "пропадал" совсем
  const float k_min = 0.08f; // 8% от max
  k = k_min + (1.0f - k_min) * k;

  uint8_t br = (uint8_t)(maxBr * k);
  pixels.setBrightness(br);

  // --- 3) мигание при >= 3000 ppm ---
  if (ppm >= 3000) {
    bool on = ((millis() / 500) % 2) == 0; // 1 Гц
    if (on) pixels.setPixelColor(0, pixels.Color(160, 0, 0));
    else    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    return;
  }

  // --- 4) ступени цвета ---
  uint8_t r = 0, g = 0, b = 0;

  if (ppm < 800) {
    r = 0;   g = 120; b = 0;   // green
  } else if (ppm < 1200) {
    r = 120; g = 120; b = 0;   // yellow
  } else if (ppm < 2000) {
    r = 160; g = 60;  b = 0;   // orange
  } else {
    r = 160; g = 0;   b = 0;   // red
  }

  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}



static void onLedChange(bool state, uint8_t level) {
  g_ledEnabled = state;
  if (level > 100) level = 100;
  g_ledLevel100 = level;

  // применяем яркость сразу (0..100 -> 0..255)
  uint8_t b = (uint8_t)((uint16_t)g_ledLevel100 * 255 / 100);
  pixels.setBrightness(b);

  // если выключили — гасим
  if (!g_ledEnabled) {
    pixels.clear();
    pixels.show();
  }
}

// ----------- Alarm --------------
static void updateCo2Alarm(uint16_t ppm) {
  if (!g_zclReady) return;

  bool newAlarm = (ppm >= 3000);
  if (newAlarm == g_alarm) return;

  g_alarm = newAlarm;

  zbAlarm.setBinaryInput(g_alarm);
}

// ---------- SCD4x init ----------
static bool initScd4x() {
  Wire.begin(I2C_SDA, I2C_SCL);

  scd4x.begin(Wire, SCD4X_I2C_ADDR);

  uint16_t err = 0;
  char errMsg[64];

  // stopPeriodicMeasurement может ругнуться если не был запущен — не фатально
  (void)scd4x.stopPeriodicMeasurement();
  delay(50);

  // ---- ASC target (fresh air reference) ----
  // Обычно 400 ppm для наружного воздуха
  err = scd4x.setAutomaticSelfCalibrationTarget(400);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x setASC target failed: %s\n", errMsg);
  }

  err = scd4x.startPeriodicMeasurement();
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x startPeriodicMeasurement failed: %s\n", errMsg);
    return false;
  }

  return true;
}

// ---------- SCD4x read ----------
static bool readScd4x(uint16_t &co2ppm, float &tempC, float &rh) {
  int16_t err = 0;
  char errMsg[64];

  bool dataReady = false;
  err = scd4x.getDataReadyStatus(dataReady);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x getDataReadyStatus failed: %s\n", errMsg);
    return false;
  }
  if (!dataReady) return false;

  err = scd4x.readMeasurement(co2ppm, tempC, rh);
  if (err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("SCD4x readMeasurement failed: %s\n", errMsg);
    return false;
  }

  if (co2ppm == 0) return false;
  return true;
}


// ---------- Zigbee reset / manual report ----------
static void handleButton() {
  if (digitalRead(button) != LOW) return;

  delay(100);
  uint32_t start = millis();

  while (digitalRead(button) == LOW) {
    delay(50);
    if (millis() - start > 3000) {
      Serial.println("Factory reset Zigbee + reboot...");
      setLedRGB(120, 0, 0);
      delay(300);
      Zigbee.factoryReset();
      ESP.restart();
    }
  }

  Serial.println("Manual report()");
  zbTempHum.report();
  zbCO2.report();
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(button, INPUT_PULLUP);
  
  pixels.setBrightness(40);  
  pixels.begin();
  pixels.clear();
  pixels.show();

  setLedRGB(0, 0, 60); // boot blue

  if (!initScd4x()) {
    setLedRGB(80, 0, 80); // sensor error purple
  }

  // Zigbee endpoints
  zbTempHum.setManufacturerAndModel("Custom", "ESP32C5_SCD4x");
  zbTempHum.setMinMaxValue(-10, 60);
  zbTempHum.setTolerance(0.2f);
  zbTempHum.addHumiditySensor(0, 100, 1.0f);

  zbCO2.setManufacturerAndModel("Custom", "ESP32C5_SCD4x");
  zbCO2.setMinMaxValue(0, 10000);
  zbCO2.setTolerance(50);
  // Zigbee LED dimmer endpoint (brightness control from HA)
  zbLedDim.setManufacturerAndModel("Custom", "ESP32C5_SCD4x_LED");
  zbLedDim.onLightChange(onLedChange);  // callback state+level :contentReference[oaicite:1]{index=1}

  Zigbee.addEndpoint(&zbAlarm);
  Zigbee.addEndpoint(&zbLedDim);
  Zigbee.addEndpoint(&zbTempHum);
  Zigbee.addEndpoint(&zbCO2);

  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start -> reboot");
    setLedRGB(120, 0, 0);
    delay(500);
    ESP.restart();
  }

  while (!Zigbee.connected()) delay(100);
  Serial.println("Zigbee connected.");
  g_connectedAt = millis();   // отметка времени, дальше подождём

  delay(500);
  if (!g_zclReady && Zigbee.connected() && g_connectedAt && (millis() - g_connectedAt > 2000)) {
    g_zclReady = true;

    // --- Alarm endpoint (теперь lock уже готов) ---
    zbAlarm.addBinaryInput();
    zbAlarm.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_SECURITY_CARBON_DIOXIDE_DETECTION);
    zbAlarm.setBinaryInputDescription("CO2 alarm");
    zbAlarm.setBinaryInput(false);
    zbLedDim.setLight(g_ledEnabled, g_ledLevel100); // безопаснее после ready

    // локально применим яркость/вкл
    onLedChange(g_ledEnabled, g_ledLevel100);
    zbLedDim.restoreLight();
  }

  
  // reporting
  zbTempHum.setReporting(10, 300, 0.2f);
  zbTempHum.setHumidityReporting(10, 300, 1.0f);
  zbCO2.setReporting(0, 30, 0);
  setLedRGB(0, 60, 0); // ready green (до первого CO2)
}

void loop() {
  handleButton();

  const uint32_t now = millis();

  if (now - lastPoll >= SENSOR_POLL_MS) {
    lastPoll = now;

    uint16_t co2ppm = 0;
    float tempC = NAN;
    float rh = NAN;

    if (readScd4x(co2ppm, tempC, rh)) {
      Serial.printf("SCD4x: CO2=%u ppm, T=%.2f C, RH=%.2f %%\n", co2ppm, tempC, rh);
      g_lastCO2 = co2ppm;
      g_hasCO2 = true;
      zbCO2.setCarbonDioxide((float)co2ppm);
      zbTempHum.setTemperature(tempC);
      zbTempHum.setHumidity(rh);
      
      updateLedByCO2(co2ppm);
      updateCo2Alarm(co2ppm);

      if (now - lastReport >= ZB_REPORT_MS) {
        lastReport = now;
        zbCO2.report();
        zbTempHum.report();
        Serial.println("Zigbee report sent.");
      }
    }
  }
  if (g_hasCO2) {
    updateLedByCO2(g_lastCO2);   // будет мигать стабильно, даже если датчик обновляется редко
  }
  delay(20);
}

