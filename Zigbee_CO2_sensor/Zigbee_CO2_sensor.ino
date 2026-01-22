#ifndef ZIGBEE_MODE_ED
#error "Zigbee End Device mode is not selected (Tools -> Zigbee mode -> End Device)."
#endif

#include <Wire.h>
#include <Zigbee.h>

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

// ---------- Timing ----------
static constexpr uint32_t SENSOR_POLL_MS = 1000;   // опрашиваем чаще, но читаем только когда ready
static constexpr uint32_t ZB_REPORT_MS   = 30000;

// ---------- Objects ----------
Adafruit_NeoPixel pixels(WS2812_LEDS, WS2812_GPIO, NEO_GRB + NEO_KHZ800);

SensirionI2cScd4x scd4x;

ZigbeeTempSensor zbTempHum(EP_TEMP_HUM);
ZigbeeCarbonDioxideSensor zbCO2(EP_CO2);

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

// ---------- SCD4x init ----------
static bool initScd4x() {
  Wire.begin(I2C_SDA, I2C_SCL);

  scd4x.begin(Wire, SCD4X_I2C_ADDR);

  uint16_t err = 0;
  char errMsg[64];

  // stopPeriodicMeasurement может ругнуться если не был запущен — не фатально
  (void)scd4x.stopPeriodicMeasurement();
  delay(50);

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

      zbCO2.setCarbonDioxide((float)co2ppm);
      zbTempHum.setTemperature(tempC);
      zbTempHum.setHumidity(rh);

      setLedByCO2(co2ppm);

      if (now - lastReport >= ZB_REPORT_MS) {
        lastReport = now;
        zbCO2.report();
        zbTempHum.report();
        Serial.println("Zigbee report sent.");
      }
    }
  }

  delay(20);
}

