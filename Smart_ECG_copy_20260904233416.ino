#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "MAX30100_PulseOximeter.h"

// ======================================================
// FINAL V3.3 AI MODEL
// ======================================================

#include "ecg_model_v3_3.h"


// ======================================================
// OLED
// ======================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);


// ======================================================
// I2C
// ======================================================

#define SDA_PIN 21
#define SCL_PIN 22


// ======================================================
// ECG PINS
// ======================================================

#define ECG_PIN 34

#define ECG_LO_MINUS 25
#define ECG_LO_PLUS 26

#define ECG_SDN 33


// ======================================================
// WIFI
// ======================================================

const char* WIFI_SSID = "abc_ceD3";

const char* WIFI_PASSWORD = "12345678";


// ======================================================
// MQTT
// ======================================================

const char* MQTT_BROKER =
  "test.mosquitto.org";

const int MQTT_PORT = 1883;

const char* MQTT_TOPIC =
  "smart_ecg/demo_esp32_01/data";

WiFiClient espClient;

PubSubClient mqttClient(
  espClient
);


// ======================================================
// MAX30100
// ======================================================

PulseOximeter pox;

bool max30100Available = false;

float spo2Value = 0.0f;

float pulseValue = 0.0f;

unsigned long lastMAXReport = 0;

const unsigned long MAX_REPORT_INTERVAL = 1000;


// ======================================================
// ECG SAMPLING
// ======================================================

#define SAMPLE_RATE 360

#define SAMPLE_PERIOD_US \
  (1000000UL / SAMPLE_RATE)

#define PRE_SAMPLES 90

#define POST_SAMPLES 162

#define BEAT_LENGTH 252


// ======================================================
// R-PEAK DETECTION
// ======================================================

#define INTEGRATION_SIZE 30

#define REFRACTORY_SAMPLES 108


// ======================================================
// RR
// ======================================================

#define RR_BUFFER_SIZE 8


// ======================================================
// V3.3 FEATURES
// ======================================================

#define RF_FEATURE_COUNT 84

#define DOWNSAMPLE 4

#define SHAPE_FEATURE_COUNT 9

#define RR_FEATURE_START 80


// ======================================================
// ECG VARIABLES
// ======================================================

int rawECG = 0;

float filteredECG = 0.0f;


// ======================================================
// FILTER STATE
// ======================================================

float hp_x_prev = 0.0f;

float hp_y_prev = 0.0f;

float lp_y_prev = 0.0f;

const float HP_ALPHA = 0.995f;

const float LP_ALPHA = 0.12f;

float previousFiltered = 0.0f;


// ======================================================
// ENERGY / R-PEAK
// ======================================================

float energyBuffer[
  INTEGRATION_SIZE
];

int energyIndex = 0;

float energySum = 0.0f;

float integratedPrevious = 0.0f;

float integratedPrevious2 = 0.0f;

float signalLevel = 0.0f;

float noiseLevel = 0.0f;

float peakThreshold = 20.0f;


// ======================================================
// R-PEAK TIMING
// ======================================================

unsigned long lastPeakTime = 0;

unsigned long previousPeakTime = 0;

int sampleCounter = 0;


// ======================================================
// HEART RATE
// ======================================================

int heartRate = 0;

unsigned long rrIntervals[
  RR_BUFFER_SIZE
];

int rrIndex = 0;

int rrCount = 0;


// ======================================================
// BEAT BUFFER
// ======================================================

float preBuffer[
  PRE_SAMPLES
];

int preIndex = 0;

int preCount = 0;

float beatBuffer[
  BEAT_LENGTH
];

bool collectingBeat = false;

int postIndex = 0;

bool beatReady = false;


// ======================================================
// PENDING BEAT
//
// We need NEXT RR interval for V3.3.
//
// Therefore a beat is classified only after the
// following R peak has arrived.
//
// ======================================================

float pendingBeat[
  BEAT_LENGTH
];

bool pendingBeatValid = false;

float pendingRRPrev = 0.0f;

float pendingRRLocal = 0.0f;


// ======================================================
// CURRENT RR INFORMATION
// ======================================================

float rrPreviousSeconds = 0.0f;

float rrNextSeconds = 0.0f;

float rrLocalSeconds = 0.0f;


// ======================================================
// RUNNING RECORD NORMALIZATION
//
// V3.3 training used record-level mean/std.
// Live ECG has no future complete record, so we
// maintain a running estimate.
//
// ======================================================

double recordMean = 0.0;

double recordM2 = 0.0;

unsigned long recordCount = 0;


// ======================================================
// V3.3 RF FEATURES
// ======================================================

float rfFeatures[
  RF_FEATURE_COUNT
];


// ======================================================
// FEATURE DEBUG VARIABLES
// ======================================================

float featureRecNormMax = 0.0f;

float featureRecNormMin = 0.0f;

float featureRecNormPTP = 0.0f;

float featureRecNormStd = 0.0f;

float featureRecNormRMS = 0.0f;

float featureRecNormMedian = 0.0f;

float featureRecNormIQR = 0.0f;

float featureShapeMax = 0.0f;

float featureShapeMin = 0.0f;

float featureShapePTP = 0.0f;

float featureMaxSlope = 0.0f;

float featureMeanSlope = 0.0f;

float featureMaxPosition = 0.0f;

float featureMinPosition = 0.0f;

float featureZCR = 0.0f;

float featureMeanAbs = 0.0f;

float featureQRSWidth = 0.0f;


// ======================================================
// AI STATUS
// ======================================================

enum AIStatus
{
  AI_UNKNOWN,
  AI_NORMAL,
  AI_ABNORMAL
};

AIStatus aiStatus = AI_UNKNOWN;

float aiConfidence = 0.0f;

float abnormalProbability = 0.0f;


// ======================================================
// DEPLOYMENT THRESHOLD
// ======================================================

const float ESP32_RF_THRESHOLD = 0.50f;


// ======================================================
// OLED WAVEFORM
// ======================================================

float displayBuffer[128];

int displayWriteIndex = 0;


// ======================================================
// TIMING
// ======================================================

unsigned long lastSampleTime = 0;

unsigned long lastDisplayTime = 0;

unsigned long lastSerialTime = 0;

unsigned long lastMQTTTime = 0;

unsigned long lastMQTTReconnectTime = 0;


const unsigned long DISPLAY_INTERVAL = 100;

const unsigned long SERIAL_INTERVAL = 1000;

const unsigned long MQTT_INTERVAL = 1000;

const unsigned long MQTT_RECONNECT_INTERVAL = 5000;


// ======================================================
// LEAD STATUS
// ======================================================

bool leadOff = true;


// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

void resetSignalProcessing();

float filterECG(float input);

void processECG(float sample);

bool detectRPeak(float sample);

void registerRPeak(float peakSample);

void calculateHeartRate();

void startBeatCollection();

void collectBeat(float sample);

void preparePendingBeat();

void calculateFeatures();

void runAI();

bool leadsConnected();

void updateDisplay();

void drawECGWaveform();

void updateMAX30100();

void printDebug();

void setupWiFi();

void reconnectMQTT();

void publishMQTT();

void printWiFiStatusReason(
  wl_status_t status
);

void updateRecordStatistics(
  float sample
);

float getRecordStd();

float calculateMedian(
  float* data,
  int length
);

float calculatePercentile(
  float* data,
  int length,
  float percentile
);

float safeDivide(
  float numerator,
  float denominator
);


// ======================================================
// MAX30100 CALLBACK
// ======================================================

void onBeatDetected()
{
  Serial.println(
    "MAX30100: Beat detected"
  );
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println(
    "=============================================="
  );

  Serial.println(
    " SMART ECG MONITORING SYSTEM"
  );

  Serial.println(
    " V3.3 AI + AD8232 + MAX30100 + MQTT"
  );

  Serial.println(
    "=============================================="
  );


  // ====================================================
  // ECG
  // ====================================================

  pinMode(
    ECG_PIN,
    INPUT
  );

  pinMode(
    ECG_LO_MINUS,
    INPUT
  );

  pinMode(
    ECG_LO_PLUS,
    INPUT
  );

  pinMode(
    ECG_SDN,
    OUTPUT
  );

  digitalWrite(
    ECG_SDN,
    HIGH
  );


  // ====================================================
  // ADC
  // ====================================================

  analogReadResolution(12);

  analogSetPinAttenuation(
    ECG_PIN,
    ADC_11db
  );


  // ====================================================
  // I2C
  // ====================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);


  // ====================================================
  // OLED
  // ====================================================

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDRESS
    )
  )
  {
    Serial.println(
      "ERROR: OLED not found!"
    );

    while (1)
    {
      delay(1000);
    }
  }


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  display.setCursor(
    20,
    8
  );

  display.println(
    "SMART ECG"
  );


  display.setCursor(
    15,
    22
  );

  display.println(
    "AI MODEL V3.3"
  );


  display.setCursor(
    12,
    38
  );

  display.println(
    "Initializing..."
  );


  display.display();

  delay(1000);


  // ====================================================
  // MAX30100
  // ====================================================

  Serial.println();

  Serial.println(
    "Initializing MAX30100..."
  );


  if (
    pox.begin()
  )
  {
    max30100Available = true;

    Serial.println(
      "MAX30100: SUCCESS"
    );


    pox.setIRLedCurrent(
      MAX30100_LED_CURR_7_6MA
    );


    pox.setOnBeatDetectedCallback(
      onBeatDetected
    );
  }

  else
  {
    max30100Available = false;

    Serial.println(
      "MAX30100: FAILED"
    );

    Serial.println(
      "ECG will continue normally."
    );
  }


  // ====================================================
  // DISPLAY BUFFER
  // ====================================================

  for (
    int i = 0;
    i < 128;
    i++
  )
  {
    displayBuffer[i] = 0;
  }


  // ====================================================
  // RESET
  // ====================================================

  resetSignalProcessing();


  // ====================================================
  // MQTT
  // ====================================================

  mqttClient.setServer(
    MQTT_BROKER,
    MQTT_PORT
  );


  // ====================================================
  // WIFI
  // ====================================================

  setupWiFi();


  // ====================================================
  // TIMERS
  // ====================================================

  lastSampleTime = micros();

  lastDisplayTime = millis();

  lastSerialTime = millis();

  lastMQTTTime = millis();

  lastMQTTReconnectTime = millis();

  lastMAXReport = millis();


  // ====================================================
  // READY
  // ====================================================

  Serial.println();

  Serial.println(
    "V3.3 MODEL INFORMATION"
  );

  Serial.println(
    "Trees    : 400"
  );

  Serial.println(
    "Features : 84"
  );

  Serial.println(
    "Threshold: 0.50"
  );

  Serial.println(
    "Window   : 252 samples"
  );

  Serial.println(
    "Sampling : 360 Hz"
  );

  Serial.println();

  Serial.println(
    "SYSTEM READY"
  );

  Serial.println(
    "=============================================="
  );
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
  // ====================================================
  // MAX30100
  // ====================================================

  if (
    max30100Available
  )
  {
    pox.update();
  }


  // ====================================================
  // ECG SAMPLING
  // ====================================================

  unsigned long currentMicros =
    micros();


  while (
    (long)(
      currentMicros -
      lastSampleTime
    ) >= 0
  )
  {
    lastSampleTime +=
      SAMPLE_PERIOD_US;


    // --------------------------------------------------
    // LEAD STATUS
    // --------------------------------------------------

    leadOff =
      !leadsConnected();


    // --------------------------------------------------
    // ECG
    // --------------------------------------------------

    if (leadOff)
    {
      resetSignalProcessing();
    }

    else
    {
      rawECG =
        analogRead(
          ECG_PIN
        );


      filteredECG =
        filterECG(
          (float)rawECG
        );


      // Running record normalization statistics
      updateRecordStatistics(
        filteredECG
      );


      processECG(
        filteredECG
      );


      // OLED waveform
      displayBuffer[
        displayWriteIndex
      ] =
        filteredECG;


      displayWriteIndex++;


      if (
        displayWriteIndex >= 128
      )
      {
        displayWriteIndex = 0;
      }
    }


    currentMicros =
      micros();
  }


  // ====================================================
  // MILLIS TASKS
  // ====================================================

  unsigned long currentMillis =
    millis();


  // ====================================================
  // MAX30100
  // ====================================================

  if (
    currentMillis -
    lastMAXReport >=
    MAX_REPORT_INTERVAL
  )
  {
    lastMAXReport =
      currentMillis;

    updateMAX30100();
  }


  // ====================================================
  // OLED
  // ====================================================

  if (
    currentMillis -
    lastDisplayTime >=
    DISPLAY_INTERVAL
  )
  {
    lastDisplayTime =
      currentMillis;

    updateDisplay();
  }


  // ====================================================
  // SERIAL
  // ====================================================

  if (
    currentMillis -
    lastSerialTime >=
    SERIAL_INTERVAL
  )
  {
    lastSerialTime =
      currentMillis;

    printDebug();
  }


  // ====================================================
  // MQTT
  // ====================================================

  reconnectMQTT();

  mqttClient.loop();


  // ====================================================
  // MQTT PUBLISH
  // ====================================================

  if (
    currentMillis -
    lastMQTTTime >=
    MQTT_INTERVAL
  )
  {
    lastMQTTTime =
      currentMillis;

    publishMQTT();
  }
}


// ======================================================
// RECORD STATISTICS
// WELFORD ONLINE MEAN / STD
// ======================================================

void updateRecordStatistics(
  float sample
)
{
  recordCount++;


  double delta =
    (double)sample -
    recordMean;


  recordMean +=
    delta /
    (double)recordCount;


  double delta2 =
    (double)sample -
    recordMean;


  recordM2 +=
    delta *
    delta2;
}


// ======================================================
// RECORD STD
// ======================================================

float getRecordStd()
{
  if (
    recordCount < 2
  )
  {
    return 1.0f;
  }


  double variance =
    recordM2 /
    (double)recordCount;


  if (
    variance < 1e-12
  )
  {
    variance = 1e-12;
  }


  return sqrt(
    (float)variance
  );
}


// ======================================================
// MAX30100 UPDATE
// ======================================================

void updateMAX30100()
{
  if (
    !max30100Available
  )
  {
    spo2Value = 0;

    pulseValue = 0;

    return;
  }


  float newPulse =
    pox.getHeartRate();


  uint8_t newSpO2 =
    pox.getSpO2();


  if (
    newPulse > 0 &&
    newPulse >= 30 &&
    newPulse <= 220
  )
  {
    pulseValue =
      newPulse;
  }


  if (
    newSpO2 > 0 &&
    newSpO2 <= 100
  )
  {
    spo2Value =
      newSpO2;
  }
}


// ======================================================
// WIFI STATUS
// ======================================================

void printWiFiStatusReason(
  wl_status_t status
)
{
  switch (status)
  {
    case WL_IDLE_STATUS:

      Serial.println(
        "Status: IDLE"
      );

      break;


    case WL_NO_SSID_AVAIL:

      Serial.println(
        "Status: SSID NOT FOUND"
      );

      break;


    case WL_CONNECT_FAILED:

      Serial.println(
        "Status: CONNECT FAILED"
      );

      break;


    case WL_CONNECTION_LOST:

      Serial.println(
        "Status: CONNECTION LOST"
      );

      break;


    case WL_DISCONNECTED:

      Serial.println(
        "Status: DISCONNECTED"
      );

      break;


    default:

      Serial.print(
        "Status code: "
      );

      Serial.println(
        (int)status
      );

      break;
  }
}


// ======================================================
// WIFI SETUP
// ======================================================

void setupWiFi()
{
  Serial.println();

  Serial.print(
    "Connecting to WiFi: "
  );

  Serial.println(
    WIFI_SSID
  );


  WiFi.disconnect(
    true,
    true
  );

  delay(200);


  WiFi.mode(
    WIFI_STA
  );


  WiFi.setSleep(
    false
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  const int MAX_ATTEMPTS = 3;

  bool connected = false;


  for (
    int attempt = 1;
    attempt <= MAX_ATTEMPTS &&
    !connected;
    attempt++
  )
  {
    Serial.print(
      "Attempt "
    );

    Serial.print(
      attempt
    );

    Serial.print(
      " of "
    );

    Serial.println(
      MAX_ATTEMPTS
    );


    unsigned long startTime =
      millis();


    while (
      WiFi.status() !=
        WL_CONNECTED &&
      millis() -
        startTime < 15000
    )
    {
      delay(500);

      Serial.print(
        "."
      );
    }


    Serial.println();


    if (
      WiFi.status() ==
      WL_CONNECTED
    )
    {
      connected = true;

      break;
    }


    printWiFiStatusReason(
      WiFi.status()
    );


    if (
      attempt < MAX_ATTEMPTS
    )
    {
      Serial.println(
        "Retrying..."
      );


      WiFi.disconnect(
        true,
        true
      );

      delay(500);


      WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
      );
    }
  }


  if (connected)
  {
    Serial.println(
      "WiFi Connected!"
    );


    Serial.print(
      "IP Address: "
    );

    Serial.println(
      WiFi.localIP()
    );
  }

  else
  {
    Serial.println(
      "WiFi connection failed."
    );

    Serial.println(
      "ECG will continue normally."
    );
  }
}


// ======================================================
// MQTT RECONNECT
// ======================================================

void reconnectMQTT()
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    if (
      millis() -
      lastMQTTReconnectTime <
      MQTT_RECONNECT_INTERVAL
    )
    {
      return;
    }


    lastMQTTReconnectTime =
      millis();


    Serial.println(
      "WiFi not connected. Retrying WiFi..."
    );


    WiFi.disconnect(
      true,
      true
    );

    delay(200);


    WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD
    );


    return;
  }


  if (
    mqttClient.connected()
  )
  {
    return;
  }


  if (
    millis() -
    lastMQTTReconnectTime <
    MQTT_RECONNECT_INTERVAL
  )
  {
    return;
  }


  lastMQTTReconnectTime =
    millis();


  Serial.print(
    "Connecting to MQTT... "
  );


  uint64_t chipID =
    ESP.getEfuseMac();


  char clientID[40];


  snprintf(
    clientID,
    sizeof(clientID),
    "SmartECG_%04X%08X",
    (uint16_t)(chipID >> 32),
    (uint32_t)chipID
  );


  if (
    mqttClient.connect(
      clientID
    )
  )
  {
    Serial.println(
      "Connected!"
    );


    Serial.print(
      "MQTT Topic: "
    );

    Serial.println(
      MQTT_TOPIC
    );
  }

  else
  {
    Serial.print(
      "Failed, state="
    );

    Serial.println(
      mqttClient.state()
    );
  }
}


// ======================================================
// MQTT PUBLISH
// ======================================================

void publishMQTT()
{
  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return;
  }


  if (
    !mqttClient.connected()
  )
  {
    return;
  }


  const char* warning;

  if (
    leadOff
  )
  {
    warning = "LEADS_OFF";
  }

  else if (
    aiStatus ==
    AI_ABNORMAL
  )
  {
    warning = "RECHECK";
  }

  else if (
    aiStatus ==
    AI_NORMAL
  )
  {
    warning = "OK";
  }

  else
  {
    warning = "WAIT";
  }


  const char* aiText;


  if (
    leadOff
  )
  {
    aiText = "WAIT";
  }

  else if (
    aiStatus ==
    AI_NORMAL
  )
  {
    aiText = "NORMAL";
  }

  else if (
    aiStatus ==
    AI_ABNORMAL
  )
  {
    aiText = "ABNORMAL";
  }

  else
  {
    aiText = "WAIT";
  }


  char payload[400];


  snprintf(
    payload,
    sizeof(payload),

    "{\"hr\":%d,"
    "\"pulse\":%.0f,"
    "\"spo2\":%.0f,"
    "\"lead\":\"%s\","
    "\"ai\":\"%s\","
    "\"confidence\":%.0f,"
    "\"warning\":\"%s\"}",

    heartRate,

    pulseValue,

    spo2Value,

    leadOff
      ? "OFF"
      : "ON",

    aiText,

    leadOff
      ? 0.0f
      : aiConfidence,

    warning
  );


  bool success =
    mqttClient.publish(
      MQTT_TOPIC,
      payload
    );


  if (success)
  {
    Serial.print(
      "MQTT PUBLISHED: "
    );

    Serial.println(
      payload
    );
  }

  else
  {
    Serial.println(
      "MQTT publish failed."
    );
  }
}


// ======================================================
// ECG FILTER
// ======================================================

float filterECG(
  float input
)
{
  float hp =
    HP_ALPHA *
    (
      hp_y_prev +
      input -
      hp_x_prev
    );


  hp_x_prev =
    input;

  hp_y_prev =
    hp;


  float lp =
    LP_ALPHA *
      hp +
    (1.0f - LP_ALPHA) *
      lp_y_prev;


  lp_y_prev =
    lp;


  return lp;
}


// ======================================================
// RESET SIGNAL PROCESSING
// ======================================================

void resetSignalProcessing()
{
  previousFiltered = 0;

  energyIndex = 0;

  energySum = 0;

  integratedPrevious = 0;

  integratedPrevious2 = 0;

  signalLevel = 0;

  noiseLevel = 0;

  peakThreshold = 20;

  lastPeakTime = 0;

  previousPeakTime = 0;

  sampleCounter = 0;

  heartRate = 0;

  rrIndex = 0;

  rrCount = 0;

  preIndex = 0;

  preCount = 0;

  collectingBeat = false;

  postIndex = 0;

  beatReady = false;

  pendingBeatValid = false;

  pendingRRPrev = 0;

  pendingRRLocal = 0;

  rrPreviousSeconds = 0;

  rrNextSeconds = 0;

  rrLocalSeconds = 0;

  abnormalProbability = 0;

  aiStatus =
    AI_UNKNOWN;

  aiConfidence = 0;


  // Reset record normalization
  recordMean = 0;

  recordM2 = 0;

  recordCount = 0;


  for (
    int i = 0;
    i < INTEGRATION_SIZE;
    i++
  )
  {
    energyBuffer[i] = 0;
  }


  for (
    int i = 0;
    i < PRE_SAMPLES;
    i++
  )
  {
    preBuffer[i] = 0;
  }


  for (
    int i = 0;
    i < RR_BUFFER_SIZE;
    i++
  )
  {
    rrIntervals[i] = 0;
  }


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    beatBuffer[i] = 0;

    pendingBeat[i] = 0;
  }


  for (
    int i = 0;
    i < RF_FEATURE_COUNT;
    i++
  )
  {
    rfFeatures[i] = 0;
  }
}


// ======================================================
// ECG PROCESSING
// ======================================================

void processECG(
  float sample
)
{
  bool peakDetected =
    detectRPeak(
      sample
    );


  if (
    peakDetected
  )
  {
    registerRPeak(
      sample
    );
  }


  if (
    collectingBeat
  )
  {
    collectBeat(
      sample
    );
  }


  if (
    beatReady
  )
  {
    beatReady = false;

    preparePendingBeat();
  }


  // ----------------------------------------------------
  // PRE BUFFER
  // ----------------------------------------------------

  preBuffer[
    preIndex
  ] =
    sample;


  preIndex++;


  if (
    preIndex >=
    PRE_SAMPLES
  )
  {
    preIndex = 0;
  }


  if (
    preCount <
    PRE_SAMPLES
  )
  {
    preCount++;
  }
}


// ======================================================
// R-PEAK DETECTION
// ======================================================

bool detectRPeak(
  float sample
)
{
  float derivative =
    sample -
    previousFiltered;


  previousFiltered =
    sample;


  float energy =
    derivative *
    derivative;


  energySum -=
    energyBuffer[
      energyIndex
    ];


  energyBuffer[
    energyIndex
  ] =
    energy;


  energySum +=
    energy;


  energyIndex++;


  if (
    energyIndex >=
    INTEGRATION_SIZE
  )
  {
    energyIndex = 0;
  }


  float integrated =
    energySum /
    INTEGRATION_SIZE;


  // ----------------------------------------------------
  // SIGNAL LEVEL
  // ----------------------------------------------------

  if (
    integrated >
    signalLevel
  )
  {
    signalLevel =
      0.95f *
      signalLevel +
      0.05f *
      integrated;
  }

  else
  {
    signalLevel =
      0.995f *
      signalLevel +
      0.005f *
      integrated;
  }


  // ----------------------------------------------------
  // NOISE
  // ----------------------------------------------------

  if (
    integrated <
    peakThreshold
  )
  {
    noiseLevel =
      0.995f *
      noiseLevel +
      0.005f *
      integrated;
  }


  // ----------------------------------------------------
  // THRESHOLD
  // ----------------------------------------------------

  peakThreshold =
    noiseLevel +
    0.45f *
    (
      signalLevel -
      noiseLevel
    );


  if (
    peakThreshold < 10
  )
  {
    peakThreshold = 10;
  }


  // ----------------------------------------------------
  // LOCAL MAXIMUM
  // ----------------------------------------------------

  bool localMaximum =
    integratedPrevious >
      integratedPrevious2 &&
    integratedPrevious >=
      integrated;


  bool thresholdCrossed =
    integratedPrevious >
    peakThreshold;


  bool refractoryOK =
    (
      sampleCounter -
      lastPeakTime
    ) >=
    REFRACTORY_SAMPLES;


  bool validPeak =
    localMaximum &&
    thresholdCrossed &&
    refractoryOK &&
    integratedPrevious >
      noiseLevel * 1.5f;


  integratedPrevious2 =
    integratedPrevious;


  integratedPrevious =
    integrated;


  sampleCounter++;


  if (
    validPeak
  )
  {
    lastPeakTime =
      sampleCounter - 1;

    return true;
  }


  return false;
}


// ======================================================
// REGISTER R-PEAK
// ======================================================

void registerRPeak(
  float peakSample
)
{
  unsigned long currentPeakTime =
    micros();


  float currentRRSeconds = 0.0f;


  // ----------------------------------------------------
  // RR interval
  // ----------------------------------------------------

  if (
    previousPeakTime != 0
  )
  {
    unsigned long rr =
      currentPeakTime -
      previousPeakTime;


    if (
      rr >= 300000 &&
      rr <= 2000000
    )
    {
      currentRRSeconds =
        (float)rr /
        1000000.0f;


      rrIntervals[
        rrIndex
      ] =
        rr;


      rrIndex++;


      if (
        rrIndex >=
        RR_BUFFER_SIZE
      )
      {
        rrIndex = 0;
      }


      if (
        rrCount <
        RR_BUFFER_SIZE
      )
      {
        rrCount++;
      }


      calculateHeartRate();
    }
  }


  // ----------------------------------------------------
  // IMPORTANT:
  //
  // Current peak gives NEXT RR for the previous beat.
  // ----------------------------------------------------

  if (
    pendingBeatValid &&
    currentRRSeconds > 0.0f &&
    pendingRRLocal > 0.0f
  )
  {
    rrNextSeconds =
      currentRRSeconds;


    calculateFeatures();

    runAI();


    pendingBeatValid =
      false;
  }


  // ----------------------------------------------------
  // Current peak becomes beginning of new beat
  // ----------------------------------------------------

  previousPeakTime =
    currentPeakTime;


  startBeatCollection();


  if (
    integratedPrevious >
    signalLevel
  )
  {
    signalLevel =
      0.90f *
      signalLevel +
      0.10f *
      integratedPrevious;
  }
}


// ======================================================
// HEART RATE
// ======================================================

void calculateHeartRate()
{
  if (
    rrCount == 0
  )
  {
    heartRate = 0;

    return;
  }


  unsigned long total = 0;


  for (
    int i = 0;
    i < rrCount;
    i++
  )
  {
    total +=
      rrIntervals[i];
  }


  float averageRR =
    (float)total /
    (float)rrCount;


  if (
    averageRR <= 0
  )
  {
    heartRate = 0;

    return;
  }


  float bpm =
    60000000.0f /
    averageRR;


  if (
    bpm >= 30 &&
    bpm <= 200
  )
  {
    heartRate =
      (int)(
        bpm + 0.5f
      );
  }
}


// ======================================================
// START BEAT COLLECTION
// ======================================================

void startBeatCollection()
{
  if (
    preCount <
    PRE_SAMPLES
  )
  {
    return;
  }


  collectingBeat = true;

  postIndex = 0;


  // ----------------------------------------------------
  // OLD 90 SAMPLES BEFORE R PEAK
  // ----------------------------------------------------

  for (
    int i = 0;
    i < PRE_SAMPLES;
    i++
  )
  {
    int index =
      preIndex + i;


    if (
      index >=
      PRE_SAMPLES
    )
    {
      index -=
        PRE_SAMPLES;
    }


    beatBuffer[i] =
      preBuffer[index];
  }


  // ----------------------------------------------------
  // R PEAK
  // ----------------------------------------------------

  beatBuffer[
    PRE_SAMPLES
  ] =
    filteredECG;


  postIndex = 1;
}


// ======================================================
// COLLECT BEAT
// ======================================================

void collectBeat(
  float sample
)
{
  if (
    !collectingBeat
  )
  {
    return;
  }


  if (
    postIndex <
    POST_SAMPLES
  )
  {
    beatBuffer[
      PRE_SAMPLES +
      postIndex
    ] =
      sample;


    postIndex++;
  }


  if (
    postIndex >=
    POST_SAMPLES
  )
  {
    collectingBeat =
      false;


    beatReady = true;
  }
}


// ======================================================
// PREPARE PENDING BEAT
// ======================================================

void preparePendingBeat()
{
  // ----------------------------------------------------
  // Copy completed beat
  // ----------------------------------------------------

  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    pendingBeat[i] =
      beatBuffer[i];
  }


  // ----------------------------------------------------
  // Previous RR
  //
  // At this point the RR ending at the current R peak
  // is available.
  // ----------------------------------------------------

  if (
    rrCount >= 1
  )
  {
    int latestIndex =
      rrIndex - 1;


    if (
      latestIndex < 0
    )
    {
      latestIndex =
        RR_BUFFER_SIZE - 1;
    }


    pendingRRPrev =
      (float)
      rrIntervals[
        latestIndex
      ] /
      1000000.0f;


    pendingRRLocal =
      pendingRRPrev;
  }

  else
  {
    pendingRRPrev =
      0.0f;

    pendingRRLocal =
      0.0f;
  }


  // ----------------------------------------------------
  // Wait for next R peak to obtain next RR.
  // ----------------------------------------------------

  if (
    pendingRRLocal > 0.0f
  )
  {
    pendingBeatValid = true;
  }
}


// ======================================================
// SAFE DIVISION
// ======================================================

float safeDivide(
  float numerator,
  float denominator
)
{
  if (
    fabs(denominator) < 1e-8f
  )
  {
    return 0.0f;
  }


  return numerator /
         denominator;
}


// ======================================================
// MEDIAN
// ======================================================

float calculateMedian(
  float* data,
  int length
)
{
  static float temp[
    BEAT_LENGTH
  ];


  for (
    int i = 0;
    i < length;
    i++
  )
  {
    temp[i] =
      data[i];
  }


  // ----------------------------------------------------
  // Insertion sort
  // ----------------------------------------------------

  for (
    int i = 1;
    i < length;
    i++
  )
  {
    float key =
      temp[i];


    int j =
      i - 1;


    while (
      j >= 0 &&
      temp[j] > key
    )
    {
      temp[j + 1] =
        temp[j];

      j--;
    }


    temp[j + 1] =
      key;
  }


  if (
    length % 2 == 0
  )
  {
    int mid =
      length / 2;


    return (
      temp[mid - 1] +
      temp[mid]
    ) * 0.5f;
  }


  return temp[
    length / 2
  ];
}


// ======================================================
// PERCENTILE
// ======================================================

float calculatePercentile(
  float* data,
  int length,
  float percentile
)
{
  static float temp[
    BEAT_LENGTH
  ];


  for (
    int i = 0;
    i < length;
    i++
  )
  {
    temp[i] =
      data[i];
  }


  // ----------------------------------------------------
  // Sort
  // ----------------------------------------------------

  for (
    int i = 1;
    i < length;
    i++
  )
  {
    float key =
      temp[i];


    int j =
      i - 1;


    while (
      j >= 0 &&
      temp[j] > key
    )
    {
      temp[j + 1] =
        temp[j];

      j--;
    }


    temp[j + 1] =
      key;
  }


  float position =
    (percentile / 100.0f) *
    (float)(length - 1);


  int lower =
    (int)floor(position);


  int upper =
    (int)ceil(position);


  if (
    lower < 0
  )
  {
    lower = 0;
  }


  if (
    upper >= length
  )
  {
    upper =
      length - 1;
  }


  if (
    lower == upper
  )
  {
    return temp[lower];
  }


  float fraction =
    position -
    (float)lower;


  return
    temp[lower] +
    fraction *
    (
      temp[upper] -
      temp[lower]
    );
}


// ======================================================
// V3.3 FEATURE EXTRACTION
// ======================================================
//
// Exact feature layout:
//
// 0  RecNorm Max
// 1  RecNorm Min
// 2  RecNorm PTP
// 3  RecNorm Std
// 4  RecNorm RMS
// 5  RecNorm Median
// 6  RecNorm IQR
//
// 7  Shape Max
// 8  Shape Min
// 9  Shape PTP
// 10 Max Slope
// 11 Mean Slope
// 12 Max Position
// 13 Min Position
// 14 Zero Crossing Rate
// 15 Mean Abs Amplitude
// 16 QRS Width
//
// 17-79 Shape[0]...Shape[248]
//
// 80 RR Prev / Local
// 81 RR Next / Local
// 82 RR Prev / Next
// 83 RR Diff / Local
//
// ======================================================

void calculateFeatures()
{
  // ----------------------------------------------------
  // Local arrays
  // ----------------------------------------------------

  static float recNormBeat[
    BEAT_LENGTH
  ];

  static float beatNorm[
    BEAT_LENGTH
  ];


  // ----------------------------------------------------
  // Record normalization
  // ----------------------------------------------------

  float recMean =
    (float)recordMean;


  float recStd =
    getRecordStd();


  if (
    recStd < 1e-6f
  )
  {
    recStd = 1.0f;
  }


  // ----------------------------------------------------
  // Create record-normalized beat
  // ----------------------------------------------------

  float sumRec = 0.0f;

  float squareRec = 0.0f;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    float x =
      pendingBeat[i];


    float normalized =
      (
        x -
        recMean
      ) /
      (
        recStd +
        1e-8f
      );


    recNormBeat[i] =
      normalized;


    sumRec +=
      normalized;


    squareRec +=
      normalized *
      normalized;
  }


  // ----------------------------------------------------
  // RecNorm features
  // ----------------------------------------------------

  float recNormMean =
    sumRec /
    BEAT_LENGTH;


  float variance =
    0.0f;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    float d =
      recNormBeat[i] -
      recNormMean;


    variance +=
      d * d;
  }


  variance /=
    BEAT_LENGTH;


  if (
    variance < 0
  )
  {
    variance = 0;
  }


  float recNormStd =
    sqrt(variance);


  float recNormMax =
    recNormBeat[0];

  float recNormMin =
    recNormBeat[0];


  for (
    int i = 1;
    i < BEAT_LENGTH;
    i++
  )
  {
    if (
      recNormBeat[i] >
      recNormMax
    )
    {
      recNormMax =
        recNormBeat[i];
    }


    if (
      recNormBeat[i] <
      recNormMin
    )
    {
      recNormMin =
        recNormBeat[i];
    }
  }


  float recNormPTP =
    recNormMax -
    recNormMin;


  float recNormRMS =
    sqrt(
      squareRec /
      BEAT_LENGTH
    );


  float recNormMedian =
    calculateMedian(
      recNormBeat,
      BEAT_LENGTH
    );


  float q25 =
    calculatePercentile(
      recNormBeat,
      BEAT_LENGTH,
      25.0f
    );


  float q75 =
    calculatePercentile(
      recNormBeat,
      BEAT_LENGTH,
      75.0f
    );


  float recNormIQR =
    q75 - q25;


  // ----------------------------------------------------
  // Store features 0-6
  // ----------------------------------------------------

  rfFeatures[0] =
    recNormMax;

  rfFeatures[1] =
    recNormMin;

  rfFeatures[2] =
    recNormPTP;

  rfFeatures[3] =
    recNormStd;

  rfFeatures[4] =
    recNormRMS;

  rfFeatures[5] =
    recNormMedian;

  rfFeatures[6] =
    recNormIQR;


  // ----------------------------------------------------
  // Beat shape normalization
  //
  // bn = (beat - beat.mean()) /
  //      (beat.std() + 1e-8)
  // ----------------------------------------------------

  float beatMean =
    0.0f;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    beatMean +=
      recNormBeat[i];
  }


  beatMean /=
    BEAT_LENGTH;


  float beatVariance =
    0.0f;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    float d =
      recNormBeat[i] -
      beatMean;


    beatVariance +=
      d * d;
  }


  beatVariance /=
    BEAT_LENGTH;


  float beatStd =
    sqrt(
      beatVariance
    );


  if (
    beatStd < 1e-8f
  )
  {
    beatStd = 1.0f;
  }


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    beatNorm[i] =
      (
        recNormBeat[i] -
        beatMean
      ) /
      (
        beatStd +
        1e-8f
      );
  }


  // ----------------------------------------------------
  // Shape features
  // ----------------------------------------------------

  float shapeMax =
    beatNorm[0];

  float shapeMin =
    beatNorm[0];

  float maxAbsSlope =
    0.0f;

  float slopeSum =
    0.0f;


  int maxPosition =
    0;

  int minPosition =
    0;


  float absSum =
    0.0f;


  int zeroCrossings =
    0;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i++
  )
  {
    float x =
      beatNorm[i];


    if (
      x > shapeMax
    )
    {
      shapeMax =
        x;

      maxPosition =
        i;
    }


    if (
      x < shapeMin
    )
    {
      shapeMin =
        x;

      minPosition =
        i;
    }


    absSum +=
      fabs(x);


    if (
      i > 0
    )
    {
      float slope =
        beatNorm[i] -
        beatNorm[i - 1];


      float absSlope =
        fabs(slope);


      slopeSum +=
        absSlope;


      if (
        absSlope >
        maxAbsSlope
      )
      {
        maxAbsSlope =
          absSlope;
      }


      if (
        (
          beatNorm[i - 1] < 0 &&
          beatNorm[i] >= 0
        )
        ||
        (
          beatNorm[i - 1] >= 0 &&
          beatNorm[i] < 0
        )
      )
      {
        zeroCrossings++;
      }
    }
  }


  float shapePTP =
    shapeMax -
    shapeMin;


  float meanSlope =
    slopeSum /
    (BEAT_LENGTH - 1);


  float maxPositionNorm =
    (float)maxPosition /
    (float)BEAT_LENGTH;


  float minPositionNorm =
    (float)minPosition /
    (float)BEAT_LENGTH;


  float zcr =
    (float)zeroCrossings /
    (float)BEAT_LENGTH;


  float meanAbsAmplitude =
    absSum /
    BEAT_LENGTH;


  // ----------------------------------------------------
  // QRS width
  //
  // Python:
  //
  // centre = BEFORE = 90
  // seg = abs(
  // beat[50:130] - median(beat)
  // )
  // threshold = 0.5 * max(seg)
  // ----------------------------------------------------

  float shapeMedian =
    calculateMedian(
      recNormBeat,
      BEAT_LENGTH
    );


  float qrsMax =
    0.0f;


  for (
    int i = 50;
    i < 130;
    i++
  )
  {
    float value =
      fabs(
        recNormBeat[i] -
        shapeMedian
      );


    if (
      value >
      qrsMax
    )
    {
      qrsMax =
        value;
    }
  }


  float qrsThreshold =
    0.5f *
    qrsMax;


  int qrsSamples =
    0;


  for (
    int i = 50;
    i < 130;
    i++
  )
  {
    float value =
      fabs(
        recNormBeat[i] -
        shapeMedian
      );


    if (
      value >
      qrsThreshold
    )
    {
      qrsSamples++;
    }
  }


  float qrsWidthMs =
    (
      (float)qrsSamples /
      (float)SAMPLE_RATE
    ) *
    1000.0f;


  // ----------------------------------------------------
  // Store shape features 7-16
  // ----------------------------------------------------

  rfFeatures[7] =
    shapeMax;

  rfFeatures[8] =
    shapeMin;

  rfFeatures[9] =
    shapePTP;

  rfFeatures[10] =
    maxAbsSlope;

  rfFeatures[11] =
    meanSlope;

  rfFeatures[12] =
    maxPositionNorm;

  rfFeatures[13] =
    minPositionNorm;

  rfFeatures[14] =
    zcr;

  rfFeatures[15] =
    meanAbsAmplitude;

  rfFeatures[16] =
    qrsWidthMs;


  // ----------------------------------------------------
  // Shape[0] ... Shape[248]
  //
  // Python:
  // wave = list(bn[::4])
  //
  // 252 / 4 = 63 samples
  //
  // IMPORTANT:
  // V3.3 feature list contains 63 waveform values,
  // therefore features 17-79.
  //
  // ----------------------------------------------------

  int featureIndex =
    17;


  for (
    int i = 0;
    i < BEAT_LENGTH;
    i += DOWNSAMPLE
  )
  {
    if (
      featureIndex >= 80
    )
    {
      break;
    }


    rfFeatures[
      featureIndex
    ] =
      beatNorm[i];


    featureIndex++;
  }


  // ----------------------------------------------------
  // Safety fill
  // ----------------------------------------------------

  while (
    featureIndex < 80
  )
  {
    rfFeatures[
      featureIndex
    ] = 0.0f;

    featureIndex++;
  }


  // ----------------------------------------------------
  // RR FEATURES
  // ----------------------------------------------------

  float rrPrev =
    pendingRRPrev;


  float rrNext =
    rrNextSeconds;


  float rrLocal =
    pendingRRLocal;


  if (
    rrLocal <= 0.0f
  )
  {
    rrLocal =
      rrPrev;
  }


  if (
    rrLocal <= 0.0f ||
    rrNext <= 0.0f
  )
  {
    rfFeatures[80] = 1.0f;

    rfFeatures[81] = 1.0f;

    rfFeatures[82] = 1.0f;

    rfFeatures[83] = 0.0f;
  }

  else
  {
    rfFeatures[80] =
      safeDivide(
        rrPrev,
        rrLocal
      );


    rfFeatures[81] =
      safeDivide(
        rrNext,
        rrLocal
      );


    rfFeatures[82] =
      safeDivide(
        rrPrev,
        rrNext
      );


    rfFeatures[83] =
      fabs(
        rrPrev -
        rrNext
      ) /
      rrLocal;
  }


  // ----------------------------------------------------
  // Debug feature values
  // ----------------------------------------------------

  featureRecNormMax =
    rfFeatures[0];

  featureRecNormMin =
    rfFeatures[1];

  featureRecNormPTP =
    rfFeatures[2];

  featureRecNormStd =
    rfFeatures[3];

  featureRecNormRMS =
    rfFeatures[4];

  featureRecNormMedian =
    rfFeatures[5];

  featureRecNormIQR =
    rfFeatures[6];

  featureShapeMax =
    rfFeatures[7];

  featureShapeMin =
    rfFeatures[8];

  featureShapePTP =
    rfFeatures[9];

  featureMaxSlope =
    rfFeatures[10];

  featureMeanSlope =
    rfFeatures[11];

  featureMaxPosition =
    rfFeatures[12];

  featureMinPosition =
    rfFeatures[13];

  featureZCR =
    rfFeatures[14];

  featureMeanAbs =
    rfFeatures[15];

  featureQRSWidth =
    rfFeatures[16];
}


// ======================================================
// AI
// ======================================================

void runAI()
{
  // ----------------------------------------------------
  // Basic validity
  // ----------------------------------------------------

  if (
    heartRate == 0
  )
  {
    aiStatus =
      AI_UNKNOWN;

    aiConfidence = 0;

    abnormalProbability = 0;

    return;
  }


  // ----------------------------------------------------
  // 84-feature Random Forest
  // ----------------------------------------------------

  float probability =
    rf_predict_proba(
      rfFeatures
    );


  // ----------------------------------------------------
  // Clamp
  // ----------------------------------------------------

  if (
    probability < 0.0f
  )
  {
    probability = 0.0f;
  }


  if (
    probability > 1.0f
  )
  {
    probability = 1.0f;
  }


  abnormalProbability =
    probability;


  // ----------------------------------------------------
  // Confidence
  // ----------------------------------------------------

  aiConfidence =
    probability *
    100.0f;


  // ----------------------------------------------------
  // Final threshold
  // ----------------------------------------------------

  if (
    probability >=
    ESP32_RF_THRESHOLD
  )
  {
    aiStatus =
      AI_ABNORMAL;
  }

  else
  {
    aiStatus =
      AI_NORMAL;
  }
}


// ======================================================
// LEAD DETECTION
// ======================================================

bool leadsConnected()
{
  int loMinus =
    digitalRead(
      ECG_LO_MINUS
    );


  int loPlus =
    digitalRead(
      ECG_LO_PLUS
    );


  if (
    loMinus == HIGH ||
    loPlus == HIGH
  )
  {
    return false;
  }


  return true;
}


// ======================================================
// OLED
// ======================================================

void updateDisplay()
{
  display.clearDisplay();

  display.setTextSize(1);

  display.setTextColor(
    SSD1306_WHITE
  );


  // ----------------------------------------------------
  // TITLE
  // ----------------------------------------------------

  display.setCursor(
    0,
    0
  );

  display.print(
    "SMART ECG V3.3"
  );


  // ----------------------------------------------------
  // HR
  // ----------------------------------------------------

  display.setCursor(
    0,
    10
  );

  display.print(
    "HR:"
  );


  if (
    !leadOff &&
    heartRate > 0
  )
  {
    display.print(
      heartRate
    );

    display.print(
      " BPM"
    );
  }

  else
  {
    display.print(
      "-- BPM"
    );
  }


  // ----------------------------------------------------
  // LEAD
  // ----------------------------------------------------

  display.setCursor(
    100,
    10
  );


  if (
    leadOff
  )
  {
    display.print(
      "OFF"
    );
  }

  else
  {
    display.print(
      "ON"
    );
  }


  // ----------------------------------------------------
  // MAX30100
  // ----------------------------------------------------

  display.setCursor(
    0,
    20
  );

  display.print(
    "P:"
  );


  if (
    pulseValue > 0
  )
  {
    display.print(
      (int)pulseValue
    );
  }

  else
  {
    display.print(
      "--"
    );
  }


  display.print(
    " O2:"
  );


  if (
    spo2Value > 0
  )
  {
    display.print(
      (int)spo2Value
    );

    display.print(
      "%"
    );
  }

  else
  {
    display.print(
      "--"
    );
  }


  // ----------------------------------------------------
  // AI
  // ----------------------------------------------------

  display.setCursor(
    0,
    29
  );

  display.print(
    "AI:"
  );


  if (
    leadOff
  )
  {
    display.print(
      "WAIT"
    );
  }

  else if (
    aiStatus ==
    AI_NORMAL
  )
  {
    display.print(
      "NORMAL"
    );
  }

  else if (
    aiStatus ==
    AI_ABNORMAL
  )
  {
    display.print(
      "ABNORMAL"
    );
  }

  else
  {
    display.print(
      "WAIT"
    );
  }


  // ----------------------------------------------------
  // CONFIDENCE
  // ----------------------------------------------------

  if (
    !leadOff &&
    aiStatus !=
      AI_UNKNOWN
  )
  {
    display.setCursor(
      92,
      29
    );

    display.print(
      (int)aiConfidence
    );

    display.print(
      "%"
    );
  }


  // ----------------------------------------------------
  // SEPARATOR
  // ----------------------------------------------------

  display.drawLine(
    0,
    38,
    127,
    38,
    SSD1306_WHITE
  );


  // ----------------------------------------------------
  // WAVEFORM
  // ----------------------------------------------------

  drawECGWaveform();


  // ----------------------------------------------------
  // BOTTOM
  // ----------------------------------------------------

  display.setCursor(
    0,
    56
  );


  if (
    leadOff
  )
  {
    display.print(
      "CONNECT ECG"
    );
  }

  else if (
    aiStatus ==
    AI_ABNORMAL
  )
  {
    display.print(
      "RECHECK ECG"
    );
  }

  else if (
    aiStatus ==
    AI_NORMAL
  )
  {
    display.print(
      "MONITORING"
    );
  }

  else
  {
    display.print(
      "PROCESSING..."
    );
  }


  display.display();
}


// ======================================================
// OLED ECG WAVEFORM
// ======================================================

void drawECGWaveform()
{
  float minValue =
    displayBuffer[0];

  float maxValue =
    displayBuffer[0];


  for (
    int i = 1;
    i < 128;
    i++
  )
  {
    if (
      displayBuffer[i] <
      minValue
    )
    {
      minValue =
        displayBuffer[i];
    }


    if (
      displayBuffer[i] >
      maxValue
    )
    {
      maxValue =
        displayBuffer[i];
    }
  }


  float range =
    maxValue -
    minValue;


  if (
    range < 20
  )
  {
    range = 20;
  }


  for (
    int x = 0;
    x < 127;
    x++
  )
  {
    int index1 =
      (
        displayWriteIndex +
        x
      ) % 128;


    int index2 =
      (
        displayWriteIndex +
        x +
        1
      ) % 128;


    int y1 =
      39 +
      (int)(
        (
          maxValue -
          displayBuffer[index1]
        )
        * 15.0f /
        range
      );


    int y2 =
      39 +
      (int)(
        (
          maxValue -
          displayBuffer[index2]
        )
        * 15.0f /
        range
      );


    y1 =
      constrain(
        y1,
        39,
        54
      );


    y2 =
      constrain(
        y2,
        39,
        54
      );


    display.drawLine(
      x,
      y1,
      x + 1,
      y2,
      SSD1306_WHITE
    );
  }
}


// ======================================================
// SERIAL DEBUG
// ======================================================

void printDebug()
{
  Serial.print(
    "RAW="
  );

  Serial.print(
    rawECG
  );


  Serial.print(
    " FILTER="
  );

  Serial.print(
    filteredECG,
    2
  );


  Serial.print(
    " ECG_HR="
  );

  Serial.print(
    heartRate
  );


  Serial.print(
    " LEAD="
  );


  Serial.print(
    leadOff
      ? "OFF"
      : "ON"
  );


  // ----------------------------------------------------
  // MAX30100
  // ----------------------------------------------------

  Serial.print(
    " PULSE="
  );

  Serial.print(
    pulseValue,
    1
  );


  Serial.print(
    " SPO2="
  );

  Serial.print(
    spo2Value,
    1
  );


  Serial.print(
    " MAX30100="
  );

  Serial.print(
    max30100Available
      ? "OK"
      : "FAIL"
  );


  // ----------------------------------------------------
  // MODEL
  // ----------------------------------------------------

  Serial.print(
    " RF_FEATURES=84"
  );


  Serial.print(
    " RF_TREES=400"
  );


  // ----------------------------------------------------
  // AI
  // ----------------------------------------------------

  Serial.print(
    " AI="
  );


  if (
    leadOff
  )
  {
    Serial.print(
      "WAIT"
    );
  }

  else if (
    aiStatus ==
    AI_NORMAL
  )
  {
    Serial.print(
      "NORMAL"
    );
  }

  else if (
    aiStatus ==
    AI_ABNORMAL
  )
  {
    Serial.print(
      "ABNORMAL"
    );
  }

  else
  {
    Serial.print(
      "WAIT"
    );
  }


  Serial.print(
    " PROB="
  );

  Serial.print(
    abnormalProbability,
    3
  );


  Serial.print(
    " CONF="
  );

  Serial.print(
    aiConfidence,
    1
  );


  // ----------------------------------------------------
  // RR
  // ----------------------------------------------------

  Serial.print(
    " RRprev="
  );

  Serial.print(
    pendingRRPrev,
    3
  );


  Serial.print(
    " RRnext="
  );

  Serial.print(
    rrNextSeconds,
    3
  );


  // ----------------------------------------------------
  // RECORD NORMALIZATION
  // ----------------------------------------------------

  Serial.print(
    " REC_STD="
  );

  Serial.print(
    getRecordStd(),
    3
  );


  // ----------------------------------------------------
  // WIFI
  // ----------------------------------------------------

  Serial.print(
    " WIFI="
  );

  Serial.print(
    WiFi.status() ==
      WL_CONNECTED
      ? "ON"
      : "OFF"
  );


  // ----------------------------------------------------
  // MQTT
  // ----------------------------------------------------

  Serial.print(
    " MQTT="
  );

  Serial.println(
    mqttClient.connected()
      ? "ON"
      : "OFF"
  );
}