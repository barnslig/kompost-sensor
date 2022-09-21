#include <Arduino.h>
#include <CayenneLPP.h>
#include <lmic.h>
#include <hal/hal.h>
#include <arduino_lmic_hal_boards.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <driver/rtc_io.h>

#include "lorawan-keys.h"

#include "Display.h"

#include "KompostApp.h"

#include "HeltecBatterySensor.h"
#include "TemperatureSensor.h"

constexpr bool kLoRaWANEnabled = true;
constexpr dr_t kLoRaWANDrTx = DR_SF12;
constexpr bool kLoRaWANAdrEnabled = true;
constexpr uint8_t kLoRaWANFPort = 10;
constexpr uint8_t kLoRaWANMaxPayloadSize = 51;

constexpr uint16_t kDisplayTimeoutSeconds = 10;
constexpr uint16_t kDisplayUpdateIntervalSeconds = 1;
constexpr uint16_t kWorkIntervalSeconds = 3 * 60 * 60;

constexpr gpio_num_t kPinPrgButton = GPIO_NUM_0;

constexpr uint8_t kPinBattery = 37;
constexpr uint8_t kPinBatteryDrain = 21;

constexpr uint8_t kPinDisplayReset = 16;
constexpr uint8_t kPinDisplayClock = 15;
constexpr uint8_t kPinDisplayData = 4;

/**
 * Pin at which the DS18B20 data pin is connected
 */
constexpr uint8_t kPinTempSensors = 13;

/**
 * Maximum number of temperature sensors on the bus
 */
constexpr uint8_t kTempSensorsMax = 3;

Preferences preferences;

HardwareSerial &serial = Serial;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, kPinDisplayReset, kPinDisplayClock, kPinDisplayData);
Display display(&u8g2);

HeltecBatterySensor batterySensor(kPinBattery, kPinBatteryDrain);

OneWire oneWire(kPinTempSensors);
DallasTemperature dallasTemp(&oneWire);
TemperatureSensor tempSensors(&dallasTemp, kTempSensorsMax);

CayenneLPP lpp(kLoRaWANMaxPayloadSize);

KompostState kompostState;
KompostApp app(&kompostState, &batterySensor, &tempSensors, &lpp);

int lastDisplayUpdate = 0;
bool displayEnabled = true;

bool shouldSleep = false;

static const char *const lmicEventNames[] = {LMIC_EVENT_NAME_TABLE__INIT};

static osjob_t doWorkJob;

static const u1_t PROGMEM DEVEUI[8] = {OTAA_DEVEUI};
static const u1_t PROGMEM APPEUI[8] = {OTAA_APPEUI};
static const u1_t PROGMEM APPKEY[16] = {OTAA_APPKEY};

// Below callbacks are used by LMIC for reading above values.
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

void printHex2(unsigned v)
{
  v &= 0xff;
  if (v < 16)
  {
    serial.print('0');
  }
  serial.print(v, HEX);
}

void printNetworkInfo(u4_t networkId,
                      devaddr_t deviceAddress,
                      u1_t networkSessionKey[16],
                      u1_t applicationSessionKey[16])
{
  serial.print("netid: ");
  serial.println(networkId, DEC);

  serial.print("devaddr: ");
  serial.println(deviceAddress, HEX);

  serial.print("AppSKey: ");
  for (size_t i = 0; i < sizeof(uint8_t) * 16; ++i)
  {
    if (i != 0)
    {
      serial.print("-");
    }
    printHex2(applicationSessionKey[i]);
  }

  serial.println();

  serial.print("NwkSKey: ");
  for (size_t i = 0; i < sizeof(uint8_t) * 16; ++i)
  {
    if (i != 0)
    {
      serial.print("-");
    }
    printHex2(networkSessionKey[i]);
  }
  serial.println();
}

void onLmicEvent(void *pUserData, ev_t ev)
{
  kompostState.lastLmicEvent = lmicEventNames[ev];
  display.print(&kompostState);

  switch (ev)
  {
  case EV_JOINING:
    serial.println(lmicEventNames[ev]);
    LMIC_setDrTxpow(DR_SF10, 14);
    break;

  case EV_JOINED:
  {
    serial.println(lmicEventNames[ev]);

    u4_t networkId = 0;
    devaddr_t deviceAddress = 0;
    u1_t networkSessionKey[16];
    u1_t applicationSessionKey[16];

    LMIC_getSessionKeys(&networkId, &deviceAddress, networkSessionKey, applicationSessionKey);

    preferences.putUInt("netid", networkId);
    preferences.putUInt("devaddr", deviceAddress);
    preferences.putBytes("appskey", applicationSessionKey, sizeof(uint8_t) * 16);
    preferences.putBytes("nwkskey", networkSessionKey, sizeof(uint8_t) * 16);

    preferences.putUInt("seqnoUp", LMIC.seqnoUp);
    preferences.putUInt("seqnoDn", LMIC.seqnoDn);

    serial.print(F("seqnoUp: "));
    serial.println(LMIC.seqnoUp);

    serial.print(F("seqnoDn: "));
    serial.println(LMIC.seqnoDn);

    printNetworkInfo(networkId,
                     deviceAddress,
                     networkSessionKey,
                     applicationSessionKey);

    break;
  }

  case EV_TXSTART:
    serial.println(lmicEventNames[ev]);
    preferences.putUInt("seqnoUp", LMIC.seqnoUp);

    serial.print(F("seqnoUp: "));
    serial.println(LMIC.seqnoUp);

    break;

  case EV_TXCOMPLETE:
    serial.println(lmicEventNames[ev]);
    preferences.putUInt("seqnoUp", LMIC.seqnoUp);

    serial.print(F("seqnoUp: "));
    serial.println(LMIC.seqnoUp);

    shouldSleep = true;

    break;

  case EV_RXCOMPLETE:
    serial.println(lmicEventNames[ev]);
    preferences.putUInt("seqnoDn", LMIC.seqnoDn);

    serial.print(F("seqnoDn: "));
    serial.println(LMIC.seqnoDn);

    break;

  case EV_JOIN_TXCOMPLETE:
  case EV_TXCANCELED:
  case EV_SCAN_TIMEOUT:
  case EV_BEACON_FOUND:
  case EV_BEACON_MISSED:
  case EV_BEACON_TRACKED:
  case EV_JOIN_FAILED:
  case EV_REJOIN_FAILED:
  case EV_LOST_TSYNC:
  case EV_RESET:
  case EV_LINK_DEAD:
  case EV_LINK_ALIVE:
    serial.println(lmicEventNames[ev]);
    break;
  }
}

void doWork(osjob_t *j)
{
  app.update();
  display.print(&kompostState);

  // Skip processWork if using OTAA and still joining.
  if (LMIC.devaddr == 0)
  {
    serial.println(F("Uplink not scheduled because still joining."));
    return;
  }

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    serial.println(F("Uplink not scheduled because TxRx pending"));
  }
  else
  {
    LMIC_setTxData2(kLoRaWANFPort, lpp.getBuffer(), lpp.getSize(), 0);
  }
}

void goToSleep()
{
  const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((kWorkIntervalSeconds * 1000)));
  if (!kLoRaWANEnabled || !timeCriticalJobs && !(LMIC.opmode & OP_TXRXPEND))
  {
    esp_sleep_enable_ext0_wakeup(kPinPrgButton, 0);
    esp_sleep_enable_timer_wakeup(kWorkIntervalSeconds * 1000000ULL);

    serial.println(F("enter deep sleep"));

    esp_deep_sleep_start();
  }
}

void setup()
{
  serial.begin(115200);

  preferences.begin("lorawan");

  batterySensor.begin();
  tempSensors.begin();

  display.begin();
  lastDisplayUpdate = millis() - kDisplayUpdateIntervalSeconds * 1000;

  rtc_gpio_pullup_en(kPinPrgButton);

  if (kLoRaWANEnabled)
  {
    // LMIC init using the computed target
    const lmic_pinmap *pPinMap = Arduino_LMIC::GetPinmap_ThisBoard();
    if (pPinMap == nullptr)
    {
      serial.println(F("board not known to library; add pinmap or update getconfig_thisboard.cpp"));
    }

    // LMIC init
    os_init_ex(pPinMap);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Enable or disable ADR (data rate adaptation).
    // Should be turned off if the device is not stationary (mobile).
    // 1 is on, 0 is off.
    LMIC_setAdrMode(kLoRaWANAdrEnabled);

    LMIC_registerEventCb(&onLmicEvent, nullptr);

    u4_t networkId = preferences.getUInt("netid");
    if (networkId != 0)
    {
      u4_t seqnoUp = preferences.getUInt("seqnoUp");
      u4_t seqnoDn = preferences.getUInt("seqnoDn");
      devaddr_t deviceAddress = preferences.getUInt("devaddr");
      u1_t networkSessionKey[16];
      preferences.getBytes("nwkskey", &networkSessionKey, sizeof(uint8_t) * 16);
      u1_t applicationSessionKey[16];
      preferences.getBytes("appskey", &applicationSessionKey, sizeof(uint8_t) * 16);

      serial.println(F("found prev info"));
      printNetworkInfo(networkId, deviceAddress, networkSessionKey, applicationSessionKey);

      serial.print(F("seqnoUp: "));
      serial.println(seqnoUp);

      serial.print(F("seqnoDn: "));
      serial.println(seqnoDn);

      LMIC_setSession(networkId, deviceAddress, networkSessionKey, applicationSessionKey);

      LMIC.seqnoUp = seqnoUp,
      LMIC.seqnoDn = seqnoDn;

      // TTN uses SF9 for its RX2 window.
      LMIC.dn2Dr = DR_SF9;

      LMIC_setDrTxpow(kLoRaWANDrTx, 14);
    }

    LMIC_startJoining();
  }

  switch (esp_sleep_get_wakeup_cause())
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    displayEnabled = true;
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    displayEnabled = false;
    u8g2.setPowerSave(1);
    break;
  }

  // Send a LoRaWAN packet every time the device gets powered/waken up
  if (kLoRaWANEnabled)
  {
    // Schedule initial doWork job for immediate execution.
    os_setCallback(&doWorkJob, doWork);
  }
}

void loop()
{
  if (kLoRaWANEnabled)
  {
    os_runloop_once();
  }

  if (shouldSleep)
  {
    goToSleep();
  }

  if (displayEnabled)
  {
    int now = millis();

    if (now - lastDisplayUpdate >= kDisplayUpdateIntervalSeconds * 1000)
    {
      app.update();
      display.print(&kompostState);

      lastDisplayUpdate = now;
    }

    if (now > kDisplayTimeoutSeconds * 1000)
    {
      shouldSleep = true;
    }
  }
}