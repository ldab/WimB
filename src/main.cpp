#include "Arduino.h"
#include "Wire.h"

/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT SerialUSB
#define BLYNK_PRINT Serial
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_MODEM_UBLOX

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module) 
#define SerialAT Serial1

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "YourAPN";
const char user[] = "";
const char pass[] = "";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "YourAuthToken";

TinyGsm modem(SerialAT);

WidgetMap myMap(V0);

BlynkTimer timer;
WidgetRTC rtc;

float lat, lon, bat;

int loc;

int active, clear = 0;

BLYNK_CONNECTED() 
{
  // Request Blynk server to re-send latest values for all pins
  Blynk.syncVirtual(V2, V3);
}

BLYNK_WRITE(V2)
{
  active = param.asInt();
}

BLYNK_WRITE(V3)
{
  clear = param.asInt();
}

void sendCoord()
{
  //Send each coordinate with date and time example 15/2 - 16:20
  String dateTime = String(day()) + "/" + month() + "-" + hour() + ":" + minute();
  myMap.location(loc, lat, lon, dateTime);
}

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(115200);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Blynk.begin(auth, modem, apn, user, pass);

  rtc.begin();

  timer.setInterval(10000L, sendCoord);
}

void loop()
{
  Blynk.run();
  timer.run();
}