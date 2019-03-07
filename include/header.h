#include "Arduino.h"
#include "Wire.h"

/* Comment this out to disable prints and save space */
// #define BLYNK_PRINT SerialUSB
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
//#define APP_DEBUG
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_MODEM_UBLOX

// Set serial for debug console (to the Serial Monitor, speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module) 
#define SerialAT Serial1

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#define BLYNK_MAX_SENDBYTES 1024
#define BLYNK_NO_BUILTIN

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

#include "lis3dh-motion-detection.h"