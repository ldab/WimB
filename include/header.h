#include "Arduino.h"
#include "Wire.h"
#include "wiring_private.h"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
//#define APP_DEBUG
#define TINY_GSM_DEBUG Serial

// Enable Serial debbug on Serial UART to see IMU registers wrote
#define SERIAL_DEBUG Serial

// Define modem type
#define TINY_GSM_MODEM_UBLOX

//LIS3DH - IMU operation mode
#define NORMAL_MODE
//#define NORMAL_MODE
//#define HIGH_RESOLUTION

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
#include "ublox_GNSS.h"
