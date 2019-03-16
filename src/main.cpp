//********************************************************************************//
/*WimB - Where is my bike - Copyright (c) 2018 Leonardo Bispo.  All right reserved.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */
//********************************************************************************//

#include "header.h"

// PINS redefinition
#define PIN_INT1          (7u)
#define PIN_INT2          (8u)
#define BATT              (23ul)
#define PWR_ON            A3

// ********************************************************************************//
// Change/ADD extra serial ports, SERCOM
// https://www.arduino.cc/en/Tutorial/SamdSercom
// .platformio\packages\framework-arduinosam\variants\trinket_m0
// https://github.com/ldab/wimb/variants

Uart Serial_GNSS(&sercom2, 22, 21, SERCOM_RX_PAD_3, UART_TX_PAD_2);

void SERCOM2_Handler()
{
  Serial_GNSS.IrqHandler();
}

Uart Serial_SARA(&sercom1, 25, 24, SERCOM_RX_PAD_1, UART_TX_PAD_0);

void SERCOM1_Handler()
{
  Serial_SARA.IrqHandler();
}
// ********************************************************************************//

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "YourAPN";
const char user[] = "";
const char pass[] = "";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "YourAuthToken";

// Accelerometer declaration, default address is 0x19.
LIS3DH myIMU(0x19);

// Modem declaration, using SerialX interface.
TinyGsm modem( Serial_SARA );

// u-blox GNSS declaration
GNSS gnss( Serial_GNSS );

// Blynk declarations;
WidgetMap myMap(V0);
BlynkTimer timer;
WidgetRTC BlynkRTC;

bool batteryNotify, moveNotify = true;

int loc_index, active, clear = 0;

float lat, lon, batt, acc;

fixType_t fix = NO_FIX;

uint16_t sampleRate = 1;  //Samples per second (Hz) - 1, 10, 25, 50, 100, 200, 400, 1600, 5000
uint8_t accelRange = 2;   //Accelerometer range = 2, 4, 8, 16g

// Timers identifiers
uint8_t sendFreq = 0;

// Clear Map Pins when button is pressed
void clearMap( void );

// Put MCU to Sleep WFI -> wait for MOVE interrupt
void goodNightSTOP( void );

// Put MCU to Sleep WFI -> wait for STOP interrupt
void goodNightMOVE( void );

// Configure Sleep mode;
void sleepConf( void );

// Send battery information to the App
void sendBattery( void );

// IMU detects movement for certain period of time therefore active peripherals and tracking
void onlineMode( void );

// Get coordinates function as timer requires void
void getCoordinates( void );

// Request Blynk server to re-send latest values for all pins in case hardware reset or 
// lose input status and to allow Clear command when hardware is offline/sleeping
BLYNK_CONNECTED() 
{
  Blynk.syncVirtual(V2, V3, V10);
}

BLYNK_WRITE(V2)
{
  active = param.asInt();
  if( active )
  {
    gnss.init( ON_OFF, 280000, 10);
    
    if( moveNotify )
    {
      String emailBody = String("<p>Asset is moving, check the app for location.</p><p>Battery = ")
                               + String(batt,2) + "V.</p>";
      Blynk.email("🚨 Asset Moving 🏃", emailBody);
      moveNotify = false;      
    }

    getCoordinates();
    sendFreq = timer.setInterval(300L, getCoordinates);
  }
  else
  {
    // Disable motion detection until next STOP -> Interrupt 2
    moveNotify = true;
    goodNightMOVE();
  }
  
}

BLYNK_WRITE(V3)
{
  clear = param.asInt();

  if(clear) clearMap();

  sendBattery();
}

BLYNK_WRITE(V10) 
{
  // Use Blynk Cloud to keep track of location index
  // TODO may need to compare with internal memory, check def behaviour on atsamd21e18
  loc_index = param.asInt();
}

void getCoordinates()
{
  gnss.getCoodinates(lon, lat, fix, acc, 50);
  // Keep track of battery and change frequency if low
  sendBattery();
  if( !batteryNotify )
    timer.changeInterval(sendFreq, 1800L);
}

void sendBattery()
{
  // Measure few times in order to smooth it out
  batt = 0;
  for( int i=0; i<10; i++)
  {
    batt += float(analogRead(BATT))*2.23f / 4095 * 2; //calculate battery considerin 1M voltage divider and 12 bits
  }

  batt /= 10.0f;

  // TODO - Battery levels to be defined;
  // currently based on https://github.com/ldab/WimB/blob/master/datasheet/GMB052030.pdf
  if( batt > 3.8 ){
      Blynk.virtualWrite(V4, 4);
      batteryNotify = true;
  }
  else if( batt > 3.6 ){
      Blynk.virtualWrite(V4, 3);
  }
  else if( batt > 3.5 ){
      Blynk.virtualWrite(V4, 2);
  }
  else{
      Blynk.virtualWrite(V4, 1);
      if( batteryNotify )
      {
        String emailBody = String("Battery is running low @") + String(batt,2) + "V charge it.";
        Blynk.email("🚨 Low Battery 🔌", emailBody);
        batteryNotify = false;
      }
  }
}

void clearMap()
{
  myMap.clear();
  loc_index = 0;
  Blynk.virtualWrite(V3, 0);
}

void goodNightSTOP()
{
  // Remove interrupt from stop to avoid multiple wakes, this PIN will be High when not moving
  detachInterrupt(INT_2);
  attachInterrupt(INT_1, onlineMode, RISING); // Sanity check
  delay(20);

  // put device to sleep
  USBDevice.standby();
  __DSB();
  __WFI();
}

void goodNightMOVE()
{
  // Turn SARA modem off
  digitalWrite(PWR_ON, LOW);
  delay(1600);

  // Remove interrupt from start to avoid multiple wakes, this PIN will be High not moving
  detachInterrupt(INT_1);
  attachInterrupt(INT_2, goodNightSTOP, RISING); // Sanity check
  delay(20);

  // put device to sleep
  USBDevice.standby();
  __DSB();
  __WFI();
}

void setup()
{
  // Control SARA Module
  pinMode(PWR_ON, OUTPUT);
  digitalWrite(PWR_ON, HIGH);
  
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  BlynkRTC.begin();

  // The default analogRead() resolution for these boards is 10 bits, for compatibility.
  // ISSUE: https://forum.arduino.cc/index.php?topic=434775.0
  analogReadResolution(12);
  analogReference(AR_INTERNAL2V23);              //Change reference to internal 2.23V
  pinMode(BATT, INPUT);                          //Battery connected to PA03 -> Pin AREF / 25
  batt = float(analogRead(BATT))*2.23f / 4095 * 2; //calculate battery considerin 1M voltage divider and 12 bits

  if( myIMU.begin(sampleRate, 1, 1, 1, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }

  //Detection threshold can be from 1 to 127 and depends on the Range chosen above, change it and 
  //test accordingly to your application -> Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(INT_1, DET_MOVE, 3, 5);
  myIMU.intConf(INT_2, DET_STOP, 1, 30);

  //Confirm configuration, no need to print as DBG will
  uint8_t readData = 0;
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);

  // Initialize GNSS UART
  pinPeripheral(21, PIO_SERCOM);
  pinPeripheral(22, PIO_SERCOM);
  Serial_GNSS.begin(9600);
  //delay(3000);
  delay(10);

  // Initialize GNSS read every x seconds
  Serial_GNSS.begin( 9600 );

  // When power reset GNSS auto goes on, turn it off;
  gnss.off();

  // Interrupt 1 to detect when it's moving -> PA00 pin 8
  pinMode(PIN_INT1, INPUT);
  attachInterrupt(PIN_INT1, onlineMode, RISING);

  // Interrupt 2 to detect when it's stopped -> PA01 pin 7
  pinMode(PIN_INT2, INPUT);
  attachInterrupt(PIN_INT2, goodNightSTOP, RISING);

  // Configure Sleep registers, wake from external interrupt
  sleepConf();

}

void loop()
{
  Blynk.run();
  timer.run();
}

void onlineMode()
{
  // Initialize GSM SARA module
  Serial_SARA.begin(115200);
  //delay(3000);
  delay(10);

  //Toggle POWER ON Pin for 0.3 seconds to turn module ON
  digitalWrite(PWR_ON, LOW);
  delay(300);
  digitalWrite(PWR_ON, HIGH);
  delay(2000); // alow few seconds for modem to turn ON
  
  // Initialize modem as restart had many invalid parameters for R412
  Serial.println("Initializing modem...");
  modem.init();

  // TinyGSM Lib does not use the right APN set@ https://github.com/vshymanskyy/TinyGSM/issues/244
  char apnConfig[] = "AT+CGDCONT=1,\"IP\",\"";
  strcat (apnConfig, apn);
  strcat (apnConfig, "\"");
  Serial_SARA.write(apnConfig);
  // TODO check reply

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  Blynk.config(modem, auth);
  //Blynk.begin(auth, modem, apn, user, pass);

  detachInterrupt(PIN_INT1);

}

void sleepConf()
{
  //************* Other options to check ****************************************************
  //https://github.com/arduino-libraries/ArduinoLowPower/blob/master/src/samd/ArduinoLowPower.cpp
  //https://github.com/arduino-libraries/RTCZero

  // http://infocenter.arm.com/help/topic/com.arm.doc.dui0662b/DUI0662B_cortex_m0p_r0p1_dgug.pdf
  SYSCTRL->OSC32K.reg |=  (SYSCTRL_OSC32K_RUNSTDBY | SYSCTRL_OSC32K_ONDEMAND); // set internal 32k oscillator to run when idle or sleep mode is chosen

  // 15.8.3 Generic Clock Control - http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf#_OPENTOPIC_TOC_PROCESSING_d115e46482
  REG_GCLK_CLKCTRL  |= GCLK_CLKCTRL_ID(GCM_EIC) |   // generic clock multiplexer id for the external interrupt controller
                       GCLK_CLKCTRL_GEN_GCLK1 |     // generic clock 1 which is osc32k
                       GCLK_CLKCTRL_CLKEN;          // enable it
  
  while (GCLK->STATUS.bit.SYNCBUSY);                // write protected, wait for sync

  // if does not work, check: https://github.com/arduino-libraries/ArduinoLowPower/blob/b38a5bc50c06da903f6614f773d2a676203c7bf2/src/samd/ArduinoLowPower.cpp#L86
  // GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K

  // Make sure that the Flash does not power all the way down https://github.com/arduino-libraries/ArduinoLowPower/blob/b38a5bc50c06da903f6614f773d2a676203c7bf2/src/samd/ArduinoLowPower.cpp#L98
  NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
  
  // Set Standby mode (OR Idle) http://infocenter.arm.com/help/topic/com.arm.doc.dui0662b/DUI0662B_cortex_m0p_r0p1_dgug.pdf#G7.1046307
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  //PM->SLEEP.reg |= PM_SLEEP_IDLE_APB; // Idle2 - sleep CPU, AHB, and APB clocks

  // May not need it as defined in Line 101 .platformio\packages\framework-arduinosam\cores\adafruit\WInterrupts.c
  // EExt_Interrupts in = g_APinDescription[PIN_INT1].ulExtInt;
  // EIC->WAKEUP.reg |= (1 << in);
  // EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN4;          // Set External Interrupt Controller 4

}
