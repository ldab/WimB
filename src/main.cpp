//********************************************************************************//
/*WimB - Where is my bike - Copyright (c) 2019 Leonardo Bispo.  All right reserved.
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

// PIN re-definition
#define PIN_INT1          (18u)   // PA06
#define PIN_INT2          (17u)   // PA07
#define BATT              (14ul)  // PA02 -> A0
#define PWR_ON            (20ul)  // PA31 -> SWDIO

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

Uart Serial_SARA(&sercom1, 8, 7, SERCOM_RX_PAD_1, UART_TX_PAD_0);

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
LIS3DH myIMU( 0x19 );

// Modem declaration, using SerialX interface.
TinyGsm modem( Serial_SARA );

// u-blox GNSS declaration
GNSS gnss( Serial_GNSS );

// Blynk declarations;
WidgetMap myMap(V0);
BlynkTimer timer;
WidgetRTC BlynkRTC;

bool batteryNotify = true;
bool moveNotify    = true;

int loc_index = 0;
int active    = 0;
int clear     = 0;

float lat, lon, batt, acc;

fixType_t fix = NO_FIX;

uint16_t  sampleRate = 1;   //Samples per second (Hz) - 1, 10, 25, 50, 100, 200, 400, 1600, 5000
uint8_t   accelRange = 2;   //Accelerometer range = 2, 4, 8, 16g

// Timers identifiers
uint8_t sendFreq = 0;

// Clear Map Pins when button is pressed
void clearMap( void );

// Put MCU to Sleep WFI -> wait for MOVE interrupt
void goodNightSTOP( void );

// Put MCU to Sleep WFI -> wait for STOP interrupt
void goodNightMOVE( void );

// WFI and DSB actually sleep command
void goToSleep( void );

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
    Serial_GNSS.begin( 9600 );
    
    // Initialize GNSS, interval mode, 280sec Off and 10sec active.
    gnss.init( ON_OFF, 280000, 10);
    
    if( moveNotify )
    {
      String emailBody = String("<p>Asset is moving, check the app for location.</p><p>Battery = ")
                               + String(batt,2) + "V.</p>";
      Blynk.email("🚨 Asset Moving 🏃", emailBody);
      moveNotify = false;      
    }

    getCoordinates();
    sendFreq = timer.setInterval(60000L, getCoordinates);
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
  if( gnss.getCoodinates(lon, lat, fix, acc, 50) )
  {
    String dateTime = String(day()) + "/" + month() + "-" + hour() + ":" + minute();
    myMap.location(loc_index, lat, lon, dateTime);
    loc_index++;
  }
  // Keep track of battery and change frequency if low
  sendBattery();
  if( !batteryNotify )
    timer.changeInterval(sendFreq, 300000L);
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
  attachInterrupt(INT_1, onlineMode, HIGH); // Sanity check
  delay(20);

  goToSleep();
}

void goodNightMOVE()
{
  // Turn SARA modem off
  digitalWrite(PWR_ON, HIGH);
  delay(1600);
  digitalWrite(PWR_ON, LOW);

  // Remove interrupt from start to avoid multiple wakes, HIGH = not moving
  detachInterrupt(INT_1);
  attachInterrupt(INT_2, goodNightSTOP, HIGH); // Sanity check
  delay(20);

  goToSleep();
}

void setup()
{
    /************************************************************************
  * \.platformio\packages\framework-arduinosam\cores\adafruit\wiring.c
  * Line 107 -> INPUT_PULLUP reduces the current significantly
  * SWCLK when Pulled UP reduces the sleep current               
  * ***********************************************************************/
  pinMode( 19, INPUT_PULLUP );

  // Control SARA Module
  pinMode(PWR_ON, OUTPUT);
  digitalWrite(PWR_ON, LOW);
  
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  BlynkRTC.begin();

  // The default analogRead() resolution for these boards is 10 bits, for compatibility.
  analogReadResolution(12);
  analogReference(AR_INTERNAL2V23);                // Change reference to internal 2.23V
  pinMode(BATT, INPUT);                            // Battery connected to PA02
  batt = float(analogRead( BATT )) * 2.23f / 4095 * 2; // Calculate battery considerin 1M voltage divider and 12 bits

  if( myIMU.begin(sampleRate, 1, 1, 1, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }

  //Detection threshold can be from 1 to 127 and depends on the Range chosen above, change it and 
  //test accordingly to your application -> Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(INT_1, DET_MOVE, 3, 5);
  myIMU.intConf(INT_2, DET_STOP, 1, 30);

  //Confirm configuration,
  // !!! POWER reset is required in order to ssave IMU Config
  uint8_t readData = 0;
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);

  // map PINs to SERCOM ALT for SARA UART
  pinPeripheral(7, PIO_SERCOM_ALT);
  pinPeripheral(8, PIO_SERCOM_ALT);

  // Initialize GNSS UART
  pinPeripheral(21, PIO_SERCOM);
  pinPeripheral(22, PIO_SERCOM);

  Serial_GNSS.begin(9600);
  delay(10);

  // Initialize GNSS on a silly number just to make it sleep in case it wakes as we're not monitoring it
  gnss.init( ON_OFF, 36000, 5);

  // When power reset GNSS auto goes on, turn it off;
  gnss.off();

  // Interrupt 1 to detect when it's moving -> PA00 pin 8
  pinMode(PIN_INT1, INPUT);
  attachInterrupt(PIN_INT1, onlineMode, HIGH);

  // Interrupt 2 to detect when it's stopped -> PA01 pin 7
  pinMode(PIN_INT2, INPUT);
  attachInterrupt(PIN_INT2, goodNightSTOP, HIGH);

}

void loop()
{
  Blynk.run();
  timer.run();
}

void onlineMode()
{
  // Initialize GSM SARA module
  Serial_SARA.begin( 115200 );
  //delay(3000);
  delay(10);

  //Toggle POWER ON Pin for 0.3 seconds to turn module ON
  digitalWrite(PWR_ON, HIGH);
  delay(300);
  digitalWrite(PWR_ON, LOW);
  delay(2000); // alow few seconds for modem to turn ON
  
  // Initialize modem as restart had many invalid parameters for R412
  Serial.println("Initializing modem...");
  modem.init();

  // TinyGSM Lib does not use the right APN set@ https://github.com/vshymanskyy/TinyGSM/issues/244
  //char apnConfig[] = "AT+CGDCONT=1,\"IP\",\"";
  //strcat (apnConfig, apn);
  //strcat (apnConfig, "\"");
  //Serial_SARA.write(apnConfig);
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

void goToSleep()
{
    // 15.8.3 Generic Clock Control - http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf#_OPENTOPIC_TOC_PROCESSING_d115e46482
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID( GCM_EIC ) |   // EIC id for the external interrupt controller
                       GCLK_CLKCTRL_GEN_GCLK2     |   // generic clock 2 which is OSCULP32K
                       GCLK_CLKCTRL_CLKEN;            // enable it

  while (GCLK->STATUS.bit.SYNCBUSY);                  // Write protected, wait for sync

  // Make sure that the Flash does not power all the way down https://github.com/arduino-libraries/ArduinoLowPower/blob/b38a5bc50c06da903f6614f773d2a676203c7bf2/src/samd/ArduinoLowPower.cpp#L98
  NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
  
  // Set Standby mode (OR Idle) http://infocenter.arm.com/help/topic/com.arm.doc.dui0662b/DUI0662B_cortex_m0p_r0p1_dgug.pdf#G7.1046307
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  Serial.end();
  Serial_GNSS.end();
  Serial_SARA.end();
  
  USBDevice.detach();

  __DSB();
  __WFI();

  USBDevice.attach();

  //Serial.begin( 115200 );
  //Serial_SARA.begin( 115200 );
  //Serial_GNSS.begin( 9600 );

}
