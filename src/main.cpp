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

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "YourAPN";
const char user[] = "";
const char pass[] = "";

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
const char auth[] = "YourAuthToken";

//Accelerometer declaration, default address is 0x19.
LIS3DH myIMU(0x19);

//Modem declaration, using SerialX interface.
TinyGsm modem(Serial_SARA);

//Blynk declarations;
WidgetMap myMap(V0);
BlynkTimer timer;
WidgetRTC rtc;

float lat, lon, batt;

int loc_index, active, clear = 0;

uint16_t sampleRate = 1;  //Samples per second (Hz) - 1, 10, 25, 50, 100, 200, 400, 1600, 5000
uint8_t accelRange = 2;   //Accelerometer range = 2, 4, 8, 16g

//Send each coordinate with date and time identifier, example 15/2 - 16:20
void sendCoord( void );

//Clear Map Pins when button is pressed
void clearMap( void );

//Clear Map Pins when button is pressed
void goodNight( void );

//Configure Sleep mode;
void sleepConf( void );

//Send battery information to the App
void sendBattery( void );

// IMU detects movement for certain period of time therefore active peripherals and tracking
void trackingMode( void );

//Request Blynk server to re-send latest values for all pins in case hardware reset or 
//lose input status and to allow Clear command when hardware is offline/sleeping
BLYNK_CONNECTED() 
{
  Blynk.syncVirtual(V2, V3, V10);
}

BLYNK_WRITE(V2)
{
  active = param.asInt();
}

BLYNK_WRITE(V3)
{
  clear = param.asInt();

  if(clear) clearMap();
}

BLYNK_WRITE(V10) 
{
  //Use Blynk Cloud to keep track of location index
  //TODO may need to compare with internal memory, check def behaviour on atsamd21e18
  loc_index = param.asInt();
}

// GPS Libraries https://github.com/loginov-rocks/UbxGps?utm_source=platformio&utm_medium=piohome
// or https://github.com/bolderflight/UBLOX
// or https://playground.arduino.cc/UBlox/GPS

// U-BLOX reference @ https://os.mbed.com/teams/ublox/?utm_source=platformio&utm_medium=piohome

void sendCoord()
{
  String dateTime = String(day()) + "/" + month() + "-" + hour() + ":" + minute();
  myMap.location(loc_index, lat, lon, dateTime);
}

void sendBattery()
{
  //Battery levels to be defined;
  //currently based on https://github.com/ldab/WimB/blob/master/datasheet/GMB052030.pdf
  if( batt > 3.8 ){
      Blynk.virtualWrite(V4, 4);
  }
  else if( batt > 3.6 ){
      Blynk.virtualWrite(V4, 3);
  }
  else if( batt > 3.5 ){
      Blynk.virtualWrite(V4, 2);
  }
  else{
      Blynk.virtualWrite(V4, 1);
  }
}

void clearMap()
{
  myMap.clear();
  loc_index = 0;
  Blynk.virtualWrite(V3, 0);
}

void goodNight()
{
  //put device to sleep
  __WFI();
}

void setup()
{
  //Control SARA Module
  pinMode(PWR_ON, OUTPUT);
  digitalWrite(PWR_ON, HIGH);
  
  // Set console baud rate
  Serial.begin(115200);
  delay(10);

  //The default analogRead() resolution for these boards is 10 bits, for compatibility.
  // ISSUE: https://forum.arduino.cc/index.php?topic=434775.0
  analogReadResolution(12);
  analogReference(AR_INTERNAL2V23);                  //Change reference to internal 2.23V
  pinMode(BATT, INPUT);                          //BAttery connected to PA03 -> Pin AREF / 25
  batt = float(analogRead(25))*2.23f / 4095 * 2; //calculate battery considerin 1M voltage divider and 12 bits

    //IF battery is too low, notify and deep sleep;

  if( myIMU.begin(sampleRate, 1, 1, 1, accelRange) != 0 )
  {
    Serial.print("Failed to initialize IMU.\n");
  }

  //Detection threshold can be from 1 to 127 and depends on the Range chosen above, change it and 
  //test accordingly to your application -> Duration = timeDur x Seconds / sampleRate
  myIMU.intConf(INT_1, DET_MOVE, 50, 5);
  myIMU.intConf(INT_2, DET_STOP, 50, 30);

  //Confirm configuration, no need to print as DBG will
  uint8_t readData = 0;
  myIMU.readRegister(&readData, LIS3DH_INT1_CFG);
  myIMU.readRegister(&readData, LIS3DH_INT2_CFG);

  //Configure Sleep registers, wake from external interrupt
  sleepConf();

  //Interrupt 1 to detect when it's moving -> PA00 pin 8
  //External wake #TODO
  pinMode(PIN_INT1, INPUT);
  attachInterrupt(PIN_INT1, trackingMode, HIGH);

  //Interrupt 2 to detect when it's stopped -> PA01 pin 7
  pinMode(PIN_INT2, INPUT);
  attachInterrupt(PIN_INT2, goodNight, HIGH);

  timer.setInterval(10000L, sendCoord);
}

void loop()
{
  Blynk.run();
  timer.run();
}

void trackingMode()
{
  // Initialize GSM SARA module
  Serial_SARA.begin(115200);
  //delay(3000);
  delay(10);

  // Initialize GNSS module
  // UBX-RXM-PMREQ -> Turn module on and off
  pinPeripheral(21, PIO_SERCOM);
  pinPeripheral(22, PIO_SERCOM);
  Serial_GNSS.begin(9600);
  //delay(3000);
  delay(10);
  
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  //This can take quite some time, do something else
  //Try Blynk.config(auth);
  Blynk.begin(auth, modem, apn, user, pass);

  rtc.begin();

  detachInterrupt(PIN_INT1);
}

void sleepConf()
{
  //As this is a crystalless board, few bits need to be changed.
  SYSCTRL->OSC32K.reg |=  (SYSCTRL_OSC32K_RUNSTDBY | SYSCTRL_OSC32K_ONDEMAND); // set internal 32k oscillator to run when idle or sleep mode is chosen

  REG_GCLK_CLKCTRL  |= GCLK_CLKCTRL_ID(GCM_EIC) |   // generic clock multiplexer id for the external interrupt controller
                       GCLK_CLKCTRL_GEN_GCLK1 |     // generic clock 1 which is osc32k
                       GCLK_CLKCTRL_CLKEN;          // enable it
  
  while (GCLK->STATUS.bit.SYNCBUSY);                // write protected, wait for sync

  EIC->WAKEUP.reg |= EIC_WAKEUP_WAKEUPEN4;          // Set External Interrupt Controller 4
  
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;  // Enable Idle0 mode - sleep CPU clock only
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_AHB; // Idle1 - sleep CPU and AHB clocks
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_APB; // Idle2 - sleep CPU, AHB, and APB clocks

  // It is either Idle mode or Standby mode, not both. 
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   // Enable Standby or "deep sleep" mode

}
