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

#include "Arduino.h"
#include "wiring_private.h"
#include "Ticker.h"
#include "ublox_GNSS.h"
//#include <Adafruit_SleepyDog.h>

// The pins will change on the next Hardware version
#define PIN_INT1          7
#define PIN_INT2          8
#define BATT              23
#define LED               12

void goodNight( void );
void goodMorning( void );
void offin5( void );
void battery( void );

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

void goodNight()
{
  Serial.println("\nGoing to Sleep");

  /*// Disable ON DEMAND and RUN Standby for DFLL48M and OSC8M
  SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 0;
  SYSCTRL->OSC8M.bit.ONDEMAND = 0;
  SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
  SYSCTRL->OSC32K.bit.ONDEMAND = 0;
  SYSCTRL->OSC32K.bit.RUNSTDBY = 0;
  SYSCTRL->XOSC32K.bit.ONDEMAND = 0;
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 0;*/

  // 15.8.3 Generic Clock Control - http://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-Family-DataSheet-DS40001882D.pdf#_OPENTOPIC_TOC_PROCESSING_d115e46482
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID( GCM_EIC ) |   // EIC id for the external interrupt controller
                       GCLK_CLKCTRL_GEN_GCLK2     |   // generic clock 2 which is OSCULP32K
                       GCLK_CLKCTRL_CLKEN;            // enable it

  while (GCLK->STATUS.bit.SYNCBUSY);                  // Write protected, wait for sync

  // Disable 3.3V Brown-Out Detector
  // SYSCTRL->BOD33.reg = 0;

  // If voltage regulator is disabled, EXT INT and Reset does not work.
  // SYSCTRL->VREG.reg = 0;

  // Based on https://www.microchip.com/wwwAppNotes/AppNotes.aspx?appnote=en591392

  // Switches all GCLK outputs to a generator that is configured with a source oscillator. BUT DFLL48M Reference
  /*for (int gclk_id = 1; gclk_id < 0x25; gclk_id++) {
		GCLK->CLKCTRL.reg = (gclk_id << GCLK_CLKCTRL_ID_Pos) | GCLK_CLKCTRL_GEN_GCLK7;
	}*/

  /*// Generic clock generator 2, divisor = 32 (2^(DIV+1))
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);

  // Enable clock generator 2 using low-power 32KHz oscillator.
  // With /32 divisor above, this yields 1024Hz(ish) clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 2 )        | // Generic clock generator 2
                      GCLK_GENCTRL_GENEN          | // Enable it
                      GCLK_GENCTRL_SRC_OSCULP32K  | // Source of the clock is Low power internal OSCULP32K
                      GCLK_GENCTRL_DIVSEL;

  while (GCLK->STATUS.bit.SYNCBUSY);                  // Write protected, wait for sync*/

  // Clear Power Manager (PM) mask
  /*
  PM->APBAMASK.reg &= ~(PM_APBAMASK_WDT  |
                        PM_APBAMASK_PAC0 |
                        //PM_APBAMASK_EIC  |
                        PM_APBAMASK_GCLK);

  PM->APBBMASK.reg &= ~(PM_APBBMASK_PAC1 |
                        PM_APBBMASK_PORT |
                        PM_APBBMASK_DSU  |
                        PM_APBBMASK_DMAC |
                        //PM_APBBMASK_USB  |
                        PM_APBBMASK_NVMCTRL);

  PM->APBCMASK.reg &= ~(PM_APBCMASK_ADC     |
                        PM_APBCMASK_PAC2    |
                        PM_APBCMASK_DAC     |
                        PM_APBCMASK_AC      |
                        PM_APBCMASK_TC7     |
                        PM_APBCMASK_TC6     |
                        PM_APBCMASK_TC5     |
                        PM_APBCMASK_TC4     |
                        PM_APBCMASK_TC3     |
                        PM_APBCMASK_TCC2    |
                        PM_APBCMASK_TCC1    |
                        //PM_APBCMASK_TCC0    |
                        PM_APBCMASK_SERCOM5 |
                        PM_APBCMASK_SERCOM4 |
                        PM_APBCMASK_SERCOM3 |
                        //PM_APBCMASK_SERCOM2 |
                        //PM_APBCMASK_SERCOM1 |
                        //PM_APBCMASK_SERCOM0 |
                        PM_APBCMASK_EVSYS   |
                        PM_APBCMASK_I2S);

  // Clear bits in the clock mask for the AHB bus.
  PM->AHBMASK.reg &= ~(PM_AHBMASK_USB |
                       PM_AHBMASK_DSU |
                       PM_AHBMASK_HPB1 |
                       PM_AHBMASK_HPB2 |
                       PM_AHBMASK_DMAC
                       // These clocks should remain enabled on this bus
                       //PM_AHBMASK_HPB1 \
                       //PM_AHBMASK_HPB2 \
                       //PM_AHBMASK_HPB0 \
                       //PM_AHBMASK_NVMCTRL \ 
                      );

  // Write protected, wait for sync
  while (GCLK->STATUS.bit.SYNCBUSY);*/

  // Errata: Make sure that the Flash does not power all the way down when in sleep mode.
  NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;

  // Sets the sleep mode of the device;
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  digitalWrite(LED, LOW);

  Serial.end();
  Serial_GNSS.end();
  USBDevice.detach();

  delay(2000);

  // put device to sleep
  __DSB();
  __WFI();

  USBDevice.attach();

  Serial_GNSS.begin( 9600 );
  Serial.begin( 9600 );

  // Send something to wake GNSS
  Serial_GNSS.write(0xFF);
  Serial_GNSS.flush();

}

Ticker turnoff( offin5, 10000 );
Ticker checkBat( battery, 2000 );

GNSS gnss( Serial_GNSS );

void goodMorning()
{
  // Placeholder for EXT wake
  turnoff.start();
}

void setup() {

  /************************************************************************
  * \.platformio\packages\framework-arduinosam\cores\adafruit\wiring.c
  * Line 107 -> INPUT_PULLUP reduces the current significantly
  * SWCLK & SWDIO when Pulled UP reduces the sleep current               
  * ***********************************************************************/
  pinMode( 19, INPUT_PULLUP );
  pinMode( 20, INPUT_PULLUP );

  pinMode( LED, OUTPUT );
  digitalWrite( LED, LOW );

  // Power ON pin, included here for test only
  pinMode( A3, OUTPUT );
  digitalWrite( A3, LOW );
  
  // Compare battery measurement
  // pinMode( A0, INPUT);
  pinMode( BATT, INPUT);

  // PA00 and PA01 have wrong Interrupt numbers on variant.cpp
  // g_APinDescription[PIN_INT1].ulExtInt = EXTERNAL_INT_0;
  // g_APinDescription[PIN_INT2].ulExtInt = EXTERNAL_INT_1;
  pinMode( PIN_INT1, INPUT_PULLDOWN );
  pinMode( PIN_INT2, INPUT_PULLDOWN );

  // attachInterrupt(PIN_INT1, goodMorning, RISING);
  // attachInterrupt(PIN_INT2, goodMorning, RISING);

  // Interrupt 1 to detect when it's moving -> PA00 pin 8
  pinMode(A0, INPUT_PULLDOWN);
  //attachInterrupt(A0, goodMorning, HIGH);

  // Serial GNSS PIN attribute
  pinPeripheral(21, PIO_SERCOM);
  pinPeripheral(22, PIO_SERCOM);

  Serial.begin( 9600 );

  Serial_GNSS.begin( 9600 );
  
  // Send something to make sure GNSS is awake
  Serial_GNSS.write(0xFF);
  Serial_GNSS.flush();

  // gnss.init( ON_OFF, 280000, 6);
  // gnss.init( PSM_1HZ );

  turnoff.start();
  // checkBat.start();

}

void offin5()
{

  turnoff.stop();
  checkBat.stop();

  digitalWrite( LED, LOW );
  
  gnss.off();

  goodNight();

}

void battery()
{
  bool pinSt = digitalRead( LED );
  digitalWrite( LED, !pinSt );

  float batt_PA02 = 0;
  float batt_PA03 = 0;

  for( int i = 0; i <10; i++ )
  {
    batt_PA02 += analogRead( A0 ) * 3.3f / 1023;
    batt_PA03 += analogRead( BATT ) * 3.3f / 1023;
  }

  batt_PA02 /= 10.0f;
  batt_PA03 /= 10.0f;

  Serial.println("Batt PA02 = " + String( batt_PA02, 2 ));
  Serial.println("Batt PA03 = " + String( batt_PA03, 2 ) + "\n");
}

void loop()
{

  turnoff.update();
  checkBat.update();

  digitalWrite( LED, HIGH );

  // If anything comes in Serial (USB),
  if (Serial.available()) 
  {
    Serial_GNSS.write(Serial.read());   // read it and send it out Serial_GNSS
  }

  // If anything comes in Serial_GNSS
  if (Serial_GNSS.available()) 
  {
    Serial.write(Serial_GNSS.read());   // read it and send it out Serial (USB)
  }

}