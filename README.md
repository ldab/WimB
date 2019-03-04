# WimB - Where is my Bike

This is a minimalist Blynk App you will 1,800 energy points.

<p>The firmware has been writen using PlatformIO which is a nice, neat IDE, check it out:&nbsp;<a href="https://platformio.org/platformio-ide" rel="nofollow">Learn how to install PlatformIO IDE</a></p>

[![GitHub version](https://img.shields.io/github/release/ldab/wimb.svg)](https://github.com/ldab/wimb/releases/latest)
[![Build Status](https://travis-ci.org/ldab/WimB.svg?branch=master)](https://travis-ci.org/ldab/WimB)
![Libraries.io dependency status for latest release](https://img.shields.io/librariesio/release/ldab/wimb.svg)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://github.com/ldab/wimb/blob/master/LICENSE)

![GitHub last commit](https://img.shields.io/github/last-commit/ldab/wimb.svg?style=social)

## Download Blynk App: [Getting Started with Blynk](https://www.blynk.cc/getting-started/)

* Download Blynk App: [](http://j.mp/blynk_Android) [](http://j.mp/blynk_iOS)
* Touch the QR-code icon and point the camera to the code below
<p><img src="https://image.ibb.co/gxZFDz/Untitled.png" height="95" /></p>

<p><img src="https://gitlab.com/ldab/wimb/raw/master/pics/QR_Blynk.jpg" alt="" width="240" height="412" />
&nbsp;&nbsp;
<img src="https://gitlab.com/ldab/wimb/raw/master/pics/App_Blynk.jpg" alt="" width="240" height="412" /></p>

* Enjoy my app!

## TODO

- [ ] Put device to sleep;
- [ ] Communnicate with SARA-R4
- [x] Set SARA-PSM values
- [ ] Communicate with GNSS;
- [ ] Set interrupts on accelerometer;
- [ ] Finish accelerometer lib

## SARA-R4 PSM - Power Save Mode configuration

The grant of PSM is a negotiation between SARA-R4/N4 series module and the **attached network:** the network accepts PSM  by  providing  the  actual  value  of  the  “Active  Timer”  (and  “Periodic  Update Timer”) to be used in the Attach/TAU/RAU accept procedure. The maximum duration, including the “Periodic Update Timer”, is about 413 days.  The SARA-R4/N4  series module  enters  PSM  low  power deep sleep mode only after the “Active Timer” expires

* If  the  power  saving  mode  is  enabled  (+CPSMS:  1),  everything  on  the  device  will  power  down  except  thereal-time clock (RTC) after the expiry of T3324 (Active Time). It will stay powered down until the expiry ofT3412 (Extended TAU Timer) or if the Power On line is toggled.
* If the power saving mode is disabled (+CPSMS: 0), the device will not enter Power Save Mode (PSM)

#### Requested_Periodic_TAU - PSM total duration

```
Bits 5 to 1 represent the binary coded timer value.
Bits 6 to 8 defines the timer value unit for the GPRS timer:
8 7 6 
0 0 0 value is incremented in multiples of 10 minutes
0 0 1 value is incremented in multiples of 1 hour
0 1 0 value is incremented in multiples of 10 hours
0 1 1 value is incremented in multiples of 2 seconds
1 0 0 value is incremented in multiples of 30 seconds
1 0 1 value is incremented in multiples of 1 minute
1 1 0 value is incremented in multiples of 320 hours
1 1 1 value indicates that the timer is deactivated.
```

**The requested extended periodic TAU value(T3412) iscoded as one byte (octet 3) of the GPRS Timer 3 information element coded as bitformat (e.g. "10100001" equals 1 minute).  see the GPRS Timer 2 IE in 3GPP TS 24.008 Table10.5.163a/3GPP TS 24.008 - page 669**

#### Requested_Active_Time - Active/Paging time

```
Bits 5 to 1 represent the binary coded timer value.
Bits 6 to 8 defines the timer value unit for the GPRS timer as follows:
Bits
8 7 6
0 0 0  value is incremented in multiples of 2 seconds
0 0 1  value is incremented in multiples of 1 minute
0 1 0  value is incremented in multiples of decihours
1 1 1  value indicates that the timer is deactivated.

Other values shall be interpreted as multiples of 1 minute in this version of the protocol. 
```

** Requested Active Time value (T3324) iscoded as one byte (octet 3) of the GPRS Timer 3 information element coded as bitformat (e.g. "00100100" equals 4 minutes).  see the GPRS Timer 2 IE in 3GPP TS 24.008 Table10.5.163/3GPP TS 24.008 - page 668**

## Schematic

![Schematic page 1](./schematic/wimb.png)
![Schematic page 2](./schematic/wimb_2.png)

## Datasheets

You can find technical information here: [datasheets](./datasheet)

## How to start?



## Credits

Github Shields and Badges created with [Shields.io](https://github.com/badges/shields/)

Icons made by [Smashicons](https://www.flaticon.com/authors/smashicons) from [Flaticon](www.flaticon.com) is licensed by Creative Commons BY 3.0 [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/)
