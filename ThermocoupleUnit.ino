/***************************************************************************
  File Name: Standalone unit - pro mini
  Processor/Platform: pro mini 5v
  Development Environment: Arduino 1.6.13

  Designed for use with with Playing With Fusion MAX31856 thermocouple
  breakout boards: SEN-30007 (any TC type) or SEN-30008 (any TC type)

  Copyright © 2015 Playing With Fusion, Inc.
  SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.

  Permission is hereby granted, free of charge, to any person obtaining a
  copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
* **************************************************************************
  REVISION HISTORY:
  Author		Date	    Comments
  J. Steinlage		2015Dec30   Baseline Rev, first production support
  J. Steilnage    2016Aug21   Change functions to support 1shot mode
  P.A. Darling .  2017Jan20 . Example ported for use with LENR logger/PID
  
  Playing With Fusion, Inc. invests time and resources developing open-source
  code. Please support Playing With Fusion and continued open-source
  development by buying products from Playing With Fusion!

 ________  ________  ________     
|\   __  \|\   __  \|\   ___ \    
\ \  \|\  \ \  \|\  \ \  \_|\ \   
 \ \   ____\ \   __  \ \  \ \\ \  
  \ \  \___|\ \  \ \  \ \  \_\\ \ 
   \ \__\    \ \__\ \__\ \_______\
    \|__|     \|__|\|__|\|_______|


 _                           _           _                             _       
| |                         | |         | |                           | |      
| | _____   _____  __      _| |__   __ _| |_   _   _  ___  _   _    __| | ___  
| |/ _ \ \ / / _ \ \ \ /\ / / '_ \ / _` | __| | | | |/ _ \| | | |  / _` |/ _ \ 
| | (_) \ V /  __/  \ V  V /| | | | (_| | |_  | |_| | (_) | |_| | | (_| | (_) |
|_|\___/ \_/ \___|   \_/\_/ |_| |_|\__,_|\__|  \__, |\___/ \__,_|  \__,_|\___/ 
                                                __/ |                          
                                               |___/                           

  
* **************************************************************************
  ADDITIONAL NOTES:
  This file contains functions to initialize and run an Arduino Uno R3 in
  order to communicate with a MAX31856 single channel thermocouple breakout
  board. Funcionality is as described below:
	- Configure Arduino to broadcast results via UART
        - call PWF library to configure and read MAX31856 IC (SEN-30005, any type)
	- Broadcast results to COM port
   Circuit:
     Arduino Uno   Arduino Mega  -->  SEN-30006
     DIO pin 10      DIO pin 10  -->  CS0
     DIO pin  9      DIO pin  9  -->  CS1
     DIO pin  8      DIO pin  8  -->  CS2
     DIO pin  7      DIO pin  7  -->  CS3
     DIO pin  6      DIO pin  6  -->  DR0 (Data Ready... not used in example, but routed)
     DIO pin  5      DIO pin  5  -->  DR1 (Data Ready... not used in example, but routed)
     DIO pin  4      DIO pin  4  -->  DR2 (Data Ready... not used in example, but routed)
     DIO pin  3      DIO pin  3  -->  DR3 (Data Ready... not used in example, but routed)
     MOSI: pin 11  MOSI: pin 51  -->  SDI (must not be changed for hardware SPI)
     MISO: pin 12  MISO: pin 50  -->  SDO (must not be changed for hardware SPI)
     SCK:  pin 13  SCK:  pin 52  -->  SCLK (must not be changed for hardware SPI)
     D03           ''            -->  FAULT (not used in example, pin broken out for dev)
     D02           ''            -->  DRDY (not used in example, only used in single-shot mode)
     GND           GND           -->  GND
     5V            5V            -->  Vin (supply with same voltage as Arduino I/O, 5V)
      NOT CONNECTED              --> 3.3V (this is 3.3V output from on-board LDO. DO NOT POWER THIS PIN!
  It is worth noting that the 0-ohm resistors on the PCB can be removed to
  free-up DIO pins for use with other shields if the 'Data Ready' funcionality
  isn't being used.


  for use with:
                  
                  (           ) (     
                  )\ )     ( /( )\ )  
                  (()/( (   )\()|()/(  
                  /(_)))\ ((_)\ /(_)) 
                  (_)) ((_) _((_|_))   
                  | |  | __| \| | _ \  
                  | |__| _|| .` |   /  
                  |____|___|_|\_|_|_\  
      
                      Logger/PID          

Added RoomTemperature via LM35 on port A0 and external leds

***************************************************************************/
#include "PlayingWithFusion_MAX31856.h"
#include "PlayingWithFusion_MAX31856_STRUCT.h"
#include "SPI.h"
#include <OnOff.h>

uint8_t TC0_CS  =  7;
uint8_t TC1_CS  =  8;
uint8_t TC2_CS  =  9;
uint8_t TC3_CS  = 10;
uint8_t TC0_FAULT = 2;                     // not used in this example, but needed for config setup
uint8_t TC0_DRDY  = 2;                     // not used in this example, but needed for config setup

PWF_MAX31856  thermocouple0(TC0_CS, TC0_FAULT, TC0_DRDY);
PWF_MAX31856  thermocouple1(TC1_CS, TC0_FAULT, TC0_DRDY);
PWF_MAX31856  thermocouple2(TC2_CS, TC0_FAULT, TC0_DRDY);
PWF_MAX31856  thermocouple3(TC3_CS, TC0_FAULT, TC0_DRDY);


struct var_max31856 TC_CH0, TC_CH1, TC_CH2, TC_CH3;
double tcTemps[4] = {0.00, 0.00, 0.00, 0.00};
uint8_t tcErrors[4] = {0, 0, 0, 0};
#define MAX_STRING_DATA_LENGTH 54
short serialBufferPos = 0; // position in read buffer
char serialBuffer[MAX_STRING_DATA_LENGTH];
char serialBufferInByte = 0;
int thermocouplesCount = 4;
unsigned long lastReadTcMillis = 0;
String delimiter(","); 
#define NaN -9999.99

//control leds, using 3 pin red and green 3mm led with single 220 ohm resister
OnOff redLed(2);
OnOff greedLed(3);
unsigned long greenLedOnMillis = 0;


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1
  };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//LM35 for room temp
void printRoomTemperature()
{ 
    int val;
    int dat;
    val=analogRead(0);//LM35 on Analog 0
    dat=(500 * val) /1024;
    Serial.println(":A^"+String(dat));
}



void resetTcErrors()
{
  for (int i = 0; i < 4; i++) {
    tcErrors[i] = 0;
  }
}

void setThermocoupleCount()
{
  int tmp = getValue(serialBuffer, '^', 1).toInt();
  
  if (tmp <= 4 && tmp >= 1) {
    thermocouplesCount = tmp;
    resetTcErrors();
    Serial.println(":@C^"+String(tmp));
  } else {
    Serial.println(":!C"+String(tmp));//Error
  }
}

void actionCommand(char command)
{
  switch (command) {
    case '?': Serial.println(":?"); // echo we are alive, kind of like a loose hand shake
      break;

    case 'C': //set thermocouplesCount (1-4 range), can only disable 2-4 TC's starting at 4 working down to 2
      if (serialBuffer[2] == '^' && strlen(serialBuffer) > 3) {
        setThermocoupleCount();
      } else {
        Serial.println(":!C");//Error
      }
      break;
    case 'c': //display current thermocouplesCount in memeory (for debugging)
      Serial.println(":c^" + String(thermocouplesCount));
      break;

    case '!':
      //host did not understand, handling this is still todo!!!
      //NOTE: sending back a ! may create endless loop but depends on how the host handles it!

      //@todo on red error light for X secs??
      break;

    default:
      //Serial.println(":!"+String(command)); //Error, host must deal with it for given command
      break;
  }
}


void processSerial()
{
  while (Serial.available() > 0)
  {
    serialBufferInByte = Serial.read();
    // add to our read buffer
    serialBuffer[serialBufferPos] = serialBufferInByte;
    serialBufferPos++;

    if (serialBufferInByte == '\n' || serialBufferPos > MAX_STRING_DATA_LENGTH-1) //end of field/cell
    {
      serialBuffer[serialBufferPos - 1] = 0; // delimit end of string with Zero!
      if (serialBuffer[0] == ':') actionCommand(serialBuffer[1]);
      //else Serial.println(":!^ubad");
      serialBuffer[0] = '\0';
      serialBuffer[1] = '\0';
      serialBuffer[2] = '\0';
      serialBufferPos = 0;
    }
  }
}


void readThermocouples()
{
  
  struct var_max31856 *tc_ptr;

  // Read CH 0
  tc_ptr = &TC_CH0;                             // set pointer
  thermocouple0.MAX31856_update(tc_ptr);        // Update MAX31856 channel 0
  // Read CH 1
  tc_ptr = &TC_CH1;                             // set pointer
  thermocouple1.MAX31856_update(tc_ptr);        // Update MAX31856 channel 1
  // Read CH 2
  tc_ptr = &TC_CH2;                             // set pointer
  thermocouple2.MAX31856_update(tc_ptr);        // Update MAX31856 channel 2
  // Read CH 3
  tc_ptr = &TC_CH3;                             // set pointer
  thermocouple3.MAX31856_update(tc_ptr);        // Update MAX31856 channel 3


  // ##### Print information to serial port ####

  // Thermocouple channel 0
  if (TC_CH0.status)
  {
    tcTemps[0] = NaN;
    tcErrors[0] = TC_CH0.status;
    // lots of faults possible at once, technically... handle all 8 of them
    // Faults detected can be masked, please refer to library file to enable faults you want represented
    /*Serial.println("fault(s) detected");
      Serial.print("Fault List: ");
      if(0x01 & TC_CH0.status){Serial.print("OPEN  ");}
      if(0x02 & TC_CH0.status){Serial.print("Overvolt/Undervolt  ");}
      if(0x04 & TC_CH0.status){Serial.print("TC Low  ");}
      if(0x08 & TC_CH0.status){Serial.print("TC High  ");}
      if(0x10 & TC_CH0.status){Serial.print("CJ Low  ");}
      if(0x20 & TC_CH0.status){Serial.print("CJ High  ");}
      if(0x40 & TC_CH0.status){Serial.print("TC Range  ");}
      if(0x80 & TC_CH0.status){Serial.print("CJ Range  ");}
      Serial.println(" ");*/
  }
  else  // no fault, print temperature data
  {
    // MAX31856 External (thermocouple) Temp
    tcTemps[0] = (double)TC_CH0.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
    tcErrors[0] = 0;
  }

  if (thermocouplesCount > 1) {

    // Thermocouple channel 1
    if (TC_CH1.status)
    {
      tcTemps[1] = NaN;
      tcErrors[1] = TC_CH1.status;
    }
    else  // no fault, print temperature data
    {
      // MAX31856 External (thermocouple) Temp
      tcTemps[1] = (double)TC_CH1.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
      tcErrors[1] = 0;
    }
  }

  if (thermocouplesCount > 2) {
    // Thermocouple channel 2
    if (TC_CH2.status)
    {
      tcTemps[2] = NaN;
      tcErrors[2] = TC_CH2.status;
    }
    else  // no fault, print temperature data
    {
      // MAX31856 External (thermocouple) Temp
      tcTemps[2] = (double)TC_CH2.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
      tcErrors[2] = 0;
    }
  }

  if (thermocouplesCount > 3) {

    // Thermocouple channel 3
    if (TC_CH3.status)
    {
      tcTemps[3] = NaN;
      tcErrors[3] = TC_CH3.status;
    }
    else  // no fault, print temperature data
    {
      // MAX31856 External (thermocouple) Temp
      tcTemps[3] = (double)TC_CH3.lin_tc_temp * 0.0078125;           // convert fixed pt # to double
      tcErrors[3] = 0;
    }
  }
}

void printTemps()
{
  String tcS1(tcTemps[0]);
  String res(":R^" + tcS1); //TC 1 always enabled
  
  if (thermocouplesCount > 1) {
    String tcS2(tcTemps[1]);
    res.concat(delimiter + tcS2);
  }
  if (thermocouplesCount > 2) {
    String tcS3(tcTemps[2]);
    res.concat(delimiter + tcS3);
  }
  if (thermocouplesCount > 3) {
    String tcS4(tcTemps[3]);
    res.concat(delimiter + tcS4);
  }

  Serial.println(res);
}

void printErrors()
{
  if (tcErrors[0] || tcErrors[1] || tcErrors[2] || tcErrors[3]) {
    
    String tcS1(tcErrors[0]);
    String res(":E^" + tcS1);//TC 1 always enabled
    
    if (thermocouplesCount > 1) {
      String tcS2(tcErrors[1]);
      res.concat(delimiter + tcS2);
    }
    if (thermocouplesCount > 2) {
      String tcS3(tcErrors[2]);
      res.concat(delimiter + tcS3);
    }
    if (thermocouplesCount > 3) {
      String tcS4(tcErrors[3]);
      res.concat(delimiter + tcS4);
    }

    Serial.println(res);
    redLed.on();
  } else {
    redLed.off();
  }
}

void setup()
{
  redLed.on();
  greedLed.on(); 
  
  delay(1000);                            // give chip a chance to stabilize
  Serial.begin(115200);                   // set baudrate of serial port
  
  // Serial.println("Playing With Fusion: MAX31856, SEN-30007/8");
  // Serial.println("Continous Mode Example");

  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // SPI speed to SPI_CLOCK_DIV16 (1MHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device

  // call config command... options can be seen in the PlayingWithFusion_MAX31856.h file
  thermocouple0.MAX31856_config(K_TYPE, CUTOFF_50HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  thermocouple1.MAX31856_config(K_TYPE, CUTOFF_50HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  thermocouple2.MAX31856_config(K_TYPE, CUTOFF_50HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  thermocouple3.MAX31856_config(K_TYPE, CUTOFF_50HZ, AVG_SEL_1SAMP, CMODE_AUTO);
  
  Serial.println(":?");
  delay(60);
  redLed.off();
  readThermocouples();
  greedLed.off();
  if (tcErrors[0] || tcErrors[1] || tcErrors[2] || tcErrors[3]) {
    redLed.on();
    delay(1000);
  }
}


void loop()
{
  processSerial();
  unsigned long now = millis(); 
  if (now - lastReadTcMillis > 1000) {
    greedLed.on();    
    readThermocouples();
    printTemps();
    printErrors();    
    printRoomTemperature();
    greenLedOnMillis = now;
    lastReadTcMillis = now;
  }
  else if (greenLedOnMillis>0 && now-greenLedOnMillis>150) {
    greedLed.off();
    greenLedOnMillis = 0;
  }
}
