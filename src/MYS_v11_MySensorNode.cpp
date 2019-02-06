/*
* MYS_v11_MySensorNode.ino - Firmware for Mys v1.1 board temperature and humidity sensor Node with nRF24L01+ module
*
* Copyright 2014 Tomas Hozza <thozza@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*
* Authors:
* Tomas Hozza <thozza@gmail.com>
* Richard Clarke <richard.ns@clarke.biz>
*
* MySensors library - http://www.mysensors.org/
* Mys v1.1 -http://www.sa2avr.se/mys-1-1/
* nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
*
Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4 (use inline 10k resistor if your board is 5V)
 -SCL = A5 (use inline 10k resistor if your board is 5V)

Mys_v1.1 board compatible with Arduino PRO Mini 3.3V@8MHz

System Clock  = 8MHz

Information on porting a 1.5.x compatible sketch to 2.0.x

https://forum.mysensors.org/topic/4276/converting-a-sketch-from-1-5-x-to-2-0-x/2

 */

 // Enable debug prints to serial monitor
 //#define MY_DEBUG
 #define DEBUG_RCC 0

 // Enable and select radio type attached
 #define MY_RADIO_RF24
 //#define MY_RADIO_RFM69

 #define MY_NODE_ID 21
 /*Makes this static so won't try and find another parent if communication with
 gateway fails*/
 #define MY_PARENT_NODE_ID 0
 #define MY_PARENT_NODE_IS_STATIC

 /*These are actually the default pins expected by the MySensors framework.
  * This means we can use the default constructor without arguments when
  * creating an instance of the Mysensors class. Other defaults will include
  * transmitting on channel 76 with a data rate of 250kbps.
  */
 #define MY_RF24_CE_PIN 9
 #define MY_RF24_CS_PIN 10
 #define MY_RF24_CHANNEL 100

#include <MySensors.h> 
#include <stdint.h>
#include <math.h> 
#include "SparkFunHTU21D.h"
#include <Wire.h>
#include <UVIS25.h>
#include <BatterySense.hpp>

#define UV_SENSOR 1
#define TEMP_HUM_SENSOR 0

// Sleep time between sensor updates (in milliseconds)
static const uint64_t DAY_UPDATE_INTERVAL_MS = 30000;
static const uint64_t NIGHT_UPDATE_INTERVAL_MS = 900000;//15 mins


enum child_id_t
{
  CHILD_ID_HUMIDITY,
  CHILD_ID_TEMP,
  CHILD_ID_UV,
  CHILD_ID_VOLTAGE
};



uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

//Create an instance of the sensor objects
#if UV_SENSOR
UVIS25 UV; //Ultraviolet sensor
MyMessage msgUVindex(CHILD_ID_UV, V_UV);
uint8_t readUVSensor(bool force);
#endif

#if TEMP_HUM_SENSOR
HTU21D myHTU21D;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
void readHTU21DTemperature(bool force);
void readHTU21DHumidity(bool force);
#endif


MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);
void switchClock(unsigned char clk);

/*Set true to have clock throttle back, or false to not throttle*/
bool highfreq = false;

BatteryLevel batt;
/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
/*If you need to do initialization before the MySensors library starts up,
define a before() function */
void before()
{

}

/*To handle received messages, define the following function in your sketch*/
void receive(const MyMessage &message)
{
  /*Handle incoming message*/
}

/* If your node requests time using requestTime(). The following function is
used to pick up the response*/
void receiveTime(unsigned long ts)
{
}

/*You can still use setup() which is executed AFTER mysensors has been
initialised.*/
void setup()
{
  
  Wire.begin();
  UV.init();
  Serial.begin(9600);
  
}

void presentation()
{
   // Send the sketch version information to the gateway and Controller
  sendSketchInfo("mys_v11-uv", "0.5");
  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_VOLTAGE, S_MULTIMETER);

#if UV_SENSOR
  present(CHILD_ID_UV, S_UV);
#endif

#if TEMP_HUM_SENSOR
  present(CHILD_ID_HUMIDITY, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
#endif
   
}


void loop()
{

  uint8_t uvi;
  uint64_t update_interval_ms = DAY_UPDATE_INTERVAL_MS;
  static uint16_t night_count = 0;
  loopCount++;
  clockSwitchCount++;
  bool forceTransmit = true;
  
  
  
  // When we wake up the 5th time after power on, switch to 1Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependent on oscillator settings).
  if ( (clockSwitchCount == 5) && highfreq)
  {
    /* Switch to 1Mhz by setting clock prescaler to divide by 16 for the reminder of the sketch, 
     * to save power but more importantly to allow operation down to 1.8V
     * 
      */
    switchClock(1<<CLKPS2); 
  }

  uint16_t battLevel = batt.getVoltage();
  send(msgVolt.set(battLevel,1));

#if TEMP_HUM_SENSOR
  readHTU21DTemperature(forceTransmit);
  readHTU21DHumidity(forceTransmit);
#endif

#if UV_SENSOR
  uvi = readUVSensor(forceTransmit);

  /*If UVI is 0 then we're assuming it's late in day so readings aren't interesting. This has to happen several times
  in a row before we switch our sleep interval to the night mode*/
  if (uvi == 0)
  {
    ++night_count; 
     
  }
  else
  {
    /*Some daylight is returning*/
    night_count = 0;
  }

  if(night_count > 10)
  {
    update_interval_ms = NIGHT_UPDATE_INTERVAL_MS;
  }
  else
  {
    update_interval_ms = DAY_UPDATE_INTERVAL_MS;
  }

  
#endif

  sleep(update_interval_ms);

}

#if UV_SENSOR
uint8_t readUVSensor(bool force)
{
  static uint8_t lastUV = 0;

  if (force)
  {
   lastUV = 200;
  }
  uint8_t temp = UV.readUV();

  if(lastUV != temp)
  {
    float uv_flt = 1.0*temp/10;
    send(msgUVindex.set(uv_flt,1));
    lastUV = temp;
    #if DEBUG_RCC
    Serial.print(" UVI:");
    Serial.print(uv_flt, 1);
    Serial.println();
    #endif
  }

  return temp;
}
#endif

uint8_t MinutesToIntervalCounts(uint64_t current_sensor_report_interval_ms, uint8_t mins)
{
    return static_cast<uint8_t>(static_cast<uint64_t>(mins)*60*1000/current_sensor_report_interval_ms);
}

void switchClock(unsigned char clk)
{
  cli();

  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;
  sei();
  highfreq = false;
}

#if TEMP_HUM_SENSOR
void readHTU21DTemperature(bool force)
{
  static float lastTemp = 0;

  if (force)
  {
   lastTemp = -100;
  }
  float temp = myHTU21D.readTemperature();

  if(lastTemp != temp)
  {
    send(msgTemp.set(temp,1));
    lastTemp = temp;
    #ifdef DEBUG_RCC
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.println();
    #endif
  }
}

void readHTU21DHumidity(bool force)
{
  static float lastHumidity = 0;

  if (force)
  {
    lastHumidity = -100;
  }
  float humd = myHTU21D.readHumidity();

  if(lastHumidity != humd)
  {
    send(msgHum.set(humd,1));
    lastHumidity = humd;
    #ifdef DEBUG_RCC
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}

#endif