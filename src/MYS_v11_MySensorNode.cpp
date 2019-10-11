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

#define MY_NODE_ID 11
/*Makes this static so won't try and find another parent if communication with
gateway fails*/
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC

/**
 * @def MY_TRANSPORT_WAIT_READY_MS
 * @brief Timeout in ms until transport is ready during startup, set to 0 for no timeout
 */
#define MY_TRANSPORT_WAIT_READY_MS (1000)

/*These are actually the default pins expected by the MySensors framework.
* This means we can use the default constructor without arguments when
* creating an instance of the Mysensors class. Other defaults will include
* transmitting on channel 76 with a data rate of 250kbps.
* MyGW1 = channel 76
* MyGW2 = channel 100
*/
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
#define MY_RF24_CHANNEL 100

#define MY_UVIS25_POWER_PIN 2

#include <MySensors.h> 
#include <stdint.h>
#include <math.h> 
#include "SparkFunHTU21D.h"
#include <Wire.h>
#include <UVSensor.hpp>
#include <BatterySense.hpp>

#define UV_SENSOR 0
#define TEMP_HUM_SENSOR 1

// Sleep time between sensor updates (in milliseconds)
static const uint32_t DAY_UPDATE_INTERVAL_MS = 30000;
static const uint32_t NIGHT_UPDATE_INTERVAL_MS = 900000;//15 mins

//static const uint32_t DAY_UPDATE_INTERVAL_MS = 10000;


enum child_id_t
{
  CHILD_ID_HUMIDITY,
  CHILD_ID_TEMP,
  CHILD_ID_UV,
  CHILD_ID_VOLTAGE
};

uint32_t clockSwitchCount = 0;

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

//Create an instance of the sensor objects
#if UV_SENSOR
const char sketchString[] = "mys_v11-uv";
UVSensor UV(MY_UVIS25_POWER_PIN); //Ultraviolet sensor
MyMessage msgUVindex(CHILD_ID_UV, V_UV);

#endif



#if TEMP_HUM_SENSOR
const char sketchString[] = "mys_v11-temp_humid";
HTU21D myHTU21D;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
void readHTU21DTemperature(bool force);
void readHTU21DHumidity(bool force);
#endif


MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);
void switchClock(unsigned char clk);

/*Set true to have clock throttle back, or false to not throttle*/
bool throttlefreq = true;
bool cpu_is_throttled = false;

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
  #if UV_SENSOR
  UV.init();
  #endif
  #if TEMP_HUM_SENSOR
  myHTU21D.begin();
  #endif
  Serial.begin(9600);
  
}

void presentation()
{
   // Send the sketch version information to the gateway and Controller
  sendSketchInfo(sketchString, "0.6");
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

  uint32_t update_interval_ms = DAY_UPDATE_INTERVAL_MS;

  //loopCount++;
  clockSwitchCount++;
  #if DEBUG_RCC
  Serial.print("clockSwitchCount = ");
  Serial.print(clockSwitchCount,DEC);
  Serial.println();
  #endif
  
  // When we wake up the 5th time after power on, switch to 1Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependent on oscillator settings).
  if ( (clockSwitchCount == 5) && throttlefreq)
  {
    /* Switch to 4Mhz by setting clock prescaler to divide by 2 for the reminder of the sketch, 
     * to save power but more importantly to allow operation down to 1.8V
     * 
      */
    //switchClock(1<<CLKPS2); // divide by 16
    #if DEBUG_RCC
    Serial.print("Setting CPU Freq to 4MHz");
    Serial.println();
    #endif
    switchClock(0x01); // divide by 2, to give 4MHz on 8MHz, 3V3 Pro Mini
    cpu_is_throttled = true;
  } //end if

  uint16_t battLevel = batt.getVoltage();
  send(msgVolt.set(battLevel,1));

#if TEMP_HUM_SENSOR
  readHTU21DTemperature(true);
  readHTU21DHumidity(true);
#endif

#if UV_SENSOR
  UV.wake();
  /*Give the voltage time to stabilise(?)*/
  wait(40); //ms
  UV.read_sensor();
  
  send(msgUVindex.set(UV.get_uvi(),1));
  UV.sleep();

  if(UV.is_night())
  {
    update_interval_ms = NIGHT_UPDATE_INTERVAL_MS;
    #if DEBUG_RCC
    Serial.print("Set sleep interval to Night Mode");
    Serial.println();
    #endif
  }
  else
  {
    update_interval_ms = DAY_UPDATE_INTERVAL_MS;
    #if DEBUG_RCC
    Serial.print("Set sleep interval to Day Mode");
    Serial.println();
    #endif
  }
  
#endif /*UV_SENSOR*/

  sleep(update_interval_ms); 
  
  
} //end loop





void switchClock(unsigned char clk)
{
  cli();

  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;
  sei();
  throttlefreq = false;
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