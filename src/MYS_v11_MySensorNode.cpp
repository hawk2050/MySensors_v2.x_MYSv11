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

// Enable signal report functionalities
//#define MY_SIGNAL_REPORT_ENABLED

// Enable and select radio type attached
//#define MY_RADIO_RF24
//#define MY_RADIO_RFM69
#define MY_RADIO_RFM95

#ifdef MY_RADIO_RFM69
// RFM69
//#define MY_RADIO_RFM69
//#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
//#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
//#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW

#endif

#ifdef MY_RADIO_RFM95
/*These are all the defaults anyway*/
#define MY_RFM95_FREQUENCY RFM95_868MHZ
#define MY_RFM95_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM95_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW
#endif

#define MY_NODE_ID 8

#ifdef HALL
#define MY_NODE_ID 7  /*7 = Hall */
#endif

#ifdef LILY
#define MY_NODE_ID 20 /*20 = Lily's room */
#endif
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
#ifdef MY_RADIO_RF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
//#define MY_RF24_CHANNEL 100
#define MY_RF24_CHANNEL 76
#endif

#include <stdint.h>
#include <math.h> 
#include <MySensors.h> 


#include "SparkFunHTU21D.h"
#include <Wire.h>

#include <BatterySense.hpp>

#define UV_SENSOR 0
#define TEMP_HUM_SENSOR 1

/*External voltage monitor is only really needed if you're powering the MCU from a voltage regulator but you want to monitor
the battery or solar panel voltage that is feeding the regulator.
*/
#define EXTERNAL_VOLTAGE_MONITOR 0 //Measure voltage at A0, resistive divider to reduce voltage at Mysv11 header J5, Pin 5.

// Sleep time between sensor updates (in milliseconds)
//static const uint32_t DAY_UPDATE_INTERVAL_MS = 60000;
//static const uint32_t DAY_UPDATE_INTERVAL_MS = 5000;

static const uint32_t DAY_UPDATE_INTERVAL_MS = 300000;


enum child_id_t
{
  CHILD_ID_HUMIDITY,
  CHILD_ID_TEMP,
  CHILD_ID_UV,
  CHILD_ID_VOLTAGE,
  CHILD_ID_EXT_VOLTAGE,
  CHILD_ID_UPLINK_QUALITY,
  CHILD_ID_TX_LEVEL,       
  CHILD_ID_TX_PERCENT,
  CHILD_ID_TX_RSSI,
  CHILD_ID_RX_RSSI,
  CHILD_ID_TX_SNR,
  CHILD_ID_RX_SNR
};

#ifdef MY_SIGNAL_REPORT_ENABLED
// Initialize general message
MyMessage msgTxRSSI(CHILD_ID_TX_RSSI, V_CUSTOM);
MyMessage msgRxRSSI(CHILD_ID_RX_RSSI, V_CUSTOM);
MyMessage msgTxSNR(CHILD_ID_TX_SNR, V_CUSTOM);
MyMessage msgRxSNR(CHILD_ID_RX_SNR, V_CUSTOM);
MyMessage msgTxLevel(CHILD_ID_TX_LEVEL, V_CUSTOM);
MyMessage msgTxPercent(CHILD_ID_TX_PERCENT, V_CUSTOM);
MyMessage msgUplinkQuality(CHILD_ID_UPLINK_QUALITY, V_CUSTOM);
#endif

//std::map<child_id_t, std::string> m {{CHILD_ID_HUMIDITY,"Humidity"}, {CHILD_ID_TEMP,"Temperature"}, {CHILD_ID_UV,"UV"}, {CHILD_ID_VOLTAGE,"MCU Voltage"}, {CHILD_ID_EXT_VOLTAGE,"External Voltage"}};

uint32_t clockSwitchCount = 0;

/*****************************/
/********* FUNCTIONS *********/
/*****************************/

//Create an instance of the sensor objects
#if UV_SENSOR
#include <UVSensor.hpp>
#define MY_UVIS25_POWER_PIN 2
const char sketchString[] = "mys_v11-uv";
UVSensor UV(MY_UVIS25_POWER_PIN); //Ultraviolet sensor
MyMessage msgUVindex(CHILD_ID_UV, V_UV);

#endif


#if EXTERNAL_VOLTAGE_MONITOR
int lastVoltage = 5000;                     // set to an arbitary number outside of expected voltage sensor range to ensure a change when first run
int extVoltagePin = A0;                         // analog pin voltage sensor or voltage divider is connected to
int extVoltSenseMax = 4200;                    // set to the maximum input voltage in millivolts of your voltage divider input   
MyMessage msgExtVolt(CHILD_ID_EXT_VOLTAGE, V_VOLTAGE);
void readExtVoltage(int pin, bool force);
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
bool throttlefreq = false;
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
  Serial.begin(115200);
  
}

void presentation()
{
   // Send the sketch version information to the gateway and Controller
  sendSketchInfo(sketchString, "0.6");
  // Register all sensors to gateway (they will be created as child devices)
  present(CHILD_ID_VOLTAGE, S_MULTIMETER, "BATT VOLT mV");

#if UV_SENSOR
  present(CHILD_ID_UV, S_UV);
#endif

#if TEMP_HUM_SENSOR
  present(CHILD_ID_HUMIDITY, S_HUM, "REL HUM %");
  present(CHILD_ID_TEMP, S_TEMP, "TEMP degC");
#endif

#if EXTERNAL_VOLTAGE_MONITOR
present(CHILD_ID_EXT_VOLTAGE, S_MULTIMETER);
#endif

#ifdef MY_SIGNAL_REPORT_ENABLED
// Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_UPLINK_QUALITY, S_CUSTOM, "UPLINK QUALITY RSSI");
	present(CHILD_ID_TX_LEVEL, S_CUSTOM, "TX LEVEL DBM");
	present(CHILD_ID_TX_PERCENT, S_CUSTOM, "TX LEVEL PERCENT");
	present(CHILD_ID_TX_RSSI, S_CUSTOM, "TX RSSI");
	present(CHILD_ID_RX_RSSI, S_CUSTOM, "RX RSSI");
	present(CHILD_ID_TX_SNR, S_CUSTOM, "TX SNR");
	present(CHILD_ID_RX_SNR, S_CUSTOM, "RX SNR");
#endif   
}


void loop()
{

  uint32_t update_interval_ms = DAY_UPDATE_INTERVAL_MS;

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
    
    #if DEBUG_RCC
    Serial.print("Setting CPU Freq to 4MHz");
    Serial.println();
    #endif
    switchClock(0x01); // divide by 2, to give 4MHz on 8MHz, 3V3 Pro Mini
    cpu_is_throttled = true;
    throttlefreq = false;
  } //end if

  uint16_t battLevel = batt.getVoltage();
  send(msgVolt.set(battLevel,1));

#ifdef MY_SIGNAL_REPORT_ENABLED
  // send messages to GW
	send(msgUplinkQuality.set(transportGetSignalReport(SR_UPLINK_QUALITY)));
	send(msgTxLevel.set(transportGetSignalReport(SR_TX_POWER_LEVEL)));
	send(msgTxPercent.set(transportGetSignalReport(SR_TX_POWER_PERCENT)));
	// retrieve RSSI / SNR reports from incoming ACK
	send(msgTxRSSI.set(transportGetSignalReport(SR_TX_RSSI)));
	send(msgRxRSSI.set(transportGetSignalReport(SR_RX_RSSI)));
	send(msgTxSNR.set(transportGetSignalReport(SR_TX_SNR)));
	send(msgRxSNR.set(transportGetSignalReport(SR_RX_SNR)));
#endif

#if TEMP_HUM_SENSOR
  readHTU21DTemperature(true);
  readHTU21DHumidity(true);
#endif

#if EXTERNAL_VOLTAGE_MONITOR
  readExtVoltage(extVoltagePin, true);
#endif


#if UV_SENSOR
  UV.read_sensor();

#if DEBUG_RCC
    Serial.print("UV reading is :");
    Serial.print(UV.get_uvi(), 1);
    Serial.println();
#endif
  
  send(msgUVindex.set(UV.get_uvi(),1));
#if DEBUG_RCC
    Serial.print("Power down UV sensor");
    Serial.println();
#endif
#endif /*UV_SENSOR*/

  sleep(update_interval_ms); 
  
  
} //end loop





void switchClock(unsigned char clk)
{
  cli();

  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;
  sei();
  
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
    #if DEBUG_RCC
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
    #if DEBUG_RCC
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}

#endif

#if EXTERNAL_VOLTAGE_MONITOR
#define NUM_SAMPLES 10
void readExtVoltage(int pin, bool force)
{
  uint8_t sample_count = 0;
  int sum = 0;
  while (sample_count < NUM_SAMPLES)
  {                                   // take a number of voltage samples  
    sum += analogRead(pin);
    sample_count++;
    delay(10);
  }

  int voltageI = map(sum/NUM_SAMPLES,0,1023,0,extVoltSenseMax);              // map the reading and get our result in millivolts

  #if DEBUG_RCC
  Serial.print("sum count..."); Serial.println((sum / NUM_SAMPLES));      // print the count result. will be between 0 and 1023
  Serial.print("mapped volts..."); Serial.println(voltageI);  // convert millivolts back to volts and print. the 1 at the end determines how many decimal places to show
  #endif
  

  //if ( voltageI != lastVoltage)
  //{                                         // check if we have a new value. only send data if it is different
  send(msgExtVolt.set(voltageI));                  // voltagel is in millivolts so we divide by 1000 to convert back to volts and
                                                                            // send voltage message to gateway with 1 decimal place
  //  lastVoltage = voltageI;                                                // copy the current voltage reading for testing on the next loop 
  //}
}

#endif
