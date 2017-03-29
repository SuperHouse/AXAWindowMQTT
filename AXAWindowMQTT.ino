/** 
 * Controls AXA electric window motors via LIN bus.
 * 
 * Connects to an MQTT broker and watches for commands. When a command
 * is received, it sends a message via the appropriate serial port which
 * is then converted to LIN format using an external module.
 * 
 * Uses external LINbus interface modules connected to the serial UARTs.
 * 
 * Message format is "motor number, command". Motor 0 is all motors. Eg:
 * 1,3  (motor 1, stop)
 * 2,1  (motor 2, open)
 * 0,2  (all motors, close)
 * 
 * Motors are:
 * 0: All motors
 * 1: Serial()
 * 2: Serial1()
 * 3: Serial2()
 * 4: Serial3()
 * 
 * Commands are:
 * 1: Open
 * 2: Close
 * 3: Stop
 * 4: Version (not implemented)
 * 5: Status  (not implemented)
 * 6: Device  (not implemented)
 * 
 * Intended for a Freetronics EtherMega: www.freetronics.com.au/ethermega
 * Suitable LIN bus interface module: www.superhouse.tv/linmod
 
 Copyright 2017 SuperHouse Automation Pty Ltd <info@superhouse.tv>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <SPI.h>          // For networking
#include <Ethernet.h>     // For networking
#include <PubSubClient.h> // For MQTT
#include "Wire.h"         // For MAC address ROM
#include "DHT.h"          // For temperature / humidity sensor

/*--------------------------- Configuration ------------------------------*/
/* Network Settings */
#define ENABLE_DHCP                 true   // true/false
#define ENABLE_MAC_ADDRESS_ROM      true   // true/false
#define MAC_I2C_ADDRESS             0x50   // Microchip 24AA125E48 I2C ROM address
static uint8_t mac[] = { 0xAA, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Default if MAC ROM is not used
IPAddress ip(192,168,1,123);               // Default if DHCP is not used

/* MQTT Settings */
const char* commandTopic = "conservatory/windows"; // MQTT topic to subscribe for commands
const char* statusTopic  = "events";               // MQTT topic to publish status reports
IPAddress broker(192,168,1,111);                   // Address of the MQTT broker
char messageBuffer[50];  // Longest message we send is how long? Probably in heartbeat.

/* Watchdog Timer Settings */
#define ENABLE_EXTERNAL_WATCHDOG        true       // true / false
#define WATCHDOG_PIN                    2          // Output to pat the watchdog
#define WATCHDOG_PULSE_LENGTH           50         // Milliseconds
#define WATCHDOG_RESET_INTERVAL         30000      // Milliseconds. Also the period for sensor reports.
long watchdogLastResetTime = 0;

/* Temperature / Humidity Sensor Settings */
// Note: Report interval is set by WATCHDOG_RESET_INTERVAL above
#define DHTTYPE DHT22
#define DHTPIN  A0
const char* temperatureTopic = "windowcontroller/temperature"; // MQTT topic for sensor reporting
const char* humidityTopic    = "windowcontroller/humidity";    // MQTT topic for sensor reporting
float temperature = 0;
float humidity    = 0;

/* LIN Bus Settings */
#define LIN_MESSAGE_DELAY 250   // Pause between messages to reduce crosstalk. Nasty hack!

/**
 * MQTT callback to process messages
 */
void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //for (int i=0;i<length;i++) {
  //  Serial.print((char)payload[i]);
  //}
  //Serial.println();
  byte motorId = payload[0];
  byte command = payload[2];

  //Serial.print("Motor ID: ");
  //Serial.write(motorId);
  //Serial.print(" Command: ");
  //Serial.write(command);
  //Serial.println("");

  switch(motorId)
  {
    case '0':
      motor0( command );
      break;
    case '1':
      motor1( command );
      break;
    case '2':
      motor2( command );
      break;
    case '3':
      motor3( command );
      break;
    case '4':
      motor4( command );
      break;
  }
}

/* Ok Bob, let's build some objects */
EthernetClient ethClient;             // Ethernet
PubSubClient   client(ethClient);     // MQTT
DHT            dht(DHTPIN, DHTTYPE);  // Temperature / humidity sensor

/**
 * Attempt connection to the MQTT broker, and try repeatedly until it succeeds
 */
void reconnect() {
  // Generate a unique MQTT client ID from our IP address
  char clientBuffer[50];
  String clientString = "Arduino-" + String(Ethernet.localIP());
  clientString.toCharArray(clientBuffer, clientString.length() + 1);
  
  while (!client.connected()) {
    //Serial.print("Attempting MQTT connection...");
    if (client.connect(clientBuffer)) {
      //Serial.println("connected");
      client.publish(statusTopic,"Window controller connected");  // Announce ourselves
      client.subscribe(commandTopic);  // Listen for incoming commands
    } else {
      //Serial.print("failed, rc=");
      //Serial.print(client.state());
      //Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}

/**
 * Setup
 */
void setup() {
  //Serial.begin(9600);
  //Serial.println("Booting");
  
  if( ENABLE_EXTERNAL_WATCHDOG )
  {
    pinMode(WATCHDOG_PIN, OUTPUT);
    digitalWrite(WATCHDOG_PIN, LOW);
  }

  if( ENABLE_MAC_ADDRESS_ROM )
  {
    //Serial.print(F("Getting MAC address from ROM: "));
    // Replace the default MAC address with a value extracted from ROM
    Wire.begin();  // Join i2c bus (I2C address is optional for the master)
    mac[0] = readRegister(0xFA);
    mac[1] = readRegister(0xFB);
    mac[2] = readRegister(0xFC);
    mac[3] = readRegister(0xFD);
    mac[4] = readRegister(0xFE);
    mac[5] = readRegister(0xFF);
  } else {
    //Serial.print(F("Using static MAC address: "));
  }
  // Print the MAC address
  //char tmpBuf[17];
  //sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  //Serial.println(tmpBuf);

  // Bring up networking
  if( ENABLE_DHCP == true )
  {
    Ethernet.begin(mac);      // Use DHCP
  } else {
    Ethernet.begin(mac, ip);  // Use static address defined above
  }
  delay(1500); // Let the Ethernet hardware sort itself out

  //Serial.print("Network ready. IP address: ");
  //Serial.println(Ethernet.localIP());

  // Set up MQTT
  client.setServer(broker, 1883);
  client.setCallback(callback);

  // Set up serial ports for the LINbus modules
  Serial.begin (19200, SERIAL_8N2);
  Serial1.begin(19200, SERIAL_8N2);
  Serial2.begin(19200, SERIAL_8N2);
  Serial3.begin(19200, SERIAL_8N2);
}

/**
 * 
 */
void loop() {
  // Make sure we're connected to the MQTT broker
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  runHeartbeat();
}


/**
 * The heartbeat takes care of both patting the watchdog and reporting sensor values
 */
void runHeartbeat()
{
  if((millis() - watchdogLastResetTime) > WATCHDOG_RESET_INTERVAL)  // Is it time to run yet?
  {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    char tempC[10];
    dtostrf(temperature,1,2,tempC);
    char relH[10];
    dtostrf(humidity,1,2,relH);

    client.publish(temperatureTopic, tempC);
    if(client.publish(humidityTopic, relH))
    {
      patWatchdog();  // Only pat the watchdog if we successfully published to MQTT
    }
    // The interval timer is updated inside patWatchdog()
  }
}

void motor0( byte command )
{
  switch(command)
  {
    case '1':
      Serial.println ("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial.println ("OPEN"); delay(LIN_MESSAGE_DELAY);
      Serial1.println("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial1.println("OPEN"); delay(LIN_MESSAGE_DELAY);
      Serial2.println("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial2.println("OPEN"); delay(LIN_MESSAGE_DELAY);
      Serial3.println("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial3.println("OPEN");
      client.publish("events", "All motors OPEN");
      break;
    case '2':
      Serial.println ("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial.println ("CLOSE"); delay(LIN_MESSAGE_DELAY);
      Serial1.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial1.println("CLOSE"); delay(LIN_MESSAGE_DELAY);
      Serial2.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial2.println("CLOSE"); delay(LIN_MESSAGE_DELAY);
      Serial3.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial3.println("CLOSE");
      client.publish("events", "All motors CLOSE");
      break;
    case '3':
      
      Serial.println ("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial1.println("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial2.println("STOP"); delay(LIN_MESSAGE_DELAY);
      Serial3.println("STOP");
      client.publish("events", "All motors STOP");
      break;
  }
}

void motor1( byte command )
{
  switch(command)
  {
    case '1':
      Serial.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial.println("OPEN");
      client.publish("events", "Motor 1 OPEN");
      break;
    case '2':
      Serial.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial.println("CLOSE");
      client.publish("events", "Motor 1 CLOSE");
      break;
    case '3':
      Serial.println("STOP");
      client.publish("events", "Motor 1 STOP");
      break;
  }
}

void motor2( byte command )
{
  switch(command)
  {
    case '1':
      Serial1.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial1.println("OPEN");
      client.publish("events", "Motor 2 OPEN");
      break;
    case '2':
      Serial1.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial1.println("CLOSE");
      client.publish("events", "Motor 2 CLOSE");
      break;
    case '3':
      Serial1.println("STOP");
      client.publish("events", "Motor 2 STOP");
      break;
  }
}

void motor3( byte command )
{
  switch(command)
  {
    case '1':
      Serial2.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial2.println("OPEN");
      client.publish("events", "Motor 3 OPEN");
      break;
    case '2':
      Serial2.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial2.println("CLOSE");
      client.publish("events", "Motor 3 CLOSE");
      break;
    case '3':
      Serial2.println("STOP");
      client.publish("events", "Motor 3 STOP");
      break;
  }
}

void motor4( byte command )
{
  switch(command)
  {
    case '1':
      Serial3.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial3.println("OPEN");
      client.publish("events", "Motor 4 OPEN");
      break;
    case '2':
      Serial3.println("STOP");  delay(LIN_MESSAGE_DELAY);
      Serial3.println("CLOSE");
      client.publish("events", "Motor 4 CLOSE");
      break;
    case '3':
      Serial3.println("STOP");
      client.publish("events", "Motor 4 STOP");
      break;
  }
}


/**
 * Pulse the hardware watchdog timer pin to reset it
 */
void patWatchdog()
{
  if( ENABLE_EXTERNAL_WATCHDOG )
  {
    digitalWrite(WATCHDOG_PIN, HIGH);
    delay(WATCHDOG_PULSE_LENGTH);
    digitalWrite(WATCHDOG_PIN, LOW);
  }
  watchdogLastResetTime = millis();
}


/**
 * Helper function used to read the MAC address ROM via I2C
 */
byte readRegister(byte r)
{
  unsigned char v;
  Wire.beginTransmission(MAC_I2C_ADDRESS);
  Wire.write(r);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(MAC_I2C_ADDRESS, 1); // Read a byte
  while(!Wire.available())
  {
    // Wait
  }
  v = Wire.read();
  return v;
}
