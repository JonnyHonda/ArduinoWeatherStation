/**
 * WH1080 inspired weather station for the Arduino.
 *
 * Author John Burrin
 *
 * Date 20 Sept 2015

 *
 * Requires:
 *          BMP085 Library from  https://github.com/adafruit/Adafruit-BMP085-Library
 *          DHT22 Library from https://github.com/adafruit/DHT-sensor-library
 *
 * Hardware:
 *          Arduino Uno
 *          DHT22 humidity sensor
 *          BMP085 pressure sensor
 *          N77NF Maplin Rain Gauge
 *          N81NF Maplin Wind Direction
 *          N09QR Maplin Wind Speed
 *          A random solar cell, or with the correct pull up resistor a CDR
**/
void incrementAnemometer();
void incrementRainTippper();
void outputToConsole();
void windDirection();


#include <Arduino.h>
//#define DEBUG true
//BMP085 include libs
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
//#include <Adafruit_BMP085.h>

// Instansiate the bmp objet
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// DHT libs
#include <DHT.h>
#include <DHT_U.h>
const int  dhtPin = 5;
const int dhtType = DHT22;

// Instansiate the DHT object
DHT dht(dhtPin, dhtType);

const int ledPin = 13;       // the pin that the LED is attached to

const int  rainTipperPin = 2; // Pin the rain tipper is connected to, this *must* be an interrupt enabled pin

// Some variables declared as volatile for use in the rain tipper interupt
volatile unsigned long int  rainTipperCounter = 0;
volatile int rainTipperState = 0;
volatile int lastRainTipperState = 0;

const int  anemometerPin = 3; // Pin the anemometer is connected to, this *must* be an interrupt enabled pin

// Some variables declared as volatile for use in the anemometer interupt
volatile unsigned long int  anemometerCounter = 0;
volatile int anemometerState = 0;
volatile int lastAnemometerState = 0;

// Various variables for use with the non interrupt sensors
double temperature = 0;
double pressure = 0;
double dhtTemperature = 0;
double humidity = 0;

const int dhtReadInterval = 2000;
unsigned long int t = 0;

// Solar cell variables
const int solarCellPin = 1; // Analog pin A1
int solarCellValue = 0;


double windSpeed = 0;
const int anemometerSampleInterval  = 1000;
unsigned long ta = 0;

// Some variables for the wind direction sensor
const int windDirectionPin = 2; // Analog pin A2
unsigned long int windDirectionValue = 0;

int windOrdinal = 0;

// Variables used in controling when console display messages are outputted
const int displayInterval = 2000;
unsigned long int di = 0;

struct packet {
  byte header[3];
  byte data[10];
  byte checksum[2];
} my_packet;

void setup() {
  // initialize the rain tipper pin as a input:
  pinMode(rainTipperPin, INPUT);

  // initialize the anemometer pin as a input:
  pinMode(anemometerPin, INPUT);

  // initialize the wind direction pin as a input:
  pinMode(windDirectionPin, INPUT);

  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  dht.begin();
  bmp.begin();


// Set the header bytes.
  my_packet.header[0] = 'A';
  my_packet.header[1] = 'W';
  my_packet.header[2] = 'S';

  // Setup the interrupt callback functions
  attachInterrupt(digitalPinToInterrupt(rainTipperPin), incrementRainTippper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), incrementAnemometer, CHANGE);
  Serial.println("Setup Complete");
}


void loop() {

  solarCellValue = analogRead(solarCellPin);
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    bmp.getTemperature(&temperature);
    pressure = event.pressure;
  }

  // Note: regarding the anemometer.
  // according to documentation found here http://www.philpot.me/weatherinsider.html
  // one revolution per second is 2.4 kph
  if (millis() > (anemometerSampleInterval + ta)) {
    windSpeed = (double)anemometerCounter / 2.4;
    anemometerCounter = 0;
    ta = millis();
  }

  windDirectionValue = analogRead(windDirectionPin);

  // The dht22 is slow but I want to avoid putting a delay in that freezes the code
  // so we'll use a millis interval check
  if (millis() > (dhtReadInterval + t)) {
    // Turn the led on
    digitalWrite(ledPin, HIGH);
    humidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature();

    t = millis();
    // Turn the led off
    digitalWrite(ledPin, LOW);
  }

  // Push all the data console
  if (millis() > (displayInterval + di)) {
    //outputToConsole();
    int packet_temperature = (temperature + 120) * 10;
    my_packet.data[1] = (byte) (packet_temperature & 0xFF);         // Low byte
    my_packet.data[0] = (byte) ((packet_temperature >> 8) & 0xFF);  // High byte

    int packet_pressure = (pressure + 800) * 10;
    my_packet.data[3] = (byte) (packet_pressure & 0xFF);         // Low byte
    my_packet.data[2] = (byte) ((packet_pressure >> 8) & 0xFF);  // High byte

// Wind Ordinal
    my_packet.data[4] = (byte) windDirectionValue;

// Humidity
    my_packet.data[5] = (byte) humidity;


// Rain tipper counter
    long rain_tipper = rainTipperCounter;
    my_packet.data[6] = (rain_tipper >> 24) & 0xFF;
    my_packet.data[7] = (rain_tipper >> 16) & 0xFF;
    my_packet.data[8] = (rain_tipper >> 8) & 0xFF;
    my_packet.data[9] = rain_tipper & 0xFF;

/// Perform checksum
    int checksum = 0;
    for (int i = 0; i < 10; ++i) {
      checksum += my_packet.data[i];
    }

    my_packet.checksum[1] = (byte) (checksum & 0xFF);         // Low byte
    my_packet.checksum[0] = (byte) ((checksum >> 8) & 0xFF);  // High byte

    for (int l = 0; l < 3; l++) {
      Serial.write(my_packet.header[l]);
    }
    for (int l = 0; l < 10; l++) {
      Serial.write(my_packet.data[l]);
    }
    for (int l = 0; l < 2; l++) {
      Serial.write(my_packet.checksum[l]);
    }
    Serial.println();

    di = millis();
  }

}

/**
 * @name: incrementAnemometer
 * @params: none
 * @return: void
 * @description: This function increments the anemometerCounter by one each time it is called.
 * this function is supposed to be called by interrupt and employs a simple form of switch debouncing in software
 *
 */
void incrementAnemometer() {
  // read the anemometer input pin:
  anemometerState = digitalRead(anemometerPin);

  // compare the anemometerState to its previous state
  if (anemometerState != lastAnemometerState) {
    // if the state has changed, increment the counter
    if (anemometerState == HIGH) {
      anemometerCounter++;
    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state,
  // for comparison next time
  lastAnemometerState = anemometerState;
}

/**
 * @name: incrementRainTippper
 * @params: none
 * @return: void
 * @description: This function increments the rainTipperCounter by one each time it is called.
 * this function is supposed to be called by interrupt and employs a simple form of switch debouncing in software
 *
 */
void incrementRainTippper() {
  // read the Tipper input pin:
  rainTipperState = digitalRead(rainTipperPin);

  // compare the rainTipperState to its previous state
  if (rainTipperState != lastRainTipperState) {
    // if the state has changed, increment the counter
    if (rainTipperState == HIGH) {
      // if the current state is HIGH then increment the counter
      rainTipperCounter++;
    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state,
  // for comparison next time
  lastRainTipperState = rainTipperState;
}

