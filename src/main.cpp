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

// Instansiate the bmp object
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
float temperature = 0.0;
double pressure = 0;
double dhtTemperature = 0;
double humidity = 0;

const int dhtReadInterval = 2000;
unsigned long int t = 0;

// Light Sensor variables
const int AnalogLightValuePin = 1; // Analog pin A1
int AnalogLightValue = 0;


double windSpeed = 0;
const int anemometerSampleInterval  = 1000;
unsigned long ta = 0;

// Some variables for the wind direction sensor
const int windDirectionPin = 2; // Analog pin A2
byte windDirectionValue = 0;

byte windOrdinal = 0;

// Variables used in controling when console display messages are outputted
const int displayInterval = 2000;
unsigned long int di = 0;

const int data_size = 14;
struct packet {
  byte header[3];
  byte data[data_size];
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

  AnalogLightValue = analogRead(AnalogLightValuePin);
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
  if (windDirectionValue >= 15 && windDirectionValue <= 17){
    //N
    windOrdinal = 0;
  }
  else if (windDirectionValue >= 148 && windDirectionValue <= 150){ 
    //NNE
    windOrdinal = 1;
  }
  else if (windDirectionValue >= 204 && windDirectionValue <= 206){ 
    //NE
    windOrdinal = 2;
  }
  else if (windDirectionValue >= 79 && windDirectionValue <= 81){ 
    //ENE
    windOrdinal = 3;
  }
  else if (windDirectionValue >= 89 && windDirectionValue <= 91){ 
    //E
    windOrdinal = 4;
  }
  else if (windDirectionValue >= 60 && windDirectionValue <= 65){
    //ESE
    windOrdinal = 5;
  }
  else if (windDirectionValue >= 180 && windDirectionValue <= 185){
    //SE
    windOrdinal = 6;
  }
  else if (windDirectionValue >= 121 && windDirectionValue <= 124){
    //SSE
    windOrdinal = 7;
  }
    else if (windDirectionValue >= 29 && windDirectionValue <= 31){
    //S
    windOrdinal = 8;
  }
  
  else if (windDirectionValue >= 240 && windDirectionValue <= 243){
    //SSW
    windOrdinal = 9;
  }
  else if (windDirectionValue >= 115 && windDirectionValue <= 119){
    //SW
    windOrdinal = 10;
  }
  else if (windDirectionValue >= 85 && windDirectionValue <= 88){
     //WSW
     windOrdinal = 11;
   }
   else if (windDirectionValue >= 176 && windDirectionValue <= 178){
     //W
     windOrdinal = 12;
   
   }else if (windDirectionValue >= 58 && windDirectionValue <= 62){
     //WNW
     windOrdinal = 13;
   }
   else if (windDirectionValue >= 118 && windDirectionValue <= 122){
     //NW
     windOrdinal = 14;
   }
   else if (windDirectionValue >= 190 && windDirectionValue <= 195){
     //NNW
     windOrdinal = 15;
   }
  



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
    int packet_temperature = (temperature + 120) * 10;
    my_packet.data[1] = (byte) (packet_temperature & 0xFF);         // Low byte
    my_packet.data[0] = (byte) ((packet_temperature >> 8) & 0xFF);  // High byte

    int packet_pressure = (pressure + 800) * 10;
    my_packet.data[3] = (byte) (packet_pressure & 0xFF);         // Low byte
    my_packet.data[2] = (byte) ((packet_pressure >> 8) & 0xFF);  // High byte

// Wind Ordinal
   my_packet.data[4] = (byte) windOrdinal;
   // my_packet.data[4] = (byte) windDirectionValue;

// Humidity
    my_packet.data[5] = (byte) humidity;


// Rain tipper counter
//    Serial.println(rainTipperCounter);
    unsigned long int rain_tipper = rainTipperCounter;
    my_packet.data[9] = rain_tipper & 0xFF;
    my_packet.data[8] = (rain_tipper >> 8) & 0xFF;
    my_packet.data[7] = (rain_tipper >> 16) & 0xFF;
    my_packet.data[6] = (rain_tipper >> 24) & 0xFF;
//
//Serial.print (windSpeed);
// Wind speed - Range 0 to 45 m/s
int packet_windspeed = (windSpeed + 45) * 100;
    my_packet.data[11] = (byte) (packet_windspeed & 0xFF);         // Low byte
    my_packet.data[10] = (byte) ((packet_windspeed >> 8) & 0xFF);  // High byte

//Serial.println (AnalogLightValue);
int packet_lightValue = AnalogLightValue;
    my_packet.data[13] = (byte) (packet_lightValue & 0xFF);         // Low byte
    my_packet.data[12] = (byte) ((packet_lightValue >> 8) & 0xFF);  // High byte


/// Perform checksum
    int checksum = 0;
    for (int i = 0; i < data_size; ++i) {
      checksum += my_packet.data[i];
    }

    my_packet.checksum[1] = (byte) (checksum & 0xFF);         // Low byte
    my_packet.checksum[0] = (byte) ((checksum >> 8) & 0xFF);  // High byte

#ifdef DEBUG
    Serial.println(temperature);
    Serial.println(pressure);
    Serial.println(windDirectionValue);
    Serial.println(humidity);
Serial.println(rainTipperCounter);
Serial.println (windSpeed);
Serial.println (AnalogLightValue);
#endif

    for (int l = 0; l < 3; l++) {
      Serial.write(my_packet.header[l]);
    }
    for (int l = 0; l < data_size; l++) {
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

