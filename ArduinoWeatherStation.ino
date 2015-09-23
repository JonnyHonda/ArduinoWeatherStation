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

#define DEBUG true
// BMP085 include libs
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Instansiate the bmp objet
Adafruit_BMP085 bmp;

// DHT libs
#include "DHT.h"
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
float temperature = 0;
float pressure = 0;
float dhtTemperature = 0;
float humidity = 0;

const int dhtReadInterval = 2000;
unsigned long int t = 0;

// Solar cell variables
const int solarCellPin = 1; // Analog pin A1
int solarCellValue = 0;


float windSpeed = 0;
const int anemometerSampleInterval  = 1000;
unsigned long ta = 0;

// Some variables for the wind direction sensor
const int windDirectionPin = 2; // Analog pin A2
int windDirectionValue = 0;
char* windDirectionText;

int windOrdinal = 0;

// Variables used in controling when console display messages are outputted
const int displayInterval = 2000;
unsigned long int di = 0;


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

  // Default windDirectionText to some value, I chose North as windDirectionValue will be 0 (zero)
  strcpy(windDirectionText, "N");

  // Setup the interrupt callback functions
  attachInterrupt(digitalPinToInterrupt(rainTipperPin), incrementRainTippper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), incrementAnemometer, CHANGE);
}

void loop() {
#if defined(DEBUG)
  int initial = 0;
  int final = 0;
  initial = micros();
#endif

  solarCellValue = analogRead(solarCellPin);
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();

  // Note: regarding the anemometer.
  // according to documentation found here http://www.philpot.me/weatherinsider.html
  // one revolution per second is 2.4 kph
  if (millis() > (anemometerSampleInterval + ta)) {
    windSpeed = (float)anemometerCounter / 2.4;
    anemometerCounter = 0;
    ta = millis();
  }

  //call the windDirection function
  windDirection();

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
    outputToConsole();
    di = millis();
  }

#if defined(DEBUG)
  final = micros();
  Serial.print("Loop execution time: "); Serial.print(final - initial); Serial.println(" micro seconds");
#endif
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
    delay(50);
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
    delay(50);
  }
  // save the current state as the last state,
  // for comparison next time
  lastRainTipperState = rainTipperState;
}


/**
 * @name: windDirection
 * @params: none
 * @return: void
 * @description: Takes a value from the windDirectionPin and set the winDirectionText and windOrdinal
 * the text is simply N,E,S,W ,.... etc and the ordinal value is the segment the sensor is pointing in
 * 0 for North, 4 for East ... etc.
 * see http://www.philpot.me/weatherinsider.html to see what resitance values are expected
 *
 */
void windDirection() {
  // At this point I can't implement the wind direction sensor but it will go something like this
  // depending of what the values come out based on the voltage used, the resistor values one 16(ish) values
  // will be returned here.
  // It would make more sense to just return an ordinal value, but then we have plenty of program space.
  // NOTE: THESE VALUES ARE MOCK JUST TO PROVE A POINT
  // BY LEAVING A WIRE IN A2 THE PIN WILL FLOAT ALL OVER THE PLACE GIVING A GOOD SIMULATION.
  windDirectionValue = analogRead(windDirectionPin);
  if (windDirectionValue >= 0 && windDirectionValue < 64) {
    strcpy(windDirectionText, "N");
    windOrdinal = 0;
  } else if (windDirectionValue >= 64 && windDirectionValue < 128) {
    strcpy(windDirectionText, "NNE");
    windOrdinal = 1;
  } else if (windDirectionValue >= 128 && windDirectionValue < 192) {
    strcpy(windDirectionText, "NE");
    windOrdinal = 2;
  } else if (windDirectionValue >= 192 && windDirectionValue < 256) {
    strcpy(windDirectionText, "ENE");
    windOrdinal = 3;
  } else if (windDirectionValue >= 256 && windDirectionValue < 320) {
    strcpy(windDirectionText, "E");
    windOrdinal = 4;
  } else if (windDirectionValue >= 320 && windDirectionValue < 384) {
    strcpy(windDirectionText, "ESE");
    windOrdinal = 5;
  } else if (windDirectionValue >= 384 && windDirectionValue < 448) {
    strcpy(windDirectionText, "SE");
    windOrdinal = 6;
  } else if (windDirectionValue >= 448 && windDirectionValue < 512) {
    strcpy(windDirectionText, "SSE");
    windOrdinal = 7;
  } else if (windDirectionValue >= 512 && windDirectionValue < 576) {
    strcpy(windDirectionText, "S");
    windOrdinal = 8;
  } else if (windDirectionValue >= 576 && windDirectionValue < 640) {
    strcpy(windDirectionText, "SSW");
    windOrdinal = 9;
  } else if (windDirectionValue >= 640 && windDirectionValue < 704) {
    strcpy(windDirectionText, "SW");
    windOrdinal = 10;
  } else if (windDirectionValue >= 704 && windDirectionValue < 768) {
    strcpy(windDirectionText, "WSW");
    windOrdinal = 11;
  } else if (windDirectionValue >= 768 && windDirectionValue < 832) {
    strcpy(windDirectionText, "W");
    windOrdinal = 12;
  } else if (windDirectionValue >= 832 && windDirectionValue < 896) {
    strcpy(windDirectionText, "WNW");
    windOrdinal = 13;
  } else if (windDirectionValue >= 896 && windDirectionValue < 960) {
    strcpy(windDirectionText, "NW");
    windOrdinal = 14;
  } else if (windDirectionValue >= 960) {
    strcpy(windDirectionText, "NNW");
    windOrdinal = 15;
  }
  // Mmmmm No default - now WHAT!!
}

/**
 * @name: outputToConsole
 * @params: none
 * @return: void
 * @description: Display all the stuff to serial console out
 *
 */
void outputToConsole() {
  Serial.println("**************************************");
  Serial.print ("Pressure = "); Serial.print(pressure / 100); Serial.println("mb");
  Serial.print ("Humidity = "); Serial.print(humidity); Serial.println("%");
  Serial.print ("Temperature = "); Serial.print(temperature); Serial.println("*C");
  Serial.print ("DHT Temperature = "); Serial.print(dhtTemperature); Serial.println(" *C");
  Serial.print ("Rain Tipper Counter = "); Serial.println(rainTipperCounter);
  Serial.print ("Anemometer Counter = "); Serial.println(anemometerCounter);
  Serial.print ("Wind Speed = "); Serial.print(windSpeed); Serial.println("kph");
  Serial.print ("Wind Direction Value = "); Serial.println(windDirectionValue);
  Serial.print ("Wind Direction Text = "); Serial.println(windDirectionText);
  Serial.print ("Wind Direction Ordinal = "); Serial.println(windOrdinal);
  Serial.print ("Solar Cell Value = "); Serial.println(solarCellValue);
  Serial.println();
}

