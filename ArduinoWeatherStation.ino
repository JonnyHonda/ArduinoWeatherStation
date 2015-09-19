// BMP085 include libs
#include <Wire.h>
#include <Adafruit_BMP085.h>

// Instansiate the bmp objet
Adafruit_BMP085 bmp;

// DHT libs
#include "DHT.h"
#define DHTPIN 5
#define DHTTYPE DHT22   // DHT 22  (AM2302)

// Instansiate the DHT object
DHT dht(DHTPIN, DHTTYPE);

const int ledPin = 13;       // the pin that the LED is attached to

const int  rainTipperPin = 2; // Pin the rain tipper is connected to, this *must* be an interrupt enabled pin

// Some vaiable declared as volatile for use in the rain tipper interupt
volatile unsigned long int  rainTipperCounter = 0;
volatile int rainTipperState = 0;
volatile int lastRainTipperState = 0;

const int  anemometerPin = 3; // Pin the anemometer is connected to, this *must* be an interrupt enabled pin

// Some vaiable declared as volatile for use in the anemometer interupt
volatile unsigned long int  anemometerCounter = 0;
volatile int anemometerState = 0;
volatile int lastAnemometerState = 0;

// Various variables for use with the non interrupt sensors
float temperature = 0;
float pressure = 0;
float dhtTemperature = 0;
float humidity = 0;

int interval = 2000;
unsigned long int t = 0;
const int solarCellPin = 1; // Analog pin A1
int solarCellValue = 0;

float windSpeed = 0;
int anemometerInterval  = 1000;
unsigned long ta = 0;

// Some variables for the wind direction sensor
const int windDirectionPin = 2; // Analog pin A2
int windDirectionValue = 0;
char* windDirectionText = "N";
int windOrdinal = 0;


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

  // Setup the interrupt callback functions
  attachInterrupt(digitalPinToInterrupt(rainTipperPin), incrementRainTippper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(anemometerPin), anemometerFunction, CHANGE);
}

void loop() {
  solarCellValue = analogRead(solarCellPin);
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();

  // Note: regarding the anemometer.
  // according to documentation found here http://www.philpot.me/weatherinsider.html
  // one reverlution per second is 2.4 kph
  if (millis() > (anemometerInterval + ta)) {
    windSpeed = (float)anemometerCounter / 2.4;
    anemometerCounter = 0;
    ta = millis();
  }

  // At this point I can't implement the wind direction sensor but it will go something like this
  // depending of what the values come out as based on the voltage used, the resistor values one 16(ish) values
  // will be returned here.
  // It would make more sense to just return an ordinal value, but then we have plenty of program space.
  // NOTE: THESE VALUES ARE MOCK JUST TO PROVE A POINT
  windDirectionValue = analogRead(windDirectionPin);
  if (windDirectionValue >= 0 && windDirectionValue < 128) {
    strcpy(windDirectionText, "N");
    windOrdinal = 0;
  } else if (windDirectionValue >= 128 && windDirectionValue < 192) {
    strcpy(windDirectionText, "NNE");
    windOrdinal = 1;
  } else if (windDirectionValue >= 192 && windDirectionValue < 256) {
    strcpy(windDirectionText, "NE");
    windOrdinal = 2;
  } else if (windDirectionValue >= 256 && windDirectionValue < 320) {
    strcpy(windDirectionText, "ENE");
    windOrdinal = 3;
  } else if (windDirectionValue >= 320 && windDirectionValue < 384) {
    strcpy(windDirectionText, "E");
    windOrdinal = 4;
  } else if (windDirectionValue >= 384 && windDirectionValue < 448) {
    strcpy(windDirectionText, "ESE");
    windOrdinal = 5;
  } else if (windDirectionValue >= 448 && windDirectionValue < 960) {
    strcpy(windDirectionText, "SE");
    windOrdinal = 6;
  } else if (windDirectionValue >= 307 && windDirectionValue < 512) {
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
  } else if (windDirectionValue >=960) {
    strcpy(windDirectionText, "NNW");
    windOrdinal = 15;
  }
  // Mmmmm No default - now WHAT!!

  // The dht22 is slow but I want to avoid putting a delay in that freezes the code
  // so we'll use a millis interval
  if (millis() > (interval + t)) {
    digitalWrite(ledPin, HIGH);
    humidity = dht.readHumidity();
    dhtTemperature = dht.readTemperature();

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
    t = millis();
    digitalWrite(ledPin, LOW);
  }

}

void anemometerFunction() {
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
  //for next time through the loop
  lastAnemometerState = anemometerState;
}

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
  //for next time through the loop
  lastRainTipperState = rainTipperState;
}

