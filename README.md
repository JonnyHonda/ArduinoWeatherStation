# ArduinoWeatherStation
A basic weather station based on the Arduino and several common sensors

Connect the DHT 22 as normal with data pin connected to pin 5 on the Arduino.
Connect the BMP085 as normal, you'll use Analog pins A4 and A5 for this.

If you have a Solar Cell connect this to GND and A1

The Rain counter switch needs a 10k pull up resistor and use Digital pin D2

See https://flic.kr/p/yx3bLy

The same anemometer, but use digital pin D3
See https://flic.kr/p/yMkHqY

For the wind direction you'll need a resistance on Analog A2, 
but if you just put a lead in it it will float and give a good simulation.

Have fun.

I'll improve this when the real sensors arrive.

JB
