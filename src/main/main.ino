#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define SEALEVELPRESSURE_HPA (1033)

Adafruit_BME280 bme280;

int redPin = 6;
int groundPin = 7;
int greenPin = 8;
int bluePin = 9;

void setup() {

  // Setup
  Serial.begin(9600);
  
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  pinMode(groundPin, OUTPUT);
  digitalWrite(groundPin, LOW);

  // Check
  
  setColor(0, 0, 250);
  delay(500);
  if (!bme280.begin(0x76))
  {
    while(true) {
      setColor(255, 0, 0);
      delay(1000);
      setColor(0, 0, 0);  
    }
  } else {
    setColor(0, 250, 0);
    }

}

void loop() {
  Serial.print("Temperature = ");
  Serial.print(bme280.readTemperature());
  Serial.println("*C");

  Serial.print("Pressure = ");
  Serial.print(bme280.readPressure() / 100.0F);
  Serial.println("hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme280.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println("m");

  Serial.print("Humidity = ");
  Serial.print(bme280.readHumidity());
  Serial.println("%");

  Serial.println();
  delay(1000);
}


void setColor(int red, int green, int blue) {
  
  #ifdef COMMON_ANODE
  
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  
  #endif
  
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
