  #include <Wire.h>
  #include "Adafruit_TCS34725.h"
  /* Connect SCL    to analog 5
     Connect SDA    to analog 4
     Connect VDD    to 3.3V DC
     Connect GROUND to common ground */
  
  /* Initialise with default values (int time = 2.4ms, gain = 1x) */
  // Adafruit_TCS34725 tcs = Adafruit_TCS34725();
  
  /* Initialise with specific int time and gain values */
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
  
  int sensorPin1 = 2;
  int sensorValue1 = 0;
  int sensorPin2 = 3;
  int sensorValue2 = 0;
  
  char sort_bottle = 0;
  int state = 0;
  char incomingByte;
  char buf[3];
  int counter = 0;
  
  void setup(void) {
    // join the i2c bus with address 8
    Wire.begin(8);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
  
    Serial.begin(9600);
  }
  
  void loop(void) {
    uint16_t r, g, b, c, colorTemp, lux;
  
    // read from the colour sensor
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  // read from IR sensors
  sensorValue1 = digitalRead(sensorPin1);
  sensorValue2 = digitalRead(sensorPin2);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("Red: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("Green: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("Blue: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("Clear: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  if (lux < 100) {
    Serial.print("Detecting  ");
  }

  else if (colorTemp < 7000) {
    if ((sensorValue1 == 0) & (sensorValue2 == 0)) {
      Serial.print("Eska Water Bottle Detected with cap  ");
      sort_bottle = 0;
    }
    else if ((sensorValue1 == 1) & (sensorValue2 == 0)) {
      Serial.print("Eska Water Bottle Detected without cap  ");
      sort_bottle = 1;
    }
    else if ((sensorValue1 == 0) & (sensorValue2 == 1)) {
      Serial.print("Eska Water Bottle Detected without cap  ");
      sort_bottle = 1;
    }
    else {
      Serial.print("Eska Water Bottle Detected  ");
      sort_bottle = 0;
    }
  }
  else if ((r > g) & (r > b) & (colorTemp >= 7000)) {
    if ((sensorValue1 == 0) & (sensorValue2 == 0)) {
      Serial.print("Red YOP Bottle Detected with cap  ");
      sort_bottle = 2;
    }
    else if ((sensorValue1 == 1) & (sensorValue2 == 0)) {
      Serial.print("Red YOP Bottle Detected without cap ");
      sort_bottle = 3;
    }
    else if ((sensorValue1 == 0) & (sensorValue2 == 1)) {
      Serial.print("Red YOP Bottle Detected without cap ");
      sort_bottle = 3;
    }
    else {
      Serial.print("Red YOP Bottle Detected ");
      sort_bottle = 2;
    }
  }
  else if ((b > g) & (b > r) & (colorTemp >= 7000)) {
    if ((sensorValue1 == 0) & (sensorValue2 == 0)) {
      Serial.print("Blue YOP Bottle Detected with cap  ");
      sort_bottle = 2;
    }
    else if ((sensorValue1 == 1) & (sensorValue2 == 0)) {
      Serial.print("Blue YOP Bottle Detected without cap  ");
      sort_bottle = 3;
    }
    else if ((sensorValue1 == 0) & (sensorValue2 == 1)) {
      Serial.print("Blue YOP Bottle Detected without cap  ");
      sort_bottle = 3;
    }
    else {
      Serial.print("Blue YOP Bottle Detected");
      sort_bottle = 3;
    }
  }
  else if (colorTemp > 7000) {
    if ((sensorValue1 == 0) & (sensorValue2 == 0)) {
      Serial.print("YOP Bottle Detected - Colour Unknown with cap  ");
      sort_bottle = 2;
    }
    else if ((sensorValue1 == 1) & (sensorValue2 == 0)) {
      Serial.print("YOP Bottle Detected - Colour Unknown without cap ");
      sort_bottle = 3;
    }
    else if ((sensorValue1 == 0) & (sensorValue2 == 1)) {
      Serial.print("YOP Bottle Detected - Colour Unknown without cap ");
      sort_bottle = 3;
    }
    else {
      Serial.print("YOP Bottle Detected - Colour Unknown  ");
      sort_bottle = 2;
    }
  }


  if (state && Serial.available() > 0 && !incomingByte) {
    incomingByte = Serial.read();
  }
}

// sending sorting determination to the pic
void requestEvent() {
  Wire.write(sort_bottle);    // send one byte of data, containing result of detection
}







