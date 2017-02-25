//Interface Arduino with PIC over I2C Connection
//Outputs keypad char to serial monitor until AAA sequence is given
//Then it will output serial input to LCD display
//Remember to enable the Arduino-PIC switches on RC3 and RC4! 

#include <Wire.h>
#include "Adafruit_TCS34725.h"


void setup() {
  Wire.begin(8);                // join i2c bus with address 8
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent); 

  Serial.begin(9600);           
}

int state = 1;
char incomingByte;
char sort_bottle = 0;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
int sensorPin1 = 2;
int sensorValue1 = 0;
int sensorPin2 = 3;
int sensorValue2 = 0;
  
void loop() {
  if (state && Serial.available() > 0 && !incomingByte) {
    incomingByte = Serial.read();
  }

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

    if ((r > g) & (r > g)) {
    sort_bottle=1;
  } else if ((g > b) & (g > r)) {
    sort_bottle=2;
  } else if ((b > g) & (b > r)) {
    sort_bottle=3;
  }

  Serial.println(sort_bottle, DEC);
  
}

char buf[3];
int counter=0;

void receiveEvent(int howMany) {

  char x = Wire.read();    // receive byte as char
  Serial.println(x);       // print to serial output

  buf[counter++] = x;
  counter=counter==3?0:counter;

  if(buf[0]=='A' && buf[1]=='A' && buf[2]=='A'){
    state = 1;
  }
}

void requestEvent() {
  Wire.write(sort_bottle + 0x31); // respond with message of 1 byte
set  incomingByte=0;           // clear output buffer
}
