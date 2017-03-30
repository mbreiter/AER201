//Interface Arduino with PIC over I2C Connection
//Remember to enable the Arduino-PIC switches on RC3 and RC4!

#include <Wire.h>
#include "Adafruit_TCS34725.h"
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70

Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

int state = 1;
char incomingByte;
char sort_bottle = 2;   // 1=eksa cap, 2=eksa no cap, 3=yop cap, 4=yop no cap
int switchPin = 2;
int bump = 0;
int servoPin = 4;
int pulse = 1500;
int pos1 = 1300;
int pos2 = 1550;
int pos3 = 1850;
int pos4 = 2150;
int position = pos1;

int servoState = LOW;
long previousMillis = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  while (!Serial);
  delay(1000);

  pinMode(switchPin, INPUT);
  pinMode(servoPin, OUTPUT);

  Wire.begin(8);                // join i2c bus with address 8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.begin(9600);

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
      }
    }
  }

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  if (millis() - previousMillis > 20) {
    servoControl(position);
    previousMillis = millis();
  }
}

void loop() {
  detections();
}

void servoControl(int position) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(position);
  digitalWrite(servoPin, LOW);
  delay(20);
}

char buf[3];
int counter = 0;

void detections() {
  uint16_t r1, g1, b1, c1, colorTemp1, lux1, r2, g2, b2, c2, colorTemp2, lux2;

  // read from bump switch
  bump = digitalRead(switchPin);
  // select colour sensor number 1
  tcaselect(2);

  // read from the colour sensor 1
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  colorTemp1 = tcs1.calculateColorTemperature(r1, g1, b1);
  lux1 = tcs1.calculateLux(r1, g1, b1);

  // select colour sensor number 2
  tcaselect(6);

  // read from the colour sensor 2
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  colorTemp2 = tcs2.calculateColorTemperature(r2, g2, b2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  if (bump == HIGH) {
    if (((float)r1 / (float)b1 > 1.6 && (r2 + g2 + b2) > 6500) || ((float)r2 / (float)b2 > 1.6 && (r1 + g1 + b1) > 7000)) {
      sort_bottle = 3;      // yop with cap
      position = pos3;
    }
    else if ((r1 + g1 + b1) > 7000 || (r2 + g2 + b2) > 6500) {
      sort_bottle = 4;      // yop without cap
      position = pos4;
    }

    else if ((r1 + g1 + b1) > 3600 || (r2 + g2 + b2) > 3600) {
      sort_bottle = 2;
      position = pos2;
    }

    if ( ((float)b1 / (float)r1) > 1.25 || ((float)b2 / (float)r2) > 1.25 ) {
      sort_bottle = 1;
      position = pos1;
    }
  }
  else {
    sort_bottle = 5;
  }

  Serial.println((uint8_t)sort_bottle);
}

void receiveEvent(int howMany) {

  char x = Wire.read();      // receive byte as char
  Serial.println(x);       // print to serial output

  buf[counter++] = x;
  counter = counter == 3 ? 0 : counter;

  if (buf[0] == 'A' && buf[1] == 'A' && buf[2] == 'A') {
    state = 1;
  }
}

void requestEvent() {
  Wire.write(sort_bottle); // respond with message of 1 byte, in this case, the detected bottle
}
