//Interface Arduino with PIC over I2C Connection
//Remember to enable the Arduino-PIC switches on RC3 and RC4!

#include <Wire.h>
#include "Adafruit_TCS34725.h"
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include <Stepper.h>

#define STEPS 200
#define TCAADDR 0x70

Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X);

char incomingByte;
char sort_bottle = 2;   // 1=eksa cap, 2=eksa no cap, 3=yop cap, 4=yop no cap

int switchPin = 2;
int bump = 0;

int servoPin = 4;
int pulse = 1500;
int pos1 = 1300;
int pos2 = 1600;
int pos3 = 1800;
int pos4 = 2150;
int position = pos1;
int servoState = LOW;
long previousMillis = 0;
int colour_sum1, colour_sum2;

// digital pins for the stepper motor
Stepper stepper(STEPS, 5, 6, 7, 8);
int enable = 9;
int proceed_detection = 0;

// hall effect sensor
int hallPin = 0;

// dc motor
int dcOn = 0;
int dcDirection = 1;
int dcPinFor = 11;
int dcPinBack = 12;
int stepCount = 0;
int stepDiff;

// sorting information
int ESKA = 0;
int ESKA_NC = 0;
int YOP = 0;
int YOP_NC = 0;
int TOTAL = 0;
int DONE_PIN = 3;
char TRANSFER_INFO = 0;
int last_bottle = 0;
unsigned long START_TIME;
int state = 1;

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
  pinMode(dcPinFor, OUTPUT);
  digitalWrite(dcPinFor, LOW);
  digitalWrite(dcPinBack, LOW);
  pinMode(dcPinBack, OUTPUT);
  pinMode(enable, OUTPUT);

  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, HIGH);

  digitalWrite(enable, HIGH);
  stepper.setSpeed(8);

  Wire.begin(8);                // join i2c bus with address 8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  state = 1;

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

  Serial.println("======= BOTTLE COUNTS =======");
  Serial.print("ESKA WITH CAP "); Serial.println(ESKA);
  Serial.print("ESKA WITHOUT CAP "); Serial.println(ESKA_NC);
  Serial.print("YOP WIT CAP "); Serial.println(YOP);
  Serial.print("YOP WITHOUT CAP "); Serial.println(YOP_NC);
  Serial.println();

  if (proceed_detection == 1) {
    //    if(dcDirection == 1) {
    //      dcDirection = 0;
    //      digitalWrite(dcPinFor, HIGH);   // do analog
    //      digitalWrite(dcPinBack, LOW);
    //    } else {
    //      dcDirection = 1;
    //      digitalWrite(dcPinFor, LOW);
    //      digitalWrite(dcPinBack, HIGH);
    //    }

    digitalWrite(dcPinFor, HIGH);   // do analog

    stepper.step(5);
    detections();
    checkDone();
  } else {
    digitalWrite(dcPinFor, LOW);
    digitalWrite(dcPinBack, LOW);
  }

  delay(100);
}

void checkDone() {
    if (TOTAL == 10 || (millis() - START_TIME) / 1000 > 90) {
      digitalWrite(DONE_PIN, LOW);
    } else if (TOTAL >= 6 && (millis() - START_TIME) / 1000 > 120) {
      digitalWrite(DONE_PIN, LOW);
    } else if (TOTAL >= 8 && (millis() - START_TIME) / 1000 > 60) {
      digitalWrite(DONE_PIN, LOW);
    }
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

  colour_sum1 = r1 + g1 + b1;
  colour_sum2 = r2 + g2 + b2;

  //  if (((float)lux1/(float)lux2 > 1.8 || (float)lux2/(float)lux1 > 1.8) && sort_bottle == 5 ) {
  if (bump == HIGH && sort_bottle == 5) {
    TOTAL += 1;

    if (colour_sum1 + colour_sum2 > 3000) {
      // yop bottle
      if ((float)r1 / (float)b1 > 1.6 || (float)r2 / (float)b2 > 1.6) {
        // with a cap
        sort_bottle = 3;
        position = pos3;
        YOP += 1;
      } else {
        // without a cap
        sort_bottle = 4;
        position = pos4;
        YOP_NC += 1;
      }
    } else {
      // eska bottle
      if ( ((float)b1 / (float)r1) > 1 || ((float)b2 / (float)r2) > 1) {
        // with a cap
        sort_bottle = 1;
        position = pos1;
        ESKA += 1;
      } else {
        // without a cap
        sort_bottle = 2;
        position = pos2;
        ESKA_NC += 1;
      }
    }
  }
  else {
    sort_bottle = 5;
  }

  Serial.println("======= COLOUR SENSOR 1 =======");
  Serial.print("RED "); Serial.println(r1);
  Serial.print("GREEN "); Serial.println(g1);
  Serial.print("BLUE "); Serial.println(b1);
  Serial.print("TEMP "); Serial.println(colorTemp1);
  Serial.print("LUX "); Serial.println(lux1);
  Serial.println();

  Serial.println("======= COLOUR SENSOR 2 =======");
  Serial.print("RED "); Serial.println(r2);
  Serial.print("GREEN "); Serial.println(g2);
  Serial.print("BLUE "); Serial.println(b2);
  Serial.print("TEMP "); Serial.println(colorTemp2);
  Serial.print("LUX "); Serial.println(lux2);
  Serial.println();

  Serial.println("======= TEST CRITERIA =======");
  Serial.print(" r1 + g1 + b1 = "); Serial.println(r1 + g1 + b1);
  Serial.print(" r2 + g2 + b2 = "); Serial.println(r2 + g2 + b2);
  Serial.print(" r1/b1 = "); Serial.println((float)r1 / (float)b1);
  Serial.print(" r2/b2 = "); Serial.println((float)r2 / (float)b2);
  Serial.print(" b1/r1 = "); Serial.println((float)b1 / (float)r1);
  Serial.print(" b2/r2 = "); Serial.println((float)b2 / (float)r2);
  Serial.print(" lux1/lux2 = "); Serial.println((float)lux1 / (float)lux2);
  Serial.print(" lux2/lux1 = "); Serial.println((float)lux2 / (float)lux1);
  Serial.println();

  Serial.println("======= DETERMINATION =======");
  Serial.println((int) sort_bottle);
  Serial.println();
}

void receiveEvent(int howMany) {
  char x = Wire.read();      // receive byte as char

  if ((int) x == 1) {
    START_TIME = millis();

    ESKA = 0;
    ESKA_NC = 0;
    YOP = 0;
    YOP_NC = 0;
    TOTAL = 0;

    proceed_detection = (int) 1;
  } else if ((int) x == 2) {
    state = 1;
    proceed_detection = 0;
  } else if ((int) x == 3) {
    state = 2;
    proceed_detection = 0;
  } else if ((int) x == 4) {
    state = 3;
    proceed_detection = 0;
  } else if ((int) x == 5) {
    state = 4;
    proceed_detection = 0;
  }

}

void requestEvent() {

  if (state == 1) {
    Wire.write((char) ESKA);
  } else if (state == 2) {
    Wire.write((char) ESKA_NC);
  } else if (state == 3) {
    Wire.write((char) YOP);
  } else {
    Wire.write((char) YOP_NC);
  }
}
