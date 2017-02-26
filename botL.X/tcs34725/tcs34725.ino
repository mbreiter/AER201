//Interface Arduino with PIC over I2C Connection
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
char sort_bottle = 0;   // 1=eksa cap, 2=eksa no cap, 3=yop cap, 4=yop no cap 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
int sensorPin1 = 2;
int sensorValue1 = 0;
int sensorPin2 = 3;
int sensorValue2 = 0;
  
void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  
  // read from the colour sensor
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  // read from IR sensors
  sensorValue1 = digitalRead(sensorPin1);
  sensorValue2 = digitalRead(sensorPin2);

//  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
//  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
//  Serial.print("Red: "); Serial.print(r, DEC); Serial.print(" ");
//  Serial.print("Green: "); Serial.print(g, DEC); Serial.print(" ");
//  Serial.print("Blue: "); Serial.print(b, DEC); Serial.print(" ");
//  Serial.print("Clear: "); Serial.print(c, DEC); Serial.print(" ");
//  Serial.println(" ");

  if (lux < 100) {
    Serial.print("Detecting  ");
  }
  
  // TODO: test plain white yop bottle

  // yop bottles almost always have a very high lux
  if (lux > 1500) {
    if (((sensorValue1 == 0) & (sensorValue2 == 0))) {
      // there is a cap detected
      if ((r > g) & (r > b) & r>2000){
        //  red is detected, and it is intense enough to register a yop
        sort_bottle = 3;
        Serial.println("RED YOP CAP");
      } else if(((b > r) & (b > g) & b>2000)) {
      //  blue is detected, and it is intense enough to register a yop
          sort_bottle = 3;
          Serial.println("BLUE YOP CAP");
      }
      else if (c > 10000) {
      // iffy on if this works how i want it to, but it will be yop
          sort_bottle = 3;
          Serial.println("YOP CAP");
      }
      else {
        // eska with cap
        sort_bottle = 1;
        Serial.println("ESKA CAP");
      }      
    } else if( ((sensorValue1 == 0) & (sensorValue2 == 1)) | ((sensorValue1 == 1) & (sensorValue2 == 0))){
      // here no cap is detection, go through same protocal as above
      if ((r > g) & (r > b) & r>2000){
        //  red is detected, and it is intense enough to register a yop
        sort_bottle = 4;
        Serial.println("RED YOP NO CAP");
      } else if(((b > r) & (b > g) & b>2000)) {
      //  blue is detected, and it is intense enough to register a yop
          sort_bottle = 4;
          Serial.println("BLUE YOP NO CAP");
      }
      else if (c > 10000) {
      // iffy on if this works how i want it to, but it will be yop
          sort_bottle = 4;
          Serial.println("YOP NO CAP");
      }
      else {
        // eska without cap
        sort_bottle = 2;
        Serial.println("ESKA NO CAP");
      }      
    }     
  } else {
    if (((sensorValue1 == 0) & (sensorValue2 == 0))) {
        // eska with cap
        sort_bottle = 1;
        Serial.println("ESKA CAP");    
    } else {
        sort_bottle = 2;
        Serial.println("ESKA NO CAP");    
    }   
  }
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
  Wire.write(sort_bottle + 0x30); // respond with message of 1 byte
}
