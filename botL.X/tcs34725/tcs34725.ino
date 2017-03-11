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
char sort_bottle = 2;   // 1=eksa cap, 2=eksa no cap, 3=yop cap, 4=yop no cap, 5=no bottle 
int irPin = 2;          // ir sensor on digital pin 2
int ir = 0;
uint16_t r1, g1, b1, c1, colorTemp1, lux1, r2, g2, b2, c2, colorTemp2, lux2;


void setup() {
  Wire.begin(8);        // join i2c bus with address 8
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent); 

  Serial.begin(9600);    
  while (!Serial) ;
 
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;
    
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
         Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }  
}

void detection() {
  // read from IR sensor
  ir = digitalRead(irPin);

  // select colour sensor number 1
  tcaselect(2);

  // read from the colour sensor 1
  tcs1.getRawData(&r1, &g1, &b1, &c1);
  colorTemp1 = tcs1.calculateColorTemperature(r1, g1, b1);
  lux1 = tcs1.calculateLux(r1, g1, b1);

  Serial.println(" ");
  Serial.println("colour sensor 1");
  Serial.println(" ");
  
  Serial.print("Color Temp: "); Serial.print(colorTemp1, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux1, DEC); Serial.print(" - ");
  Serial.print("Red: "); Serial.print(r1, DEC); Serial.print(" ");
  Serial.print("Green: "); Serial.print(g1, DEC); Serial.print(" ");
  Serial.print("Blue: "); Serial.print(b1, DEC); Serial.print(" ");
  Serial.print("Clear: "); Serial.print(c1, DEC); Serial.println(" ");
  Serial.print("ir: "); Serial.print(ir); Serial.println(" ");

  // select colour sensor number 2
  tcaselect(6);

  // read from the colour sensor 2
  tcs2.getRawData(&r2, &g2, &b2, &c2);
  colorTemp2 = tcs2.calculateColorTemperature(r2, g2, b2);
  lux2 = tcs2.calculateLux(r2, g2, b2);

  Serial.println(" ");
  Serial.println("colour sensor 2");
  Serial.println(" ");

  Serial.print("Color Temp: "); Serial.print(colorTemp2, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux2, DEC); Serial.print(" - ");
  Serial.print("Red: "); Serial.print(r2, DEC); Serial.print(" ");
  Serial.print("Green: "); Serial.print(g2, DEC); Serial.print(" ");
  Serial.print("Blue: "); Serial.print(b2, DEC); Serial.print(" ");
  Serial.print("Clear: "); Serial.print(c2, DEC); Serial.println(" ");
  Serial.print("ir: "); Serial.print(ir); Serial.println(" ");

  //sort_bottle = random(1, 5);   // use this for testing
  if(ir == 0) {
    // there is a cap detected
    
    if (((r1 > g1) & (r1 > b1) & r1>2000) | ((r2 > g2) & (r2 > b2) & r2>2000)){
      //  red is detected, means it is a yop with a cap
      sort_bottle = 3;
      
    } else if(((b1 > r1) & (b1 > g1) & b1>2000) & ((c2 > r2) & (c2 > g2) & (c2 > b2) & b1>2000)) {
      //  blue is detected and white is detected. yop with cap.
        sort_bottle = 3;
    } else if(((b2 > r2) & (b2 > g2) & b2>2000) & ((c1 > r1) & (c1 > g1) & (c1 > b1) & b1>2000)) {
      //  blue is detected and white is detected. yop with cap.
        sort_bottle = 3;
        
    } else if(((c1 > r1) & (c1 > g1) & (c1 > b1) & c1>2000) & ((c2 > r2) & (c2 > g2) & (c2 > b2) & c2>2000)) {
      //  only white is detected. yop without cap.
        sort_bottle = 4;
    } 

    else if(((b1 > r1) & (b1 > g1) & b1>2000) & ((b2 > r2) & (b2 > g2) & b2>2000)) {
      //  eska with a cap.
        sort_bottle = 1;
    } 

    else {
        sort_bottle = 2;
    }   
  } else {
      sort_bottle = 5;
  }
}
  
void loop() {
  detection()
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
  detection()
  Wire.write(sort_bottle); // respond with message of 1 byte
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
