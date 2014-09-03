// TODO: add check for blocked fan
// TODO: detect open window and set fan full speed

#include <OneWire.h>
#include <Wire.h>
#include <avr/io.h>
#include <math.h>

#define PWM_64KHZ _BV(CS20)  // no prescale
#define PWM_8KHZ _BV(CS21)   // divide by 8
#define PWM_2KHZ _BV(CS21) | _BV(CS20) // divide by 32
#define PWM_1KHZ _BV(CS22)  // divide by 64

#define FREQ PWM_64KHZ

#define TACHO_PIN 3
#define TIMEOUT 100000

#define K1 6.11213
#define K2 17.5043
#define K3 241.2

#define LN08 -0.223143551 // natural logarithm of 80%
#define MIN_AW08_DIFF 150

#define CMDLEN 10
#define BUFLEN 10

OneWire thermo(2);

byte i;
double hum;
double tempAir;
double tempWall;
double tempDew;
double tempAw08;
byte fanSpeed;
double logHum;
long lastReading;
char command[CMDLEN];
char buffer[BUFLEN];
byte index;
boolean enableFan;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initPWM(FREQ);
  setSpeed(0);
  pinMode(TACHO_PIN, INPUT);
  lastReading = 0;
  index = 0;
  enableFan = true;
}

void loop() {
  // if more than one second has passed read sensors again
  if (abs(lastReading - millis()) > 1000) {
    readHyt221(0x28, &hum, &tempAir);
    readDS1820(&tempWall);
    tempDew = getDewPoint(tempAir, hum);
    tempAw08 = getAw08(hum, tempDew);
    if(enableFan) {
      setFanSpeed(tempWall, tempAw08);
    }
    lastReading = millis();
  }
  // read bytes from serial port
  if (Serial.available() > 0) {
    buffer[index] = Serial.read();
    // if newline is received, the command is complete
    if (buffer[index] == '\n') {
      strncpy(command, buffer, index);
      strcat(command, "\0");
      index = 0;
    }
    // stop incrementing index at end of buffer to avoid overflow
    else if (index >= BUFLEN){
      index = BUFLEN - 1;
    }
    // increment index to read next character
    else {
      index++;
    }
  }
  // react to received command
  if (strcmp(command, "")) {
    if (!strcmp(command, "meas") || !strcmp(command, "MEAS")) {
      sendData();
    }
    if (!strcmp(command, "fan off") || !strcmp(command, "FAN OFF")) {
      enableFan = false;
      setSpeed(0);
      Serial.print("Fan off. OK.\n");
    }
    if (!strcmp(command, "fan on") || !strcmp(command, "FAN ON")) {
      enableFan = true;
      Serial.print("Fan on. OK.\n");
    }
    for (i = 0; i < CMDLEN; i++) {
      command[i] = '\0';
    }
  }
}

void sendData() {
  Serial.print(hum);
  Serial.print("\t");
  Serial.print(tempAir);
  Serial.print("\t");
  Serial.print(tempWall);
  Serial.print("\t");
  Serial.print(tempDew);
  Serial.print("\t");
  Serial.print(tempAw08);
  Serial.print("\t");
  Serial.print(tempWall - tempAw08);
  Serial.print("\t");
  Serial.print(fanSpeed);
  Serial.print("\t");
  Serial.print(getSpeed());
  Serial.print("\n");
}

byte readHyt221(byte addr, double *hum, double *temp) {
  int rawT, rawH;
  // send Measurement Request
  Wire.beginTransmission(addr);
  Wire.write(0);
  Wire.available();
  Wire.read();
  Wire.endTransmission();
  // wait for measurement to finish
  delay(100);
  // get measurement data
  Wire.beginTransmission(addr);
  Wire.requestFrom(int(addr), 4);
  rawH = Wire.read()<<8;
  rawH |= Wire.read();
  rawT = Wire.read()<<8;
  rawT |= Wire.read();
  rawT = rawT>>2;
  Wire.write(0);
  Wire.endTransmission();

  *hum = (100.0 / 16383.0) * rawH;
  *temp = (165.0 / 16383.0) * rawT - 40;
  
  return 0;
}

byte readDS1820(double *temp) {
  byte i;
  byte present = 0;
  byte addr[8];
  byte data[12];
  byte type_s;
  
  // search for 1st sensor on the network
  if (!thermo.search(addr)) {
    // if none found return error code
    *temp = 0.0;
    return 1;
  }
  // Test CRC of address
  if (OneWire::crc8(addr, 7) != addr[7]) {
    *temp = 0.0;
    return 1;
  }
  // reset search, otherwise the next run yields the next sensor
  // or an error due to no more addresses found
  thermo.reset_search();
  // get chip type from first address byte
  switch(addr[0]) {
    case 0x10:
      // DS18S20
      type_s = 1;
      break;
    case 0x28:
      // DS18B20
      type_s = 0;
      break;
    case 0x22:
      // DS1820
      type_s = 0;
      break;
    default:
      return 1;
  }
  thermo.reset();
  thermo.select(addr);
  thermo.write(0x44, 1);
  delay(100);
  
  present = thermo.reset();
  thermo.select(addr);
  thermo.write(0xBE);
  
  for (i=0; i<9; i++) {
    data[i] = thermo.read();
  }
  // check for error in data
  if(OneWire::crc8(data, 8) != data[8]) {
    *temp = 0.0;
    return 1;
  }
  
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  *temp = (float) raw / 16.0;
  return 0;
}

void initPWM(uint8_t freq) {
  // use PWM from timer2A on PB3 (Arduino pin #11)
  TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21); // fast PWM, turn on oc2a
  TCCR2B = freq & 0x7;
  OCR2A = 0;
  pinMode(11, OUTPUT);
}

void setSpeed(uint8_t s) {
//  if (s == 0) {
//    initPWM(0b000);
//  }
//  else {
  initPWM(FREQ);
//  }
  // avoid low throttle setting to make sure
  // fan rotates
  if ((s > 0) && (s < 10))
    s = 10;
  OCR2A = s;
  fanSpeed = s;
}

double getSpeed() {
  byte oldSpeed;
  bool pinState;
  unsigned long time0, time1, time2;
  
  if (fanSpeed < 1)
    return 0.0;
  // save old speed setting
  oldSpeed = fanSpeed;
  // set motor pin high to avoid PWM disturbing the tacho signal
  setSpeed(255);
  delayMicroseconds(20);
  // get tacho pin state
  pinState = digitalRead(TACHO_PIN);
//  pinState = PIND & _BV(TACHO_PIN);
  // set timeout
  time0 = micros() + TIMEOUT;
  // wait for next edge or timeout
  while ((digitalRead(TACHO_PIN) == pinState) && (micros() < time0)) {
//  while ((PIND & _BV(TACHO_PIN) == pinState) && (micros() < time0)) {
    delayMicroseconds(20);
  }
  // take time
  time1 = micros();
  // wait for half-cycle edge
  while ((digitalRead(TACHO_PIN) == !pinState) && (micros() < time0)) {
//  while ((PIND & _BV(TACHO_PIN) == !pinState) && (micros() < time0)) {
    delayMicroseconds(20);
  }
  // take time
  time2 = micros();
  setSpeed(oldSpeed);
  if (time2-time1 > TIMEOUT)
    return 0.0;
  else
    return 15.0e6 / double(time2-time1);
}

double getDewPoint(double temp, double hum) {
  logHum = log(hum/100.0);
  return K3 * (K2 * tempAir / (K3 + tempAir) + logHum) / (K2 * K3 / (K3 + tempAir) - logHum);
}

double getAw08(double hum, double dewPoint) {
  double c;
  c = K2 * dewPoint / (K3 + dewPoint);
  return K3 * (c - LN08) / (K2 - c + LN08);
}

double setFanSpeed(double tempAir, double tempAw08) {
  int diff;
  diff = MIN_AW08_DIFF - int((tempAir - tempAw08) * 100);
  if (diff <= 0) {
    diff = 0;
  }
  else if (diff > MIN_AW08_DIFF) {
    diff = MIN_AW08_DIFF;
  }
  fanSpeed = map(diff, 0, MIN_AW08_DIFF, 0, 255);
  setSpeed(fanSpeed);
}
