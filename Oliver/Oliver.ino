#include <SoftwareSerial.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define PINServo 9
#define PINVoltKnob A0
#define ONE_WIRE_BUS 7

#define addr 0x0D //I2C Address for The HMC5883
Servo ServoM;
int ValueKnob;
int ValueServ;
int angle = 10;

MPU6050 mpu;
//SoftwareSerial BTSerial(BT_RX, BT_TX);
#define BTSerial Serial2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasSensors(&oneWire);
Adafruit_BME280 bme = Adafruit_BME280();

char    serial_buf[100];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int error = 0;

void readMPUData();
void readHMCData();
//void readTemperature();
void readHumidity();
/********************************************************************/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  BTSerial.begin(9600);
  ServoM.attach(PINServo);
  ServoM.write(10);
  Wire.begin();
  //Serial.println(Wire.begin() > 0 ? "HMC5883L sensor found" : "HMC5883L sensor not found");

  Wire.beginTransmission(addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // Set the Register
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // Set the Register
  Wire.endTransmission();

  mpu.initialize();
  delay(500);
  dallasSensors.begin();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.println(dallasSensors.getDS18Count() > 0 ? "Temperature sensor found" : "Temperature sensor not found");
  Serial.println(bme.begin(0x76) > 0 ? "Humidity sensor found" : "Humidity sensor not found");
}

void loop(void)
{
  readMPUData();
  readHMCData();
  readHumidity();
  ValueKnob = analogRead(PINServo);
  ValueServ = map(ValueKnob, 0, 1023, 5, 175); // will map knob value range to servo value range
  ServoM.write(ValueServ); // shaft of servo will start to rotate.

  // scan from 0 to 180 degrees
  for (angle = 10; angle < 180; angle++)
  {
    ServoM.write(angle);
    delay(15);
  }
  // now scan back from 180 to 0 degrees
  for (angle = 180; angle > 10; angle--)
  {
    ServoM.write(angle);
    delay(15);
  }

  delay(1000);
  Serial.println(".");
}

void readMPUData() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(1500);
    sprintf(serial_buf, "X%d|Y%d|Z%d|GX%d|GY%d|GZ%d|", ax, ay, az, gx, gy, gz);
    Serial.println(serial_buf);
    BTSerial.write(serial_buf);
  }
}
void readHMCData() {

  int x, y, z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(addr);
  Wire.write(0x00); //start with register 3.
  Wire.endTransmission();

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(addr, 6);
  if (6 <= Wire.available()) {
    x = Wire.read(); //MSB  x
    x |= Wire.read() << 8; //LSB  x
    z = Wire.read(); //MSB  z
    z |= Wire.read() << 8; //LSB z
    y = Wire.read(); //MSB y
    y |= Wire.read() << 8; //LSB y
  }
  delay(500);
  Serial.print("|MX");
  BTSerial.print("|MX");
  Serial.print(x);
  BTSerial.print(x);
  Serial.print("|MY");
  BTSerial.print("|MY");
  Serial.print(y);
  BTSerial.println(y);
  Serial.print("|MZ");
  BTSerial.print("|MZ");
  Serial.println(z);
  BTSerial.print(z);
  Serial.print("|");
}

void readHumidity() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure();

  if (temp != 0 || hum != 0 || pres != 0)
  {
    Serial.print("|BT");
    BTSerial.print("|BT");
    Serial.print(temp);
    BTSerial.print(temp);
    Serial.print("|BH");
    BTSerial.print("|BH");
    Serial.print(hum);
    BTSerial.print(hum);
    Serial.print("|BP");
    BTSerial.print("|BP");
    Serial.print(pres);
    BTSerial.print(pres);
    Serial.print("|");
    BTSerial.print("|");
  }
}
