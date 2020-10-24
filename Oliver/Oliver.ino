/// Start Add Libraries
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
/// End Of Libraries

/// Start of Definiations
Servo ServoM;
MPU6050 mpu;
#define BTSerial Serial2 //(17(RX), 16(TX))
Adafruit_BME280 bme = Adafruit_BME280();
/// End of Definiations

/// Start of parameter Declarations
#define addr 0x0D /// I2C Address for The HMC5883L
int     angle = 10;   /// Just dummy Motor Control position angle
char    serial_buf[100];
int16_t ax, ay, az;
int16_t gx, gy, gz;
/// End of Parameter Declarations

/// Start of Function Definiation
void readMPUData();
void readHMCData();
void readHumidity();
/// End of function Definiation

/********************************************************************/
void setup(void)
{
  // Initialize Software Serial port
  Serial.begin(9600);
  delay(500);
  // Initialize Bluetooth Serial port
  BTSerial.begin(9600);
  delay(500);

  // Initialize HMC5883L Communication
  Wire.begin();  /// used for HMC5883L Sensor
  //Serial.println(Wire.begin() > 0 ? "HMC5883L sensor found" : "HMC5883L sensor not found");
  Wire.beginTransmission(addr);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(addr); //start talking
  Wire.write(0x09); // Tell the HMC5883L to Continuously Measure
  Wire.write(0x1D);
  Wire.endTransmission();
  delay(500);
  /// End of HMC5883L Initialize

  /// MPU Initialize
  mpu.initialize();
  delay(500);
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  /// BME Initialize (0x76 is the I2C CHIP ID)
  Serial.println(bme.begin(0x76) > 0 ? "Humidity sensor found" : "Humidity sensor not found");
}

void loop(void)
{
  //// Contineously measure data
  readMPUData();
  readHMCData();
  readHumidity();
  ////

  ////////////////////////// Dummy Test Servo
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
  //////////////////////// End of Dummy Test Servo
  delay(1000);
}
///////////////////// MPU Sensor data
void readMPUData() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    delay(1500);
    sprintf(serial_buf, "X%d|Y%d|Z%d|GX%d|GY%d|GZ%d|", ax, ay, az, gx, gy, gz);
    Serial.println(serial_buf);
    BTSerial.write(serial_buf);
  }
}

///////////////////// HMC Sensor data
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
///////////////////// Temp. Humidity and Pressure Sensor data
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
