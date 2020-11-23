/*
   Project Flight Simulator
   Developer: Janybasha Shaik
   Board Arduino UNO
   
 * */
/// Start of Libraries
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
#include <LIDARLite.h>
#include <SoftwareSerial.h>
/// End Of Libraries

/// Start of Definiations
Servo ServoM;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servoBluetooth;
MPU6050 mpu;
LIDARLite myLidarLite;
SoftwareSerial BTSerial(0, 1);

#define PINServo 9
#define PINServoX 6
#define PINServoY 7
#define PINServoZ 8
#define PINServoB 10

#define NO             0
#define YES            1
#define COMMAND_SERVO  'S'

Adafruit_BME280 bme = Adafruit_BME280();
/// End of Definiations

/// Start of parameter Declarations
#define addr 0x0D /// I2C Address for The HMC5883L
int     angle = 10;   /// Just dummy Motor Control position angle
char    serial_buf[100];
int ax1B = 0;
int ay1B = 0;
int az1B = 0;
int LidarVal = 0;
int prevVal1 = 0;
int prevVal2 = 0;
int prevVal3 = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int timer = 0;
bool        packet_complete_;
char        packet_[100],
            *packet_ptr_;
/// End of Parameter Declarations

/// Start of Function Definiation
void clear_packet();
void read_packet();
void readMPUData();
void readHMCData();
void readHumidity();
void command_servo(char packet[]);
void LIDAR();
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
  ServoM.attach(PINServo); /// attach the Servo Control on PIN 9
  servoBluetooth.attach(PINServoB); /// attach the Servo Control on PIN 10
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); // Change this number to try out alternate configurations


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
  clear_packet();
}

/********************************************************************/
void loop(void)
{
  read_packet();
  if (packet_complete_) {
    switch (packet_[0]) {
      case  COMMAND_SERVO:
        command_servo(packet_);
        break;
    }

    packet_complete_ = NO;
    packet_ptr_ = packet_;
  }

  //// Contineously measure data
  Serial.print("D");
  BTSerial.print("D");
  readMPUData();
  readHMCData();
  readHumidity();
  Serial.println();
  BTSerial.println();
  LIDAR();
  delay(100);
}

/********************************************************************/
/* MPU Sensor data */
void readMPUData() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sprintf(serial_buf, "X%d|Y%d|Z%d|GX%d|GY%d|GZ%d|", ax, ay, az, gx, gy, gz);
    Serial.println(serial_buf);
    BTSerial.write(serial_buf);

    int val1 = map(ax, -17000, 17000, 0, 179);
    if (abs(val1 - prevVal1) > 10)
    {
      servo1.attach(6);
      servo1.write(val1);
      Serial.println("ax: ");
      Serial.println(val1);
      prevVal1 = val1;
    }

    int val2 = map(ay, -17000, 17000, 0, 179);
    if (abs(val2 - prevVal2) > 10)
    {
      servo2.attach(7);
      servo2.write(val2);
      Serial.println("ay: ");
      Serial.println(val2);
      prevVal2 = val2;
    }

    int val3 = map(az, -17000, 17000, 0, 179);
    if (abs(val3 - prevVal3) > 10)
    {
      servo3.attach(8);
      servo3.write(val3);
      Serial.println("az: ");
      Serial.println(val3);
      prevVal3 = val3;
    }
    delay(100);
    servo1.detach();
    servo2.detach();
    servo3.detach();
  }
}

/********************************************************************/
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
  Serial.print(z);
  BTSerial.print(z);
  Serial.print("|");
  BTSerial.print("|");
}

/********************************************************************/
///////////////////// Temp. Humidity and Pressure Sensor data
void readHumidity() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure();

  if (temp != 0 || hum != 0 || pres != 0)
  {
    Serial.print("BT");
    BTSerial.print("BT");
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

/********************************************************************/
/*LIDAR Distance measuring*/
void LIDAR()
{
  Serial.println(myLidarLite.distance());
  int Dist = myLidarLite.distance();
  if (Dist > 180)
  {
    LidarVal = 180;
  }
  else
  {
    LidarVal = Dist;
  }
  ServoM.attach(PINServo); /// attach the Servo Control on PIN 9
  ServoM.write(LidarVal);
  delay(300);
  ServoM.detach();
}

/********************************************************************/
void read_packet()
{
  char  chr,
        *ptr;

  int   num_bytes;
  if (BTSerial.available() && !packet_complete_) {
    Serial.println("Available");
    ////////////////////////////////////////////////////////////////////
    // read characters into the array 'buf' until a newline character //
    // is encountered. Replace the newline ('\n') with a null ('\0'). //
    ////////////////////////////////////////////////////////////////////
    while (!packet_complete_  &&  BTSerial.available()) {
      chr = BTSerial.read();
      if (chr != '\r') {
        if (chr == '\n') {
          chr = '\0';
          packet_complete_ = YES;
        }

        *packet_ptr_++ = chr;
        if (packet_complete_) {
          Serial.print("Packet read:");
          Serial.println(packet_);
        }
      }
    }
  }
}

/********************************************************************/
void clear_packet()
{
  packet_ptr_ = packet_;
  *packet_ptr_ = '\0';
  packet_complete_ = NO;
}

/********************************************************************/
void command_servo(char packet[]) {
  int value = atoi(&packet[2]);
  if (packet[1] == 'X') {
    Serial.print("Servo:");
    Serial.println(value);
    // ServoM.attach(PINServo); /// attach the Servo Control on PIN 10
    servoBluetooth.write(value);
    delay(500);
  }
}
