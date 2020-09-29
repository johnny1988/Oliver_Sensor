//#include <SoftwareSerial.h>
//#include <HardwareSerial.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BME280.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BT_RX 9 /// Change hardware Rx to 17  
#define BT_TX 8  /// Change hardware Tx to 16
#define ONE_WIRE_BUS 7

MPU6050 mpu;
#define Uart2 Serial2   ///17(RX), 16(TX)
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//SoftwareSerial BTSerial(BT_RX, BT_TX);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasSensors(&oneWire);
Adafruit_BME280 bme = Adafruit_BME280();

char    serial_buf[100];
int16_t ax, ay, az;
int16_t gx, gy, gz;

void readMPUData();
void readHMCData();
void readTemperature();
void readHumidity();
void displaySensorDetails();
/********************************************************************/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Uart2.begin(9600);
  Wire.begin();
  mpu.initialize();
 // BTSerial.begin(9600);
  delay(500);
  dallasSensors.begin();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Serial.println(mag.begin() && mag.isAvailable() ? "HMC5883 connection successful" : "HMC5883 connection failed");

  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }
  displaySensorDetails();

  Serial.println(dallasSensors.getDS18Count() > 0 ? "Temperature sensor found" : "Temperature sendor not found");
  Serial.println(bme.begin() > 0 ? "Humidity sensor found" : "Humidity sendor not found");
}

void loop(void)
{
  Serial.print("D");
  //BTSerial.print("D");
  Uart2.print("D"); /// I dont why i need this, but i just copied from above to see
  readMPUData();
  readHMCData();
  readTemperature();
  readHumidity();
  Serial.println();
  //BTSerial.println();
  Uart2.println(); /// I dont why i need this, but i just copied from above to see
  delay(100);
}

void readMPUData() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sprintf(serial_buf, "X%d|Y%d|Z%d|GX%d|GY%d|GZ%d|", ax, ay, az, gx, gy, gz);
    Serial.print(serial_buf);
    //BTSerial.print(serial_buf);
   // Uart2.print(serial_buf); /// I dont know why print, i wish to write, Uart2.write(serial_buf) I need to underdstand what is doing here
  }
}

void readHMCData()
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");
  
  /* if (mag.isAvailable())
    {
     sensors_event_t event;
     mag.getEvent(&event);
     Serial.print("MX");
     BTSerial.print("MX");
     Serial.print(event.magnetic.x);
     BTSerial.print(event.magnetic.x);
     Serial.print("|MY");
     BTSerial.print("|MY");
     Serial.print(event.magnetic.y);
     BTSerial.print(event.magnetic.y);
     Serial.print("|MZ");
     BTSerial.print("|MZ");
     Serial.print(event.magnetic.z);
     BTSerial.print(event.magnetic.z);
     Serial.print("|");
     BTSerial.print("|");
    }*/
}

void readTemperature() {
  if (dallasSensors.getDS18Count() > 0)
  {
    dallasSensors.requestTemperatures();
    Serial.println("DONE");
    float temp = dallasSensors.getTempCByIndex(0);
    Serial.print("T");
   // BTSerial.print("T");
    Serial.print(temp);
   // BTSerial.print(temp);
    Serial.print("|");
  //  BTSerial.print("|");
  }
}

void readHumidity() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float pres = bme.readPressure();

  if (temp != 0 || hum != 0 || pres != 0) {
    Serial.print("BT");
   // BTSerial.print("BT");
    Serial.print(temp);
   // BTSerial.print(temp);
    Serial.print("|BH");
   // BTSerial.print("|BH");
    Serial.print(hum);
   // BTSerial.print(hum);
    Serial.print("|BP");
   // BTSerial.print("|BP");
    Serial.print(pres);
  //  BTSerial.print(pres);

    Serial.print("|");
  //  BTSerial.print("|");
  }

}

void displaySensorDetails()
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
