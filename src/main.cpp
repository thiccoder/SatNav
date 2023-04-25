#include <TinyGPS++.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, HMC5883L_ADDRESS);
static const uint32_t GPS_BAUD = 9600;
TinyGPSPlus gps;
double laat[2] = {0, 50.595409}, lnng[2] = {0, 36.587220};
int current = 0;
int count = 2;
void moveTank(int power, int dir)
{
  Serial.print("Move with power ");
  Serial.print(power);
  Serial.print("and dir ");
  Serial.println(dir);
}
void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);
  while (!compass.begin())
  {
    Serial.println("Could not find compass!");
    delay(500);
  }
  Serial.println("Initialize compass");
  float declinationAngle = (9.0 + (28.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
}

void loop()
{
  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      if (gps.location.isValid())
      {
        delay(10);
        sVector_t mag = compass.readRaw();
        compass.getHeadingDegrees();
        double heading = mag.HeadingDegress;
        double course = gps.courseTo(gps.location.lat(), gps.location.lng(), laat[current], lnng[current]);
        double dir = heading - course;
        double distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), laat[current], lnng[current]);
        if (distance > 20)
        {
          moveTank(200, dir);
        }
        else
        {
          moveTank(0, 0);
          delay(1000);
          Serial.print(laat[current], 8);
          Serial.print(";");
          Serial.println(lnng[current], 8);
          delay(1000);
          current++;
          if (count == current)
          {
            moveTank(0, 0);
            delay(999999);
          }
        }
      }
    }
  }
}
/*
#include <TinyGPSPlus.h>
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
void displayInfo();
void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}*/