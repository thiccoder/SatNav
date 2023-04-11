#include <TinyGPS++.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, HMC5883L_ADDRESS);
static const uint32_t GPS_BAUD = 9600;
TinyGPSPlus gps;
double laat[10]={},lnng[10]={};
String buf = "";
bool ready =false;
int now=0;
int n=0;
float heading=0;

void forward(int power)
{
}
void backward(int power)
{
}
void right(int power)
{
}
void left(int power)
{
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }
  Serial.println("Initialize HMC5883");
  // compass.setRange(HMC5883L_RANGE_1_3GA);
  Serial.print("compass range is:");
  Serial.println(compass.getRange());
  // compass.setMeasurementMode(HMC5883L_CONTINOUS);
  Serial.print("compass measurement mode is:");
  Serial.println(compass.getMeasurementMode());
  // compass.setDataRate(HMC5883L_DATARATE_15HZ);
  Serial.print("compass data rate is:");
  Serial.println(compass.getDataRate());
  // compass.setSamples(HMC5883L_SAMPLES_8);
  Serial.print("compass samples is:");
  Serial.println(compass.getSamples()); 
  float declinationAngle = (9.0 + (28.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  while (true) if (Serial.available())
  {
    char c = Serial.read();
    if ((c == '\n') && (buf != "")) 
    {
      break;
    }
    else 
    {
      buf.concat(c);
    }
  }
  String s = ""; 
  for (int i = 1;buf[i] != '$';i++)
  {
    s.concat(buf[i]);
  }
  n = s.toInt();
  int idx = s.length() + 2;
  for (int i = 0; i<n;i++)
  {
    s = "";
    while (buf[idx] != '$') s.concat(buf[idx++]);
    s.replace(',','.');
    laat[i] = s.substring(0,s.indexOf('&')).toDouble();
    lnng[i] = s.substring(s.indexOf('&')+1).toDouble();
    Serial.print("point(lat = ");
    Serial.print(laat[i], 16);
    Serial.print(",      lng = ");
    Serial.print(lnng[i], 16);
    Serial.println(")");
    idx++;
  }
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
        heading = mag.HeadingDegress;
        double course = gps.courseTo(gps.location.lat(),gps.location.lng(),laat[now],lnng[now]);
        if(course+20<heading)
        {
          left(90);
        }
            else if(course-20>heading)
        {
          right(90);
        }
        else
        {
          forward(0);
          if (gps.distanceBetween(gps.location.lat(),gps.location.lng(),laat[now],lnng[now])>17)
          {
            forward(180);
          }
          else
          {
            forward(0);
            delay(1000);
            Serial.print(laat[now], 8);
            Serial.print(";");
            Serial.println(lnng[now], 8);
            delay(1000);
            now++;
            if(n==now)
            {
              forward(0);
              delay(999999);
            }
          }
        }
      }
    }
  }
}