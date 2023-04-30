#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Adafruit_BMP280 bmp;
SoftwareSerial GPS_SoftSerial(4, 3);
TinyGPSPlus gps;			

volatile float minutes, seconds;
volatile int degree, secs, mins;

void setup() {
  Serial.begin(115200); // Start serial communication
  if (!bmp.begin(0x76)) { // Check if BMP280 is connected
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Set sampling mode and sampling rates
                  Adafruit_BMP280::SAMPLING_X2,     // (higher rates give better results but take more time)
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  GPS_SoftSerial.begin(115200);
  }

void loop() {
  float temperature = bmp.readTemperature(); // Read temperature
  float pressure = bmp.readPressure() / 100.0F; // Read pressure and convert to hPa
  float altitude = bmp.readAltitude(1013.25); // Calculate altitude relative to sea level (pressure at sea level is assumed to be 1013.25 hPa)

  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  delay(1000); // Delay for one second before reading again

  smartDelay(1000);	/* Generate precise delay of 1ms */
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        uint8_t hr_val, min_val, sec_val;
        bool loc_valid, alt_valid, time_valid;
        lat_val = gps.location.lat();	/* Get latitude data */
        loc_valid = gps.location.isValid();	/* Check if valid location data is available */
        lng_val = gps.location.lng();	/* Get longtitude data */
        alt_m_val = gps.altitude.meters();	/* Get altitude data in meters */
        alt_valid = gps.altitude.isValid();	/* Check if valid altitude data is available */
        hr_val = gps.time.hour();	/* Get hour */
        min_val = gps.time.minute(); 	/* Get minutes */
        sec_val = gps.time.second();	/* Get seconds */
        time_valid = gps.time.isValid();	/* Check if valid time data is available */
        if (!loc_valid)
        {          
          Serial.print("Latitude : ");
          Serial.println("***");
          Serial.print("Longitude : ");
          Serial.println("***");
        }
        else
        {
          DegMinSec(lat_val);
          Serial.print("Latitude in Decimal Degrees : ");
          Serial.println(lat_val, 6);
          Serial.print("Latitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
          DegMinSec(lng_val);	/* Convert the decimal degree value into degrees minutes seconds form */
          Serial.print("Longitude in Decimal Degrees : ");
          Serial.println(lng_val, 6);
          Serial.print("Longitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
        }
        if (!alt_valid)
        {
          Serial.print("Altitude : ");
          Serial.println("***");
        }
        else
        {
          Serial.print("Altitude : ");
          Serial.println(alt_m_val, 6);    
        }
        if (!time_valid)
        {
          Serial.print("Time : ");
          Serial.println("***");
        }
        else
        {
          char time_string[32];
          sprintf(time_string, "Time : %02d/%02d/%02d \n", hr_val, min_val, sec_val);
          Serial.print(time_string);    
        }
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())	/* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)		/* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}