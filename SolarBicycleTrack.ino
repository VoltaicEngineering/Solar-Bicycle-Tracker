#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

//Pin connection between Fona and Arduino
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

//Communication between FONA and Arduino using SoftwareSerial library
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Watchdog Interrupt
ISR(WDT_vect)
{
  wdt_disable();  // disable watchdog
}

//Address name and location variables
String yourThing = "plv6465";//*************Change Address Name*************//
float GSMLat = 0.0, GSMLong = 0.0;
float GPSLat = 0.0, GPSLong = 0.0;
bool GPSSuccess = 0, GSMSuccess = 0;

void setup()
{
  pinMode(9, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  //De-Activate FONA
  digitalWrite(9, LOW);
  delay(1000);
  digitalWrite(9, HIGH);
  delay(2000);
  digitalWrite(9, LOW);
  delay(3000);
  pinMode(9, INPUT);

  //Disable peripherals
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();

  //Put Arduino in deep sleep for 30min
  for (int i = 0; i < 188; i++) { //1/2hour sleep
    myWatchdogEnable (0b100001);
  }

  //Wake-Up and re-enable the peripherals
  power_all_enable();

  //Activate FONA
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  delay(2000);

  //Enable Adafruit Fona and GPS/GSM network
  Serial.println(F("Adafruit FONA 808 & 3G GPS demo"));
  Serial.println(F("Initializing FONA... (May take a few seconds)"));
  fonaSerial->begin(4800);

  //Establish communication btw Arduino and Fona
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
  }
  else {
    Serial.println(F("FONA is OK"));
    // Try to enable GPRS
    Serial.println(F("Enabling GPS..."));
    fona.enableGPS(true);

    //Acquire a location via GPS/GSM. Timeout in 1 minute if network is down
    while ( (GSMSuccess == 0 or GPSSuccess == 0) and millis() < 66000 ) {
      delay(2000);
      Serial.println(millis());
      float latitude, longitude, speed_kph, heading, speed_mph, altitude;

      // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
      boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

      // Acquire location via GPS network
      if (gps_success) {
        GPSSuccess = 1;
        Serial.print("GPS lat:");
        Serial.println(latitude, 6);
        Serial.print("GPS long:");
        Serial.println(longitude, 6);
        GPSLat = latitude;
        GPSLong = longitude;

      } else {
        Serial.println("GPS Failed");
      }

      // Acquire location via GSM network
      if (fona.getNetworkStatus() == 1) {
        boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);
        if (gsmloc_success) {
          GSMSuccess = 1;
          Serial.print("GSMLoc lat:");
          Serial.println(latitude, 6);
          Serial.print("GSMLoc long:");
          Serial.println(longitude, 6);
          GSMLat = latitude;
          GSMLong = longitude;
        }
        else {
          Serial.println("GSM location failed");
          fona.enableGPRS(false);
          fona.enableGPRS(true);
        }
      }

    }
    GPSSuccess = 0;
    GSMSuccess = 0;

    //Read the Lippo battery % charge
    wdt_enable(WDTO_8S);
    uint16_t vbat;
    if (! fona.getBattPercent(&vbat)) {
      Serial.println(F("Failed to read Batt"));
    } else {
      Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
    }

    //Read and process the current time
    char gtime[23];
    fona.getTime(gtime, 23);
    Serial.print(F("Time = ")); Serial.println(gtime);
    String gsmtime, gsmdate;
    for (int i = 10; i < 18; i++) {
      gsmtime += gtime[i];
      gsmdate += gtime[i - 9];
    }
    wdt_reset();

    //Send GSM/GPS/VBatt location to the server
    wdt_enable(WDTO_8S);
    uint16_t statuscode;
    int16_t length;
    String url = "http://dweet.io/dweet/for/";
    url += yourThing;
    url += "?gsmlatitude=";
    url += String(GSMLat, 6);
    url += "&gsmlongitude=";
    url += String(GSMLong, 6);
    url += "&gpslatitude=";
    url += String(GPSLat, 6);
    url += "&gpslongitude=";
    url += String(GPSLong, 6);
    url += "&batt=";
    url += String(vbat);
    url += "&da=";
    url += gsmdate;
    url += "&ti=";
    url += gsmtime;
    url += "&t=";
    url += String(1);
    char buf[80];
    url.toCharArray(buf, url.length());
    Serial.print("Request: ");
    Serial.println(buf);

    // Send location to Dweet.io
    if (!fona.HTTP_GET_start(buf, &statuscode, (uint16_t *)&length)) {
      Serial.println("Failed!");
    }
    else {
      while (length > 0) {
        while (fona.available()) {
          char c = fona.read();
          Serial.write(c);
          length--;
        }
      }
    }
  }
  wdt_reset();

  //Reset the Arduino
  software_Reset() ;
}

//Function: Arduino will go in deep sleep mode when this function is called
void myWatchdogEnable(const byte interval)
{
  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  wdt_reset();
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  sleep_mode();            // now goes to Sleep and waits for the interrupt
}

//Function: Arduino will restarts the program from beginning
void software_Reset() 
{
  asm volatile ("  jmp 0");
}


