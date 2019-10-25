#include <DS3232RTC.h>
#include <RTClib.h>
RTC_DS3231 rtc;

/*
 *  Wire connect
 *  DS3231 --> UNO
 *  SCL --> A5
 *  SDA --> A4
 *  VCC --> 5V
 *  GND --> GND
 */

int countNum = 0;
void setup() {
  // Start the serial interface
  //
  Serial.begin(115200); // open serial connection to print out distance values
  while (!Serial) {
        yield(); // wait for serial port to connect. Needed for native USB port only
  }   
  rtc.begin();             // initialize the I2C bus here.
}


void loop() {
  while (countNum == 0) {  
    tmElements_t tm;
    tm.Hour      = 17;             //set the tm structure to 23h31m30s on 13Feb2009
    tm.Minute    = 42;
    tm.Second    = 30;
    tm.Day       = 28;
    tm.Month     = 9;
    tm.Year      = 2019 - 1970;    //tmElements_t.Year is the offset from 1970
    RTC.write(tm);                 //set the RTC from the tm structure
    countNum ++;    
  }
  
  DateTime present = rtc.now();
  Serial.println(F("Current Time: "));  
  Serial.println(printDateTime_v2(present));
  delay(1000);
} 
  

String printDateTime_v2(DateTime t) {
  String output = String(t.year()) +\
                  String(((t.month() < 10) ? "0" : "")) +\
                  String(((t.month()))) +\
                  String(((t.day() < 10) ? "0" : "")) +\
                  String(((t.day()))) +\
                  String(((t.hour() < 10) ? "0" : "")) +\
                  String(((t.hour()))) +\
                  String(((t.minute() < 10) ? "0" : "")) +\
                  String(((t.minute()))) +\
                  String(((t.second() < 10) ? "0" : "")) +\
                  String(((t.second())));
  return output;      
}
