#include <Wire.h>  // Wire library is used for I2C communication
#include "src\DS3231\DS3231.h" // https://github.com/NorthernWidget/DS3231
#include "src\SIM7020\SIMcore.h"
#include "SIM7020_httpGet.h"
#include <Sleep_n0m1.h>
#include <Battery.h>

/** Low pass filter RC constant for filtering acceleration values, time in
   microseconds.  Larger values will make the display look more stable, but it
   will take longer to display the true roll and pitch.
*/
#define LPF_ALPHA (500000.0)

/** Time interval in microseconds since the of the last loop() call. This is
   used to calculate accurate alpha values for low pass filters.  Note that
   this is a 16bit value, allowing for a maximum of approx 65 milliseconds
   between subsequent loop() calls.
*/
uint16_t delta_time;

/** Timestamp (value of micros()) when the last time loop() was called.  Used
   by `update_timer` to measure delta times and keep track of the beep time.
*/
uint32_t last_loop_start = 0;

int ADXL345 = 0x53; // The ADXL345 sensor I2C address
const byte pin_sleep = 9;
const byte pin_tx = 7;
const byte pin_rx = 8;
const byte pin_k = 6;
const byte pin_battery_activation = 5;
String timeStamp;
int BatLev;

// Max sleep time is 49.7 days
const unsigned long PROGMEM sleepTime = 480000; // Unit in microsecond // 8*60*1000 = 480000

SoftwareSerial mySerial(pin_rx, pin_tx);
SIMcore sim7020(&mySerial, pin_k, pin_sleep);
DS3231 Clock;
Sleep sleep;
Battery battery(3400, 4200, A0); 

//void offset_calibration(int addr) {
//    // Off-set Calibration
//    // X-axis
//    Wire.beginTransmission(addr);
//    Wire.write(0x1E);
//    Wire.write(round(6/4)); // Unit in LSB
//    Wire.endTransmission();
//    delay(10);
//    // Y-axis
//    Wire.beginTransmission(addr);
//    Wire.write(0x1F);
//    Wire.write(round(20/4)); // Unit in LSB
//    Wire.endTransmission();
//    delay(10);
//    // Z-axis
//    Wire.beginTransmission(addr);
//    Wire.write(0x20);
//    Wire.write(round(-14/4)); // Unit in LSB
//    Wire.endTransmission();
//    delay(10);
//}

void RTCread(String &str) {
  bool Century = false;
  char DateStr[14];
  bool h12;
  bool PM;
  byte year; byte month;  byte day;
  byte hour; byte minute; byte second;

  year = Clock.getYear();
  month = Clock.getMonth(Century);
  day = Clock.getDate();
  hour = Clock.getHour(h12, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();

  memset(DateStr, 0, 14);
  sprintf(DateStr, "20%02d%02d%02d%02d%02d%02d",
          year,
          month,
          day,
          hour,
          minute,
          second);
  str = DateStr;
}

void read_accelerometer(int addr, float output[3]) {
  // The filtered accelerometer values.  This is 'static', so it remembers
  // its values between `read_accelerometer' calls.
  static float filter[3];

  // Step 0: Begin a transmission to the I2C slave device with the given address
  Wire.beginTransmission(addr);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false); // If false, endTransmission() sends a restart message after transmission.
  Wire.requestFrom(addr, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  // Step 1: Read the raw values from the accelerometer
  output[0] = ( Wire.read() | Wire.read() << 8); // X-axis value
  output[1] = ( Wire.read() | Wire.read() << 8); // Y-axis value
  output[2] = ( Wire.read() | Wire.read() << 8); // Z-axis value

  // Step 2: Calculate the filter alpha value and update the filter.
  float alpha = float(delta_time) / (LPF_ALPHA + float(delta_time));

  filter[0] = filter[0] * (1 - alpha) + output[0] * alpha;
  filter[1] = filter[1] * (1 - alpha) + output[1] * alpha;
  filter[2] = filter[2] * (1 - alpha) + output[2] * alpha;

  // Step 3: Produce the final filtered values.
  output[0] = filter[0];
  output[1] = filter[1];
  output[2] = filter[2];
}

void calcu_roll_pitch(float input[3], float ang[2]) {
  // Roll
  ang[0]  = atan(input[1] / sqrt(pow(input[0], 2) + pow(input[2], 2))) * 180 / PI;
  // Pitch
  ang[1] = atan(-1 * input[0] / sqrt(pow(input[1], 2) + pow(input[2], 2))) * 180 / PI;
}

void update_timer() {
  uint32_t now = micros();

  // Calculate `delta_time`, taking care of roll overs which happen
  // approximately every 70 minutes
  if (now < last_loop_start) { // roll over
    delta_time = (0xFFFFFFFF - last_loop_start) + now;
  } else {
    delta_time = now - last_loop_start;
  }
  last_loop_start = now;
}


void setup() {
  pinMode(pin_battery_activation, OUTPUT);
  digitalWrite(pin_battery_activation, HIGH);
  Serial.begin(19200); // Initiate serial communication for printing the results on the Serial monitor
  while (!Serial) {
    yield(); // wait for serial port to connect. Needed for native USB port only
  }
  Wire.begin();// Initiate the Wire library

  // Enable battery sense
  senseBatteryLevel();

  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // Bit D3 High for measuring enable (8dec -> 0000 1000 binary)
  Wire.endTransmission();
  delay(10);

  // Accepted measurment values are 2g, 4g, 8g or 16g
  Wire.beginTransmission(ADXL345);
  Wire.write(0x31);
  // 2g : B00000000   // 4g : B00000001
  // 8g : B00000010   // 16g: B00000011
  Wire.write(B00000000);
  Wire.endTransmission();
  delay(10);
  //offset_calibration(ADXL345);
}
//, char roll_input, char pitch_input

void upload(String tStamp, float arg1, float arg2, int arg3) {  
  char pathBuffer[150];
  sprintf(pathBuffer,"/update_general.php?site=INCM01&time=%s&id=5002&roll=%s&pitch=%s&yaw=0&field1=%s&field2=0&field3=0",
          String(tStamp).c_str(), String(arg1).c_str(), String(arg2).c_str(), String(arg3).c_str());
  Serial.println(F("-----------The path is here!-----------------"));
  Serial.println(pathBuffer);  
  sim7020.init();
  delay(2000);
  sim7020.checkNetwork(100);
  if (!sim7020.isNetworkReady()) {
    sim7020.turnOFF();
    return;
  }
  sim7020.httpGet("http://icebergtek.ddns.net", pathBuffer);
  sim7020.turnOFF();
}

int senseBatteryLevel(){
    digitalWrite(pin_battery_activation, LOW);
    battery.begin(3300, 1.47, &sigmoidal);
    float value = 0;
    uint8_t sampling_num = 10;
    for (uint8_t i=0; i<sampling_num; i++) {
        float sample_val = battery.level();
        value += sample_val;
    }
    float avg = value/sampling_num;
    digitalWrite(pin_battery_activation, HIGH);
    return avg;
}

void loop() {
  update_timer();
  float acceleration[3];
  float angle_r_p[2];
  uint8_t ii;
  for (ii = 0; ii <= 200; ii++) {
    read_accelerometer(ADXL345, acceleration);
  }
  calcu_roll_pitch(acceleration, angle_r_p);
  Serial.print(F("Xa= "));
  Serial.print(acceleration[0]);
  Serial.print(F("   Ya= "));
  Serial.print(acceleration[1]);
  Serial.print(F("   Za= "));
  Serial.println(acceleration[2]);
  Serial.print(F("Roll= "));
  Serial.print(angle_r_p[0]);
  Serial.print(F("   Pitch= "));
  Serial.println(angle_r_p[1]);
  
  BatLev = senseBatteryLevel();
  Serial.print(F("Battery level: "));
  Serial.print(BatLev);
  Serial.println(F(" %"));
  
  RTCread(timeStamp);
  Serial.print(F("TimeStamp= "));
  Serial.println(timeStamp);
  
  upload(timeStamp, angle_r_p[0], angle_r_p[1], BatLev);  
  Serial.println(F("It is time to sleep."));
  delay(2000);  
  sleep.pwrDownMode(); // Set sleep mode 
  sleep.sleepDelay(sleepTime); // Sleep for: sleepTime // Uint in microsecond
}
