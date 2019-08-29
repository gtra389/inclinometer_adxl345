#include "src\ADXL345\ADXL345.h"
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
#include <EEPROM.h>

/** Amount of time, in microseconds, needed to hold a button (reset or
 * calibrate) in order to perform the action.  This prevents starting
 * calibration or resetting the max values accidentally.
 */
#define HOLD_INTERVAL (1000000)

/** Low pass filter RC constant for filtering acceleration values, time in
 * microseconds.  Larger values will make the display look more stable, but it
 * will take longer to display the true roll and pitch.
 */
#define LPF_ALPHA (500000.0)

// Calibration parameters.  These will need to be updated for each unit, see
// the Calibration section in README.md

#define X_SLOPE 0.003984064
#define X_INTERCEPT -0.047808765
#define Y_SLOPE 0.003937008
#define Y_INTERCEPT 0.07480315
#define Z_SLOPE 0.003944773
#define Z_INTERCEPT 0.065088757

#define CALIBRATE_PIN 12

/** Place (address) in EEPROM where we store the calibration data
 */
#define EEPROM_CALIBRATION_ADDRESS (0)

/** State values for the accelerometer application.  See main loop() function.
 */
#define STATE_ACQUIRE_DOWN_DIRECTION 0
#define STATE_ACQUIRE_FORWARD_DIRECTION 1
#define STATE_RUNNING 2

#define STATE_MASK 0x0F
uint8_t status_flags = 0;
#define SET_STATE(s) do { status_flags = (status_flags & ~STATE_MASK) | s; } while(0)

/** Timestamp (value of micros()) when the last time loop() was called.  Used
 * by `update_timer` to measure delta times and keep track of the beep time.
 */
uint32_t last_loop_start = 0;

/** Time interval in microseconds since the of the last loop() call. This is
 * used to calculate accurate alpha values for low pass filters.  Note that
 * this is a 16bit value, allowing for a maximum of approx 65 milliseconds
 * between subsequent loop() calls.
 */
uint16_t delta_time;

/** Accumulated time the calibrate button is held down for.
 */
int32_t calibrate_hold_time = 0;

/** X, Y and Z axis for the installed accelerometer position.  These are used
 * to determine the roll and pitch regardless of the position in which the
 * unit is installed.  They are set during the calibration process, see
 * on_acquire_down_direction() and on_acquire_forward_direction()
 *
 * This data is also saved to EEPROM and restored when the unit starts up.
 */
struct orientation_t {
    float xaxis[3];
    float yaxis[3];
    float zaxis[3];
} orientation;

/** The ADXL345 accelerometer access class
 */
//Adafruit_ADXL345_Unified adxl345;
int16_t x,y,z;

float vlen_squared(float vec[3])
{
    return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

float vlen(float vec[3])
{
    return sqrt(vlen_squared(vec));
}

void vzero(float vec[3])
{
    vec[0] = vec[1] = vec[2] = 0.0;
}

float* vnormalize(float vec[3], float vec_out[3])
{
    float m = vlen(vec);
    vec_out[0] = vec[0] / m;
    vec_out[1] = vec[1] / m;
    vec_out[2] = vec[2] / m;
    return vec_out;
}

float vdot(float vec_a[3], float vec_b[3])
{
    return vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1] + vec_a[2] * vec_b[2];
}

float* vcross(float vec_a[3], float vec_b[3], float vec_out[3])
{
    float x = vec_a[1] * vec_b[2] - vec_a[2] * vec_b[1];
    float y = vec_a[2] * vec_b[0] - vec_a[0] * vec_b[2];
    float z = vec_a[0] * vec_b[1] - vec_a[1] * vec_b[0];
    vec_out[0] = x;
    vec_out[1] = y;
    vec_out[2] = z;
    return vec_out;
}

const uint16_t crc_table[16] PROGMEM = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

uint16_t crc_table_ref(int index)
{
    // NOTE: crc_table + index is different from crc_table[index] since
    // crc_table is in PROGMEM!
    return pgm_read_word_near(crc_table + index);
}

/** Calculate a 16 bit CRC checksum of the 'data' buffer containing 'len'
 * bytes.
 */
uint16_t calculate_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    while (len-- > 0) {
        uint8_t byte = *data++;
        uint16_t tmp = crc_table_ref(crc & 0x0F);
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ crc_table_ref(byte & 0x0F);
        tmp = crc_table_ref(crc & 0x0F);
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ crc_table_ref((byte >> 4) & 0x0F);
    }
    return crc;
}

/** Save the calibration values to EEPROM along with a CRC checksum.  Next
 * time the unit boots up it will reuse these values, avoiding the need for a
 * calibration.
 */
void save_calibration_to_eeprom()
{
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), crc);
}

/** Read calibration data from EEPROM, if the CRC of the data is valid, go
 * straight to running state, otherwise go to calibration mode.
 */
void restore_calibration_from_eeprom()
{
    uint16_t stored_crc;
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), stored_crc);
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));

    if (crc == stored_crc) {
        SET_STATE(STATE_RUNNING);
    } else {
        // reset the Z-Axis so it is acquired again
        vzero(orientation.zaxis);
        SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
    }
}

/** Convert 'angle' from radians to degrees.
 */
float rad2deg(float angle)
{
    return 180.0 * (angle / 3.1415926);
}

/** Read the accelerometer X, Y and Z values and store them in 'output'.  The
 * output values are calibrated and filtered.
 */
void read_accelerometer(float output[3])
{
    // The filtered accelerometer values.  This is 'static', so it remembers
    // its values between `read_accelerometer' calls.
    static float filter[3];

    // Step 1: read the raw values from the accelerometer
    adxl.readAccel(&x, &y, &z); 
    output[0] = x;
    output[1] = y;
    output[2] = z;

    // Step 2: calibrate the values, see the readme.md file
    output[0] = output[0] * X_SLOPE + X_INTERCEPT;
    output[1] = output[1] * Y_SLOPE + Y_INTERCEPT;
    output[2] = output[2] * Z_SLOPE + Z_INTERCEPT;

    // Step 3: calculate the filter alpha value and update the filter.
    float alpha = float(delta_time) / (LPF_ALPHA + float(delta_time));

    filter[0] = output[0] * (1 - alpha) + output[0] * alpha;
    filter[1] = output[1] * (1 - alpha) + output[1] * alpha;
    filter[2] = output[2] * (1 - alpha) + output[2] * alpha;

    // Step 4: produce the final calibrated and filtered values.
    output[0] = filter[0];
    output[1] = filter[1];
    output[2] = filter[2];
}

/** Return the pitch angle in degrees (forward - backward inclination) of the
 * vehicle based on the current "down" direction stored in 'cal'.  'cal' is
 * converted to local coordinates (accounts for the installation orientation
 * of the inclinometer itself).
 *
 * A positive pitch angle indicates that the vehicle is pointing up, a
 * negative angle indicates that the vehicle is pointing down.
 */
float calculate_pitch(float cal[3])
{
    float down[3] = {0, 0, 1};
    float pitch_dir[3] = { cal[0], 0, cal[2] };
    vnormalize(pitch_dir, pitch_dir);
    float pitch = vdot(down, pitch_dir);
    float angle = rad2deg(acos(pitch));
    if (pitch_dir[0] > 0)
        return -angle;                   // up
    else
        return +angle;                  // down
}

/** Return the roll angle in degrees (left - right inclination) of the vehicle
 * based on the current "down" direction stored in 'cal'.  'cal' is converted
 * to local coordinates (accounts for the installation orientation of the
 * inclinometer itself).
 *
 * A positive roll angle indicates that the vehicle is rolling to the right, a
 * negative angle indicates that it is rolling to the left.
 */
float calculate_roll(float cal[3])
{
    float down[3] = {0, 0, 1};
    float roll_dir[3] = { 0, cal[1], cal[2] };
    vnormalize(roll_dir, roll_dir);
    float roll = vdot(down, roll_dir);
    float angle = rad2deg(acos(roll));
    if (roll_dir[1] > 0)
        return angle;                  // right
    else
        return -angle;                   // left
}
// ................................................... main application ....

/** Update blink and buzzer timers and sound the buzzer if the warn conditions
 * are set.  This function should be the first one called inside `loop()`.
 */
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

/** Check the status of the calibration button.  A long press will cause the
 *  unit to switch to calibration mode.
 */
void handle_calibrate_button() {
    if (digitalRead(CALIBRATE_PIN) == 0) {        
        SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
        // Reset the Z-Axis so it is acquired again
        vzero(orientation.zaxis);

        calibrate_hold_time = 0;        
    } else {
        calibrate_hold_time += delta_time;
    }
    if (calibrate_hold_time > HOLD_INTERVAL) {
        calibrate_hold_time = 0;
    }
}

/** Determine which way is down.  We assume that the vehicle is on level
 * ground.
 *
 * We wait for the accelerometer to stabilize than the reading cal[] reading
 * becomes the 'zaxis'.
 */
void on_acquire_down_direction(float cal[3]){
    float dot = vdot(cal, orientation.zaxis);
    if (dot > 0.95 && dot < 1.05) {
        vnormalize(orientation.zaxis, orientation.zaxis);
        
        vzero(orientation.xaxis);
        orientation.xaxis[0] = 1;
        orientation.xaxis[1] = 0;
        vzero(orientation.yaxis);
        orientation.yaxis[0] = 0;
        orientation.yaxis[1] = 1;
        
        save_calibration_to_eeprom();
        SET_STATE(STATE_RUNNING);
//        Serial.print("------Look at here ! (A)------");
//        Serial.print("\n");
    }
    else {
        orientation.zaxis[0] = (orientation.zaxis[0] + cal[0]) * 0.5;
        orientation.zaxis[1] = (orientation.zaxis[1] + cal[1]) * 0.5;
        orientation.zaxis[2] = (orientation.zaxis[2] + cal[2]) * 0.5;
//        Serial.print("------Look at here ! (B)------");
//        Serial.print("\n");
    }
}

/** Display vehicle roll and pitch based on the current accelerometer reading
 * in 'cal'.
 */
void on_running(float cal[3])
{
    // 'cal' is in world coordinates, transform it to local coordinates, to
    // calculate the calibrated roll and pitch.  Note that the vdot() calls
    // together make a matrix -- vector multiplication.
    float ncal[3];
    ncal[0] = vdot(orientation.xaxis, cal);
    ncal[1] = vdot(orientation.yaxis, cal);
    ncal[2] = vdot(orientation.zaxis, cal);

    float pitch = calculate_pitch(ncal);
    float roll = calculate_roll(ncal);
    float gforce = vlen(cal); // the magnitude of 3-D accerlation vetors    
    
    //display_pitch_roll(pitch, roll, gforce);
       
    Serial.print(F("roll ="));
    Serial.print(roll);    
    Serial.print(F("; pitch ="));
    Serial.print(pitch);
    Serial.print(F("; gforce ="));
    Serial.print(gforce);
    Serial.print(F("\n"));    
}

void setup(){
  Serial.begin(19200);
  while (!Serial) {
      yield(); // wait for serial port to connect. Needed for native USB port only
  }   
    // To enable the internal pullup resistors
    pinMode(CALIBRATE_PIN, INPUT_PULLUP); 
     
    // Power on the ADXL345
    adxl.powerOn(); 
    // Give the range settings
    // Accepted values are 2g, 4g, 8g or 16g
    adxl.setRangeSetting(2);     

    last_loop_start = micros();
    restore_calibration_from_eeprom();
}

void loop() {
    update_timer();
    float acceleration[3];
    read_accelerometer(acceleration);

switch (status_flags & STATE_MASK) {
    case STATE_ACQUIRE_DOWN_DIRECTION:
        on_acquire_down_direction(acceleration);
        Serial.print(F("STATE_ACQUIRE_DOWN_DIRECTION"));
        Serial.print(F("\n"));
        break; 
    case STATE_RUNNING:
        Serial.print(F("--------------------"));
        Serial.print(F("\n"));
        on_running(acceleration);
        Serial.print(F("STATE_RUNNING"));
        Serial.print(F("\n"));
        
    break;
    }
     handle_calibrate_button();
//     Serial.print("calibrate_hold_time = ");
//     Serial.print(calibrate_hold_time);
//     Serial.print("\n");     
//     Serial.print("orientation.xaxis = [");
//     Serial.print(orientation.xaxis[0]);
//     Serial.print(",");
//     Serial.print(orientation.xaxis[1]);
//     Serial.print(",");
//     Serial.print(orientation.xaxis[2]);
//     Serial.print("]");
//     Serial.print("\n");
//     Serial.print("orientation.yaxis = [");
//     Serial.print(orientation.yaxis[0]);
//     Serial.print(",");
//     Serial.print(orientation.yaxis[1]);
//     Serial.print(",");
//     Serial.print(orientation.yaxis[2]);
//     Serial.print("]");
//     Serial.print("\n");
//     Serial.print("orientation.zaxis = [");
//     Serial.print(orientation.zaxis[0]);
//     Serial.print(",");
//     Serial.print(orientation.zaxis[1]);
//     Serial.print(",");
//     Serial.print(orientation.zaxis[2]);
//     Serial.print("]");
//     Serial.print("\n");
   
   delay(2000);
}
