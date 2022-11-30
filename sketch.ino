// Custom chips playground
// See https://link.wokwi.com/custom-chips-alpha for more info


// ======================= Copied Definitions ====================
// Copied some adafruit servo driver code rather than import the library - will do that later
// This explains the defines below etc...

#include <Wire.h>

// #define ENABLE_DEBUG_OUTPUT 

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */


#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

// ======================= End of Copied Definitions ====================


// chip address. must match the wiring in diagram.json
const uint8_t _i2caddr = 0x1F;


static void begin(uint8_t prescale=0);
static void reset();
static void restart();
static void setPWMFreq(float f);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // initialise the PCA9685 chip
  begin();

  setPWM(_i2caddr, 0, 1500, 2000);

  delay(5000);

  reset();

  delay(1000);


  setPWM(_i2caddr, 0, 1000, 3000);
  delay(5000);
  
  restart();
  // setPWM(_i2caddr, 1, 4000, 3000);


}

void loop() {
  // static uint16_t ndx;

  // for (uint16_t i = 0; i < 8; i++) {
  //   // Serial.print(ndx);Serial.print(" spwm ");Serial.print(i); Serial.print(" on:"); Serial.print(i * 256); Serial.print(" off:");Serial.println((i+ 1 + (ndx %10))*256);
  //   setPWM(_i2caddr, i, i * 256, (i+ 1 + (ndx & 7))*256);
  // }
  // ndx++;
  delay(500);
}

void begin(uint8_t prescale=0) {
  restart();
  // set a default frequency
  setPWMFreq(1);
}

/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
void restart() {
  write8(PCA9685_MODE1, MODE1_RESTART);
  delay(10);
}


void setPWMFreq(float freq) {

  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

// #ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: ");
  Serial.println(prescale);
// #endif

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode);                             // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}


void setPWM(int addr, int chan,  uint16_t on, uint16_t off ) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(chan);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif

  Wire.beginTransmission(addr);
  Wire.write(6 + 4 * chan);
  Wire.write(on);
  Wire.write(on >> 8);
  Wire.write(off);
  Wire.write(off >> 8);
  Wire.endTransmission();

}


uint8_t read8(uint8_t addr) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return Wire.read();
}

void write8(uint8_t addr, uint8_t d) {
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
}

void reset() {
  Wire.beginTransmission(0);
  Wire.write(6);
  Wire.endTransmission();
}

