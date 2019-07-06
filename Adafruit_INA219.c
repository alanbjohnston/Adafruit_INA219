/*!
 * @file Adafruit_INA219.cpp
 *
 * @mainpage Adafruit INA219 current/power monitor IC
 *
 * @section intro_sec Introduction
 *
 *  Driver for the INA219 current sensor
 *
 *  This is a library for the Adafruit INA219 breakout
 *  ----> https://www.adafruit.com/products/904
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 * Converted to C for Raspberry Pi by Alan Johnston, KU2Y
 *
 */

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <time.h>
#include "Adafruit_INA219.h"

/*!
 *  @brief  Sends a single command byte over I2C
 *  @param  reg
 *          register address
 *  @param  value
 *          value to write
 */

/*  Use wiringPiI2CWriteReg16(x_fd, INA219_REG_CALIBRATION, x_calValue); */
void wireWriteRegister(int fd, uint8_t reg, uint16_t value) {
//  _i2c->beginTransmission(ina219_i2caddr);
//  _i2c->write(reg);                 // Register
//  _i2c->write((value >> 8) & 0xFF); // Upper 8-bits
//  _i2c->write(value & 0xFF);        // Lower 8-bits
//  _i2c->endTransmission();
//  delay();
  int x_fd;
  wiringPiI2CWriteReg16(x_fd, reg, value);
}


/*!
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *          register address
 *  @param  *value
 *          read value
 *
 * use instead wiringPiI2CReadReg16(x_fd, INA219_REG_CURRENT) */
uint16_t wireReadRegister(int fd, uint8_t reg) {
  int16_t value;
  
 // _i2c->beginTransmission(ina219_i2caddr);
 // _i2c->write(reg); // Register
 // _i2c->endTransmission();

//  delay(1); // Max 12-bit conversion time is 586us per sample
  delay(1);
 // _i2c->requestFrom(ina219_i2caddr, (uint8_t)2);
  // Shift values to create properly formed integer
 // *value = ((_i2c->read() << 8) | _i2c->read());
  value = ((wiringPiI2CRead(fd) << 8 ) | wiringPiI2CRead (fd)));
  return value;
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void setCalibration_32V_2A(int fd) {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 2.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.000061              (61uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,000488              (488uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0001 (100uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)

  ina219_calValue = 4096;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.002 (2mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  wiringPiI2CWriteReg16(fd, INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wiringPiI2CWriteReg16(fd, INA219_REG_CONFIG, config);
}

/*!
 *  @brief  Set power save mode according to parameters
 *  @param  on
 *          boolean value
 */
void powerSave(int fd, int on) {
  uint16_t current;
  //wireReadRegister(INA219_REG_CONFIG, &current);
  current = wiringPiI2CReadReg16(fd, INA219_REG_CONFIG);
  uint8_t next;
  if (on) {
    next = current | INA219_CONFIG_MODE_POWERDOWN; 
  } else {
    next = current & ~INA219_CONFIG_MODE_POWERDOWN; 
  }
  wiringPiI2CWriteReg16(fd, INA219_REG_CONFIG, next);
}


/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 1A
 *          of current.  Each unit of current corresponds to 40uA, and each
 *          unit of power corresponds to 800�W. Counter overflow occurs at
 *          1.3A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void setCalibration_32V_1A(int fd) {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1			(Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 1.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000305             (30.5�A per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.000244              (244�A per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0000400 (40�A per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 10240 (0x2800)

  ina219_calValue = 10240;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0008 (800�W per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.31068A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // ... In this case, we're good though since Max_Current is less than
  // MaxPossible_I
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.131068V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.31068 * 32V
  // MaximumPower = 41.94176W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  wiringPiI2CWriteReg16(fd, INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wiringPiI2CWriteReg16(fd, INA219_REG_CONFIG, config);
}

/*!
 *  @brief set device to alibration which uses the highest precision for
 *     current measurement (0.1mA), at the expense of
 *     only supporting 16V at 400mA max.
 */
void setCalibration_16V_400mA(int fd) {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  ina219_calValue = 8192;

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  wiringPiI2CWriteReg16(fd, INA219_REG_CALIBRATION, ina219_calValue);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wiringPiI2CWriteReg16(fd, INA219_REG_CONFIG, config);
}

/*!
 *  @brief  Instantiates a new INA219 class
 *  @param addr the I2C address the device can be found on. Default is 0x40
 *
void Adafruit_INA219(uint8_t addr) {
  ina219_i2caddr = addr;
  ina219_currentDivider_mA = 0;
  ina219_powerMultiplier_mW = 0.0f;
}
*/

/*!
 *  @brief  Setups the HW (defaults to 32V and 2A for calibration values)
 *  @param theWire the TwoWire object to use
 *
void begin(TwoWire *theWire) {
  _i2c = theWire;
  init();
}
*/

/*!
 *  @brief  begin I2C and set up the hardware
 */
void init(int fd) {
//  
//   Add initialization of I2C using WiringPi
//
//  _i2c->begin();
  // Set chip to large range config values to start
  setCalibration_32V_2A(fd);
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t getBusVoltage_raw(int fd) {
  uint16_t value;
  //wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
  value = wiringPiI2CReadReg16(fd, INA219_REG_BUSVOLTAGE);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((value >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t getShuntVoltage_raw(int fd) {
  uint16_t value;
  //wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  value = wiringPiI2CReadReg16(fd, INA219_REG_SHUNTVOLTAGE);
  return (int16_t)value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t getCurrent_raw(int fd) {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wiringPiI2CWriteReg16(fd, INA219_REG_CALIBRATION, ina219_calValue);

  // Now we can safely read the CURRENT register!
  //wireReadRegister(INA219_REG_CURRENT, &value);
  value = wiringPiI2CReadReg16(fd, INA219_REG_CURRENT);

  return (int16_t)value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t getPower_raw(int fd) {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  wiringPiI2CWriteReg16(fd, INA219_REG_CALIBRATION, ina219_calValue);

  // Now we can safely read the POWER register!
  //wireReadRegister(INA219_REG_POWER, &value);
  value = wiringPiI2CReadReg16(fd, INA219_REG_POWER);

  return (int16_t)value;
}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float getShuntVoltage_mV(int fd) {
  int16_t value;
  value = getShuntVoltage_raw(fd);
  return value * 0.01;
}

/*!
 *  @brief  Gets the shunt voltage in volts
 *  @return the bus voltage converted to volts
 */
float getBusVoltage_V(int fd) {
  int16_t value = getBusVoltage_raw(fd);
  return value * 0.001;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float getCurrent_mA(int fd) {
  float valueDec = getCurrent_raw(fd);
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float getPower_mW(int fd) {
  float valueDec = getPower_raw(fd);
  valueDec *= ina219_powerMultiplier_mW;
  return valueDec;
}
