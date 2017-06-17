#include <Wire.h>
#include <AS3935.h>

/*
 * a line-by-line port of https://github.com/pcfens/particle-as3935/
 * an exercise for me to write a library.
 */

AS3935::AS3935(uint8_t i2c_address, uint8_t int_pin)
{
  _i2c_address = i2c_address;
  _int_pin = int_pin;
}

AS3935::AS3935::~AS3935()
{
}

/**
 * Begin using the object with default SDA, and SCL pin numbers.
 */
void AS3935::begin()
{
    begin((int) _default_sda, (int) _default_scl);
}

/**
 * Begin using the object
 * - Begin wire
 * - Enable interrupt pin as INPUT
 * - Disable Oscillators on interrupt pin.
 */
void AS3935::begin(int sda, int scl)
{
    Wire.begin(sda, scl);
    pinMode(_int_pin, INPUT);
    disableOscillators();
}

/**
 * Find the shift required to make the mask use the LSB.
 * @param mask The mask to find the shift of
 * @return The number of bit positions to shift the mask
 */
uint8_t AS3935::_getShift(uint8_t mask)
{
    uint8_t i = 0;
    for (i = 0; ~mask & 1; i++)
        mask >>= 1;
    return i;
}

/**
 * Read a byte from a register.
 * @param reg The register address
 * @return The value in the register
 */
uint8_t AS3935::readRegister(uint8_t reg)
{
    uint8_t v;
    Wire.beginTransmission(_i2c_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)_i2c_address, 1);
    v = Wire.read();
    return v;
}

/**
 * Read a byte from a register, return a masked and shifted value
 * @param reg The register address
 * @param mask The mask to use when shifting contents
 * @return An uint8_t with the right most bits containing the masked and
 * shifted contents of the requested register
 */
uint8_t AS3935::readRegisterWithMask(uint8_t reg, uint8_t mask)
{
    uint8_t v;
    v = readRegister(reg) & mask;
    return (v >> _getShift(mask));
}

/**
 * Write a masked value to register reg, preserving other bits
 * @param reg The register address
 * @param mask The bitmask to mask
 * @param value The value to write to the register
 */
void AS3935::writeRegisterWithMask(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t registerValue;
    registerValue = readRegister(reg);
    registerValue &= ~(mask);
    registerValue |= ((value << (_getShift(mask))) & mask);
    Wire.beginTransmission(_i2c_address);
    Wire.write(reg);
    Wire.write(registerValue);
    Wire.endTransmission();
}

/**
 * Write value to register reg.
 * @param reg the register address to write value to.
 * @param value the value to write to the register.
 */
void AS3935::writeRegister(uint8_t reg, uint8_t value)
{
    writeRegisterWithMask(reg, 0xff, value);
}

/**
 * Sets all registers in default mode
 */
void AS3935::setDefault(void)
{
    writeRegister(0x3c, 0x96);
}

/**
 * Calibrates the internal RC Oscillators automatically
 */
void AS3935::calibrateRCO(void)
{
    writeRegister(0x3D, 0x96);
}

/**
 * Disable LCO/SRCO/TRCO on IRQ pin.
 */
void AS3935::disableOscillators(void)
{
    writeRegisterWithMask(0x08, 0xE0, 0x00);
}

/**
 * Get intrrupt reason
 * @return one of AS3935_INT_STRIKE, AS3935_INT_DISTURBER, AS3935_INT_NOISE
 */
uint8_t AS3935::getIntrruptReason(void)
{
    return readRegisterWithMask(0x03, 0b00001111);
}

/**
 * Return the estimated distance in km to the head of an approaching storm.
 * @return int8_t value of the estimated distance in km,
 * AS3935_DISTANCE_OUT_OF_RANGE when out of range, or -1 when the register
 * value is invalid. See also: 8.9.3 Statistical Distance Estimation
 */
int8_t AS3935::getDistance(void)
{
    uint8_t v;
    int8_t d;
    v = readRegisterWithMask(0x07, 0b00111111);
    switch(v) {
        case 0b111111:
            d = AS3935_DISTANCE_OUT_OF_RANGE;
            break;
        case 0b101000:
            d = 40;
            break;
        case 0b100101:
            d = 37;
            break;
        case 0b100010:
            d = 34;
            break;
        case 0b011111:
            d = 31;
            break;
        case 0b011011:
            d = 27;
            break;
        case 0b011000:
            d = 24;
            break;
        case 0b010100:
            d = 20;
            break;
        case 0b010001:
            d = 17;
            break;
        case 0b001110:
            d = 14;
            break;
        case 0b001100:
            d = 12;
            break;
        case 0b001010:
            d = 10;
            break;
        case 0b001000:
            d = 8;
            break;
        case 0b000110:
            d = 6;
            break;
        case 0b000101:
            d = 5;
            break;
        case 0b000001:
            d = 0;
            break;
        default:
            d = -1;
            break;
    }
    return d;
}

/**
 * Returns bool whether or not current AFE setting is indoor.
 * @return bool
 */
bool AS3935::isIndoor()
{
    return readRegisterWithMask(0x00, 0b11000001) == AS3935_AFE_INDOOR;
}

/**
 * Set AFE setting to indoor mode
 * @return true or false whether if setting to indoor mode succeeded.
 */
bool AS3935::setIndoor()
{
    writeRegisterWithMask(0x00, 0b11000001, AS3935_AFE_INDOOR);
    return isIndoor();
}

/**
 * Set or unset AFE setting to indoor mode.
 * @param bool
 * @return true or false whether if setting to indoor mode succeeded.
 */
bool AS3935::setIndoor(bool enable)
{
    return enable ? setIndoor() : setOutdoor();
}

/**
 * Returns bool whether or not current AFE setting is outdoor.
 * @return bool
 */
bool AS3935::isOutdoor()
{
    return readRegisterWithMask(0x00, 0b11000001) == AS3935_AFE_OUTDOOR;
}

/**
 * Set or unset AFE setting to outdoor mode.
 * @param bool
 * @return true or false whether if setting to outdoor mode succeeded.
 */
bool AS3935::setOutdoor()
{
    writeRegisterWithMask(0x00, 0b11000001, AS3935_AFE_OUTDOOR);
    return isOutdoor();
}

/**
 * Set or unset AFE setting to outdoor mode.
 * @param bool
 * @return true or false whether if setting to outdoor mode succeeded.
 */
bool AS3935::setOutdoor(bool enable)
{
    return enable ? setOutdoor() : setIndoor();
}

/**
 * Get minimum number of lightning
 * @return uint8_t number of minimum number of lightning, one of 1, 5, 9, or
 * 16.
 */
uint8_t AS3935::getMinimumLightning(void)
{
    switch(readRegisterWithMask(0x02, 0b11001111)) {
        case 0:
            return 1;
            break;
        case 1:
            return 5;
            break;
        case 2:
            return 9;
            break;
        case 3:
            return 16;
            break;
        default:
            return 255;
            break;
    }
}

/**
 * Set minimum number of lightning to trigger an event
 * @param uint8_t number, one of 1, 5, 9, or 16.
 * @return bool whether or not setting the value succeeded.
 */
bool AS3935::setMinimumLightning(uint8_t n)
{
    if (n == 1 || n == 5 || n == 9 || n == 16) {
        writeRegisterWithMask(0x02, 0b11001111, n);
        return getMinimumLightning();
    }
    return false;
}

/**
 * Clear the statistics built up by the lightning distance estimation algorithm
 * block.
 */
void AS3935::clearStats(void)
{
    writeRegisterWithMask(0x02, 0b10111111, 1);
    delay(2);
    writeRegisterWithMask(0x02, 0b10111111, 0);
    delay(2);
    writeRegisterWithMask(0x02, 0b10111111, 1);
    delay(2);
}