#include "magnetic_encoder.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

// Default as5600 i2c address, not configurable.
#define AS5600_DEFAULT_ADDRESS 0x36
// Maximum number of bytes to read or write.
// Does not count registry.
#define AS5600_RW_MAX 2
// Number of bytes needed for registry.
#define REG 1
// Number of bits in byte.
#define BYTE 8
// Max value for byte.
#define BYTE_MAX 255
// Value for permanent as5600 angle burn
#define BURN_ANGLE 0x80
// Value for permanent as5600 settings burn
#define BURN_SETTING 0x40

#define SQUARE(x) ((x) * (x))

/**
 * @brief Available registers for AS5600.
 */
typedef enum {
    // Configuration registers
    ZMCO = 0x00, // ZPOS and MPOS modification counter
    ZPOS = 0x01, // Zero position
    MPOS = 0x03, // Max position
    MANG = 0x05, // Maximum angle
    CONF = 0x07, // Configuration

    // Output registers
    RAW_ANGLE = 0x0c, // Unmodified angle
    ANGLE = 0x0e, // Scaled angle

    // Status registers
    STATUS = 0x0b, // Magnet status
    AGC = 0x1a, // Automatic gain control
    MAGNITUDE = 0x1b, // Magnitude of CORDIC

    // Burn commands
    BURN = 0xff // Burn setting to non-violate memory

} as5600_reg_t;

/**
 * @brief Data size (in bytes) for each register.
 */
typedef enum {
    ZMCO_LEN = 1,
    ZPOS_LEN = 2,
    MPOS_LEN = 2,
    MANG_LEN = 2,
    CONF_LEN = 2,
    RAW_ANGLE_LEN = 2,
    ANGLE_LEN = 2,
    STATUS_LEN = 1,
    AGC_LEN = 1,
    MAGNITUDE_LEN = 2,
    BURN_LEN = 1
} as5600_data_len_t;

/**
 * @brief i2c pin types.
 */
typedef enum {
    CLOCK,
    DATA
} gpio_i2c_t;

/**
 * @brief       Get gpio pin i2c type.
 *
 * @param gpio  gpio pin.
 * @return      i2c pin type.
 */
gpio_i2c_t get_gpio_i2c_type(uint8_t gpio)
{
    if (gpio % 2)
        return CLOCK;
    return DATA;
}

/**
 * @brief       Get i2c instance number.
 *
 * @param gpio  gpio pin.
 * @return      i2c instance number.
 */
i2c_inst_t* get_i2c_inst(uint8_t gpio)
{
    if ((gpio % 4) < 2)
        return i2c0;
    return i2c1;
}

/**
 * @brief           Initialise as5600 module.
 *
 * @param sda       sda (data) gpio number.
 * @param clk       clk (clock) gpio number.
 * @param as5600    as5600 instance to fill.
 * @return          3 if sda not connected to sda pin on pi pico,
 *                  2 if clk not connected to clk pin on pi pico,
 *                  1 if pins are under different i2c instances,
 *                  0 on success.
 *
 * @warning         i2c must be initialised first!
 */
int8_t as5600_init(uint8_t sda, uint8_t clk, as5600_t* as5600)
{
    if (get_gpio_i2c_type(sda) != DATA)
        return 3;
    if (get_gpio_i2c_type(clk) != CLOCK)
        return 2;

    as5600->i2c_inst = get_i2c_inst(sda);

    if (as5600->i2c_inst != get_i2c_inst(clk))
        return 1;

    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(clk, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(clk);

    as5600->sda = sda;
    as5600->clk = clk;

    return 0;
}

/**
 * @brief           Read data from as5600 via i2c, generic function.
 *
 * @param as5600    as5600 instance.
 * @param reg       register to read.
 * @param len       length of data to receive.
 * @return          data received.
 */
uint16_t as5600_read(as5600_t* as5600, uint8_t reg, uint8_t len)
{
    uint8_t buff[AS5600_RW_MAX] = { 0 };

    i2c_write_blocking(as5600->i2c_inst, AS5600_DEFAULT_ADDRESS, &reg, 1, false);
    i2c_read_blocking(as5600->i2c_inst, AS5600_DEFAULT_ADDRESS, buff, len, false);

    if (len == 1)
        return buff[0];
    return (buff[0] << BYTE) | buff[1];
}

/**
 * @brief           Write data to as5600 via i2c, generic function.
 *
 * @param as5600    as5600 instance.
 * @param reg       register to write to.
 * @param val       value to write.
 * @param len       length of data to write (in bytes).
 */
void as5600_write(as5600_t* as5600, uint8_t reg, uint16_t val, uint8_t len)
{
    uint8_t buff[REG + AS5600_RW_MAX] = { 0 };
    len += REG;
    buff[0] = reg;

    if (len == 1)
        buff[1] = val;
    else {
        buff[1] = val >> BYTE;
        buff[2] = val;
    }

    i2c_write_blocking(as5600->i2c_inst, AS5600_DEFAULT_ADDRESS, buff, len, false);
}

/**
 * @brief           Get ZMCO (possibly Zero Maximum Counter) counter value.
 *
 * @param as5600    as5600 instance.
 * @return          ZMCO counter value.
 *
 * @note            ZMCO indicates number of writes to internal memory.
 *                  According to manual, maximum number of writes is 3.
 */
uint8_t as5600_read_zmco(as5600_t* as5600)
{
    return as5600_read(as5600, ZMCO, ZMCO_LEN);
}

/**
 * @brief           Configure zpos (zero position).
 *
 * @param as5600    as5600 instance.
 * @param angl      zero position angle.
 *
 * @note            Setting zero position does not scale resolution to narrower range.
 */
void as5600_write_zpos(as5600_t* as5600, uint16_t angl)
{
    as5600_write(as5600, ZPOS, angl, ZPOS_LEN);
}

/**
 * @brief           Read zpos (zero position).
 *
 * @param as5600    as5600 instance.
 * @return          as5600 zero position.
 */
uint16_t as5600_read_zpos(as5600_t* as5600)
{
    return as5600_read(as5600, ZPOS, ZPOS_LEN);
}

/**
 * @brief           Write mpos (max position).
 *
 * @param as5600    as5600 instance.
 * @param angl      literal (zpos independent) max angle.
 *
 * @note            Setting max position does not scale resolution to narrower range.
 *                  Compared to mang (maximum angle), mpos is literal maximum position,
 *                  while mang is based on zpos (zero position).
 */
void as5600_write_mpos(as5600_t* as5600, uint16_t angl)
{
    as5600_write(as5600, MPOS, angl, MPOS_LEN);
}

/**
 * @brief           Read mpos (max position).
 *
 * @param as5600    as5600 instance.
 * @return          literal (zpos independent) max position value.
 *
 * @note            Compared to mang (maximum angle), mpos is literal maximum position,
 *                  while mang is based on zpos (zero position).
 */
uint16_t as5600_read_mpos(as5600_t* as5600)
{
    return as5600_read(as5600, MPOS, MPOS_LEN);
}

/**
 * @brief           Write mang (maximum angle).
 *
 * @param as5600    as5600 instance.
 * @param angl      maximum angle value.
 *
 * @note            Setting max angle does not scale resolution to narrower range.
 *                  Maximum angle (mang), other than literal maximum position (mpos),
 *                  is based on zero position (zpos).
 */
void as5600_write_mang(as5600_t* as5600, uint16_t angl)
{
    as5600_write(as5600, MANG, angl, MANG_LEN);
}

/**
 * @brief           Read mang (maximum angle).
 *
 * @param as5600    as5600 instance.
 * @return          max angle value.
 *
 * @note            Maximum angle (mang), other than literal maximum position (mpos),
 *                  is based on zero position (zpos).
 */
uint16_t as5600_read_mang(as5600_t* as5600)
{
    return as5600_read(as5600, MANG, MANG_LEN);
}

/**
 * @brief               Write configuration to as5600.
 *
 * @param as5600        as5600 instance.
 * @param as5600_conf   as5600 config to write.
 */
void as5600_write_conf(as5600_t* as5600, as5600_conf_t* as5600_conf)
{
    uint16_t conf_bitmap = 0;

    conf_bitmap |= as5600_conf->pm & 0b11;
    conf_bitmap |= (as5600_conf->hyst << 2) & 0b11;
    conf_bitmap |= (as5600_conf->outs << 4) & 0b11;
    conf_bitmap |= (as5600_conf->pwmf << 6) & 0b11;
    conf_bitmap |= (as5600_conf->sf << 8) & 0b11;
    conf_bitmap |= (as5600_conf->fth << 10) & 0b111;
    conf_bitmap |= (as5600_conf->wd << 13) & 0b1;

    as5600_write(as5600, CONF, conf_bitmap, CONF_LEN);
}

/**
 * @brief               Read configuration from as5600.
 *
 * @param as5600        as5600 instance.
 * @param as5600_conf   as5600 configuration to fill.
 * @return              config bitmap.
 */
uint16_t as5600_read_conf(as5600_t* as5600, as5600_conf_t* as5600_conf)
{
    uint16_t conf_bitmap = as5600_read(as5600, CONF, CONF_LEN);

    as5600_conf->pm = conf_bitmap & 0b11;
    as5600_conf->hyst = (conf_bitmap >> 2) & 0b11;
    as5600_conf->outs = (conf_bitmap >> 4) & 0b11;
    as5600_conf->pwmf = (conf_bitmap >> 6) & 0b11;
    as5600_conf->sf = (conf_bitmap >> 8) & 0b11;
    as5600_conf->fth = (conf_bitmap >> 10) & 0b111;
    as5600_conf->wd = (conf_bitmap >> 13) & 0b1;

    return conf_bitmap;
}

/**
 * @brief           Read raw (literal) angle from as5600.
 *
 * @param as5600    as5600 instance.
 * @return          raw (literal) angle.
 */
uint16_t as5600_read_raw_angl(as5600_t* as5600)
{
    return as5600_read(as5600, RAW_ANGLE, RAW_ANGLE_LEN);
}

/**
 * @brief           Read angle from as5600.
 *
 * @param as5600    as5600 instance.
 * @return          angle.
 */
uint16_t as5600_read_angl(as5600_t* as5600)
{
    return as5600_read(as5600, ANGLE, ANGLE_LEN);
}

/**
 * @brief           Read (magnet) status for AS5600.
 *
 * @param as5600    as5600 instance.
 * @return          - 0x00 if failed to obtain status,
 *                  - as5600_status_t otherwise.
 *
 * @note            Statuses reported by AS5600 are not mutually exclusive, only MH and ML are.
 *                  The raw data might look like garbage but it's expected.
 *                  Took me 2 evenings, additional AS5600 and Reddit search to figure that out.
 *
 *                  What's more, as5600 seems to return MD status only when magnet is off center,
 *                  which does not work when measuring. This makes reading status not reliable
 *                  for calibration.
 */
as5600_status_t as5600_read_status(as5600_t* as5600)
{
    as5600_status_t status = as5600_read(as5600, STATUS, STATUS_LEN);
    status &= (MD | ML | MH);

    return status;
}

/**
 * @brief           Read automatic gain control value.
 *
 * @param as5600    as5600 instance.
 * @return          - 0-255 steps in 5v mode.
 *                  - 0-128 steps in 3v mode.
 *
 * @note            Gain is compensation for variations in magnetic field due to various factors,
 *                  with most important one being magnet air gap.
 *                  Ideally this value should be in the middle of the range.
 */
uint8_t as5600_read_agc(as5600_t* as5600)
{
    return as5600_read(as5600, AGC, AGC_LEN);
}

/**
 * @brief           Read magnitude value calculated by the internal CORDIC.
 *
 * @param as5600    as5600 instance.
 * @return          magnitude value.
 */
int16_t as5600_read_magnitude(as5600_t* as5600)
{
    return as5600_read(as5600, MAGNITUDE, MAGNITUDE_LEN);
}

/**
 * @brief           Permanently burn zpos and mpos to as5600.
 *
 * @param as5600    as5600 instance.
 *
 * @note            Can be executed up to 3 times only if magnet is detected.
 *                  Number of burns is reported by zmco.
 */
void as5600_burn_angle(as5600_t* as5600)
{
    as5600_write(as5600, BURN, BURN_ANGLE, BURN_LEN);
}

/**
 * @brief           Permanently burn config and mang to as5600.
 *
 * @param as5600    as5600 instance.
 *
 * @note            Can be executed only once, only if zpos and mpos were never burned.
 */
void as5600_burn_setting(as5600_t* as5600)
{
    as5600_write(as5600, BURN, BURN_SETTING, BURN_LEN);
}

/**
 * @brief       Convert mang (maximum angle) to mpos (maximum position)
 *
 * @param zpos  zpos (zero position).
 * @param mang  mang (maximum angle).
 * @return      mpos (maximum position).
 */
uint16_t as5600_mang_to_mpos(uint16_t zpos, uint16_t mang)
{
    return (zpos + mang) % AS5600_MAX_ANGLE;
}

/**
 * @brief       Convert angle to degrees.
 *
 * @param angl  angle.
 * @param zpos  zero position (zpos).
 * @param mpos  max position (mpos).
 * @return      angle in degrees (integer).
 *
 * @note        For raw angle 0 (zpos) and AS5600_MAX_ANGLE (mpos) can be used.
 */
uint16_t as5600_angl_to_degr(uint16_t angl, uint16_t zpos, uint16_t mpos)
{
    return (uint32_t)(angl * (mpos - zpos)) / (SQUARE(AS5600_MAX_ANGLE) / 360);
}

/**
 * @brief       Convert angle to degrees.
 *
 * @param angl  angle.
 * @param zpos  zero position (zpos).
 * @param mpos  max position (mpos).
 * @return      angle in degrees (float).
 *
 * @note        For raw angle 0 (zpos) and AS5600_MAX_ANGLE (mpos) can be used.
 */
float as5600_angl_to_degr_float(uint16_t angl, uint16_t zpos, uint16_t mpos)
{
    return (uint32_t)(angl * (mpos - zpos)) / (SQUARE(AS5600_MAX_ANGLE) / 360.0f);
}

/**
 * @brief           Convert degrees (float) to angle.
 *
 * @param degr      angle in degrees.
 * @return          raw angle value.
 */
uint16_t as5600_float_degrees_to_angl(float degr)
{
    degr = fmodf(degr, 360.0f);

    return (AS5600_MAX_ANGLE * degr) / 360.0f;
}

/**
 * @brief           Convert degrees (integer) to angle.
 *
 * @param degr      angle in degrees.
 * @return          raw angle value.
 */
uint16_t as5600_degrees_to_angl(uint16_t degr)
{
    degr %= 360;

    return (AS5600_MAX_ANGLE * degr) / 360;
}

/**
 * @brief           Get scale for (magnet) status, more human readable.
 *
 * @param status    AS5600 (magnet) status.
 * @return          - (-2)  magnet too close.
 *                  - (-1) magnet too close, but working.
 *                  - 0 perfect magnet distance.
 *                  - 1 magnet too far, but working.
 *                  - 2 magnet too far.
 *
 * @note            as5600 seems to return MD status (0) only when magnet is off center,
 *                  which does not work when measuring. This makes reading status not reliable
 *                  for calibration.
 */
int8_t as5600_status_to_scale(as5600_status_t status)
{
    if (status & MH)
        if (status & MD)
            return 1;
        else
            return 2;
    if (status & ML)
        if (status & MD)
            return -1;
        else
            return -2;
    return 0;
}