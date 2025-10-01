/*
 * File: dwm_pico_AS5600.h
 * Project: dwm_pico_as5600
 * -----
 * This source code is released under BSD-3 license.
 * Check LICENSE file for full license agreement.
 * Check COPYING for 3rd party licenses.
 * -----
 * Copyright 2024 M.Kusiak (Timax)
 */

#ifndef DWM_PICO_AS5600_H
#define DWM_PICO_AS5600_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define AS5600_MAX_ANGLE 4095

/**
 * @brief AS5600 instance.
 */
typedef struct
{
    uint8_t sda;
    uint8_t clk;
    i2c_inst_t* i2c_inst;
} as5600_t;

/**
 * @brief AS5600 power mode variable.
 */
typedef enum {
    AS5600_VAR_PM_NOM = 0b00,
    AS5600_VAR_PM_LPM1 = 0b01,
    AS5600_VAR_PM_LPM2 = 0b10,
    AS5600_VAR_PM_LPM3 = 0b11
} as5600_var_pm_t;

/**
 * @brief AS5600 hysteresis variable.
 */
typedef enum {
    AS5600_VAR_HYST_0LSB = 0b00, // OFF
    AS5600_VAR_HYST_1LSB = 0b01,
    AS5600_VAR_HYST_2LSB = 0b10,
    AS5600_VAR_HYST_3LSB = 0b11
} as5600_var_hyst_t;

/**
 * @brief AS5600 output stage variable.
 */
typedef enum {
    AS5600_VAR_OUTS_ANALOG_FULL = 0b00,
    AS5600_VAR_OUTS_ANALOG_REDUCED = 0b01,
    AS5600_VAR_OUTS_DIGITAL_PWM = 0b10
} as5600_var_outs_t;

/**
 * @brief AS5600 PWM frequency variable.
 */
typedef enum {
    AS5600_VAR_PWMF_115HZ = 0b00,
    AS5600_VAR_PWMF_230HZ = 0b01,
    AS5600_VAR_PWMF_460HZ = 0b10,
    AS5600_VAR_PWMF_920HZ = 0b11
} as5600_var_pwmf_t;

/**
 * @brief AS5600 slow filter variable.
 */
typedef enum {
    AS5600_VAR_SF_16X = 0b00,
    AS5600_VAR_SF_8X = 0b01,
    AS5600_VAR_SF_4X = 0b10,
    AS5600_VAR_SF_2X = 0b11
} as5600_var_sf_t;

/**
 * @brief AS5600 fast filter threshold variable.
 */
typedef enum {
    AS5600_VAR_FTH_0LSB = 0b000, // Slow filter only
    AS5600_VAR_FTH_6LSB = 0b001,
    AS5600_VAR_FTH_7LSB = 0b010,
    AS5600_VAR_FTH_9LSB = 0b011,
    AS5600_VAR_FTH_18LSB = 0b100,
    AS5600_VAR_FTH_21LSB = 0b101,
    AS5600_VAR_FTH_24LSB = 0b110,
    AS5600_VAR_FTH_10LSB = 0b111
} as5600_var_fth_t;

/**
 * @brief AS5600 watchdog variable.
 */
typedef enum {
    AS5600_VAR_WD_OFF = 0b0,
    AS5600_VAR_WD_ON = 0b1
} as5600_var_wd_t;

/**
 * @brief AS5600 configuration.
 */
typedef struct
{
    as5600_var_pm_t pm; // Power mode
    as5600_var_hyst_t hyst; // Hysteresis
    as5600_var_outs_t outs; // Output stage
    as5600_var_pwmf_t pwmf; // PWM frequency
    as5600_var_sf_t sf; // Slow filter
    as5600_var_fth_t fth; // Fast filter threshold
    as5600_var_wd_t wd; // Watchdog
} as5600_conf_t;

/**
 * @brief  AS5600 status registry (magnet) statuses.
 */
typedef enum {
    MD = 1 << 5, // Magnet detected
    ML = 1 << 4, // Magnet too weak
    MH = 1 << 3 // Magnet too strong
} as5600_status_t;

#ifdef __cplusplus
extern "C" {
#endif
// Setup functions

int8_t as5600_init(uint8_t sda, uint8_t clk, as5600_t* as5600);

// Read / write functions

uint8_t as5600_read_zmco(as5600_t* as5600);

void as5600_write_zpos(as5600_t* as5600, uint16_t angl);
uint16_t as5600_read_zpos(as5600_t* as5600);

void as5600_write_mpos(as5600_t* as5600, uint16_t angl);
uint16_t as5600_read_mpos(as5600_t* as5600);

void as5600_write_mang(as5600_t* as5600, uint16_t angl);
uint16_t as5600_read_mang(as5600_t* as5600);

void as5600_write_conf(as5600_t* as5600, as5600_conf_t* as5600_conf);
uint16_t as5600_read_conf(as5600_t* as5600, as5600_conf_t* as5600_conf);

uint16_t as5600_read_raw_angl(as5600_t* as5600);

uint16_t as5600_read_angl(as5600_t* as5600);

as5600_status_t as5600_read_status(as5600_t* as5600);

uint8_t as5600_read_agc(as5600_t* as5600);

int16_t as5600_read_magnitude(as5600_t* as5600);

// Burn functions (untested)

void as5600_burn_angle(as5600_t* as5600);
void as5600_burn_setting(as5600_t* as5600);

// Utility functions

uint16_t as5600_mang_to_mpos(uint16_t zpos, uint16_t mang);
uint16_t as5600_angl_to_degr(uint16_t angl, uint16_t zpos, uint16_t mpos);
float as5600_angl_to_degr_float(uint16_t angl, uint16_t zpos, uint16_t mpos);
uint16_t as5600_float_degrees_to_angl(float degr);
uint16_t as5600_degrees_to_angl(uint16_t degr);
int8_t as5600_status_to_scale(as5600_status_t status);
#ifdef __cplusplus
}
#endif

#endif