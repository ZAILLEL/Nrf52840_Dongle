#pragma once


#include "max30102_defines.h"


#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>


typedef enum max30102_mode_t
{
    max30102_heart_rate = 0x02,
    max30102_spo2 = 0x03,
    max30102_multi_led = 0x07
} max30102_mode_t;

typedef enum max30102_smp_ave_t
{
    max30102_smp_ave_1,
    max30102_smp_ave_2,
    max30102_smp_ave_4,
    max30102_smp_ave_8,
    max30102_smp_ave_16,
    max30102_smp_ave_32,
} max30102_smp_ave_t;

typedef enum max30102_sr_t
{
    max30102_sr_50,
    max30102_sr_100,
    max30102_sr_200,
    max30102_sr_400,
    max30102_sr_800,
    max30102_sr_1000,
    max30102_sr_1600,
    max30102_sr_3200
} max30102_sr_t;

typedef enum max30102_led_pw_t
{
    max30102_pw_15_bit,
    max30102_pw_16_bit,
    max30102_pw_17_bit,
    max30102_pw_18_bit
} max30102_led_pw_t;

typedef enum max30102_adc_t
{
    max30102_adc_2048,
    max30102_adc_4096,
    max30102_adc_8192,
    max30102_adc_16384
} max30102_adc_t;

typedef enum max30102_multi_led_ctrl_t
{
    max30102_led_off,
    max30102_led_red,
    max30102_led_ir
} max30102_multi_led_ctrl_t;

typedef struct max30102_t
{
    I2C_HandleTypeDef *_ui2c;
    uint32_t _ir_samples[32];
    uint32_t _red_samples[32];
    uint8_t _interrupt_flag;
} max30102_t;

void max30102_plot(uint32_t ir_sample, uint32_t red_sample);

void max30102_init(max30102_t *obj, I2C_HandleTypeDef *hi2c);
void max30102_write(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);
void max30102_read(max30102_t *obj, uint8_t reg, uint8_t *buf, uint16_t buflen);

void max30102_reset(max30102_t *obj);

void max30102_set_a_full(max30102_t *obj, uint8_t enable);
void max30102_set_ppg_rdy(max30102_t *obj, uint8_t enable);
void max30102_set_alc_ovf(max30102_t *obj, uint8_t enable);
void max30102_set_die_temp_rdy(max30102_t *obj, uint8_t enable);
void max30102_set_die_temp_en(max30102_t *obj, uint8_t enable);

void max30102_on_interrupt(max30102_t *obj);
uint8_t max30102_has_interrupt(max30102_t *obj);
void max30102_interrupt_handler(max30102_t *obj);

void max30102_shutdown(max30102_t *obj, uint8_t shdn);

void max30102_set_mode(max30102_t *obj, max30102_mode_t mode);
void max30102_set_sampling_rate(max30102_t *obj, max30102_sr_t sr);

void max30102_set_led_pulse_width(max30102_t *obj, max30102_led_pw_t pw);
void max30102_set_adc_resolution(max30102_t *obj, max30102_adc_t adc);

void max30102_set_led_current_1(max30102_t *obj, float ma);
void max30102_set_led_current_2(max30102_t *obj, float ma);
void max30102_set_multi_led_slot_1_2(max30102_t *obj, max30102_multi_led_ctrl_t slot1, max30102_multi_led_ctrl_t slot2);
void max30102_set_multi_led_slot_3_4(max30102_t *obj, max30102_multi_led_ctrl_t slot3, max30102_multi_led_ctrl_t slot4);

void max30102_set_fifo_config(max30102_t *obj, max30102_smp_ave_t smp_ave, uint8_t roll_over_en, uint8_t fifo_a_full);
void max30102_clear_fifo(max30102_t *obj);
void max30102_read_fifo(max30102_t *obj);

void max30102_read_temp(max30102_t *obj, int8_t *temp_int, uint8_t *temp_frac);

#endif