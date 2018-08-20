/*
 * Support for the Semtech SX1509 GPIO expanders
 *
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * resarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 *
 */



#ifndef H_SX1509
#define H_SX1509

#include "os/mynewt.h"

#ifdef __cplusplus
extern "C" {
#endif



#define SX1509_PIN_INPUT_DISABLE     (1 << 0)   // default: cleared: input being used
#define SX1509_PIN_INCREASE_SLEW     (1 << 1)   // default: cleared: increased slew disabled
#define SX1509_PIN_LOW_DRIVE_ENABLE  (1 << 2)   // default: cleared: low drive disabled
#define SX1509_PIN_PULL_UP_ENABLE    (1 << 3)   // default: cleared: pull up disabled
#define SX1509_PIN_PULL_DOWN_ENABLE  (1 << 4)   // default: cleared: pull down disabled
#define SX1509_PIN_OPENDRAIN         (1 << 5)   // default: cleared: push-pull
#define SX1509_PIN_INVERT_POLARITY   (1 << 6)   // default: cleared: normal polarity
#define SX1509_PIN_INPUT             (1 << 7)   // default: set:     input
#define SX1509_PIN_INTERRUPT_MASK    (1 << 8)   // default: set:     no interrupt
#define SX1509_PIN_LED_DRIVER_ENABLE (1 << 9)   // default: cleared: led driver disabled
#define SX1509_PIN_DEBOUNCE_ENABLE   (1 << 10)  // default: cleared: debounce disabled
#define SX1509_PIN_HIGH_INPUT_ENABLE (1 << 11)  // default: cleared: max 3.6V
#define SX1509_PIN_BREATHE           (1 << 12)  // default: non-breathing (pwm only)

#define SX1509_REGISTER_COUNT      (0x6A)


#define SX1509_REG_INPUTDISABLE    (0x00)
#define SX1509_REG_LONGSLEW        (0x02)
#define SX1509_REG_LOWDRIVE        (0x04)
#define SX1509_REG_PULLUP          (0x06)
#define SX1509_REG_PULLDOWN        (0x08)
#define SX1509_REG_OPENDRAIN       (0x0A)
#define SX1509_REG_POLARITY        (0x0C)
#define SX1509_REG_DIR             (0x0E)
#define SX1509_REG_DATA            (0x10)
#define SX1509_REG_INTERRUPTMASK   (0x12)
#define SX1509_REG_SENSE           (0x14)
#define SX1509_REG_INTERRUPTSRC    (0x18)
#define SX1509_REG_EVENTSTATUS     (0x1A)
#define SX1509_REG_LEVELSHIFTER    (0x1C)

#define SX1509_REG_CLOCK           (0x1E)
#define SX1509_CLOCK_FREQ_OFFSET   (0)
#define SX1509_CLOCK_FREQ_WIDTH    (0b00001111)
#define SX1509_CLOCK_OSCIO_OFFSET  (4)
#define SX1509_CLOCK_OSCIO_WIDTH   (0b00000001)
#define SX1509_CLOCK_SOURCE_OFFSET (5)
#define SX1509_CLOCK_SOURCE_WIDTH  (0b00000011)


#define SX1509_REG_MISC            (0x1F)
#define SX1509_REG_LEDDRIVERENABLE (0x20)
#define SX1509_REG_DEBOUNCECFG     (0x22)
#define SX1509_REG_DEBOUNCEENABLE  (0x23)
#define SX1509_REG_KEYCONFIG       (0x25)
#define SX1509_REG_KEYDATA_COLUMN  (0x27)
#define SX1509_REG_KEYDATA_ROW     (0x28)
#define SX1509_REG_HIGHINPUT       (0x69)
#define SX1509_REG_RESET           (0x7D)

/* Register map for the Time On setting (per pin) */
#define SX1509_T_ON        {0x29, 0x2C, 0x2F, 0x32, 0x35, 0x3A, 0x3F, 0x44, \
                            0x49, 0x4C, 0x4F, 0x52, 0x55, 0x5A, 0x5F, 0x64}

/* Register map for the Intensity setting (per pin) */
#define SX1509_I_ON        {0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45, \
                            0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x60, 0x65}

/* Register map for the Time Off setting (per pin) */
#define SX1509_T_OFF       {0x2B, 0x2E, 0x31, 0x34, 0x37, 0x3C, 0x41, 0x46, \
                            0x4B, 0x4E, 0x51, 0x54, 0x57, 0x5C, 0x61, 0x66}

/* Register map for the Fade In setting (per pin, 0 == not supported) */
#define SX1509_T_RISE      {0x00, 0x00, 0x00, 0x00, 0x38, 0x3D, 0x42, 0x47, \
                            0x00, 0x00, 0x00, 0x00, 0x58, 0x5D, 0x62, 0x67}

/* Register map for the Fade Out setting (per pin, 0 == not supported) */
#define SX1509_T_FALL      {0x00, 0x00, 0x00, 0x00, 0x39, 0x3E, 0x43, 0x48, \
                            0x00, 0x00, 0x00, 0x00, 0x59, 0x5E, 0x63, 0x68}

#define SX1509_PIN_COUNT  (15)


    enum sx1509_clock_source {
        sx1509_clock_off = 0,
        sx1509_clock_external = 1,
        sx1509_clock_internal = 2,
        sx1509_clock_reserved = 3
    };

    enum sx1509_oscio {
        sx1509_oscio_input = 0,
        sx1509_oscio_output = 1
    };


    enum sx1509_edge_sense {
        sx1509_edge_none = 0,
        sx1509_edge_rising = 1,
        sx1509_edge_falling = 2,
        sx1509_edge_both = 3
    };

    struct sx1509_io_cfg {
        uint8_t pin;            // 0-15
        uint8_t value;          // data bit: 0, 1
        enum sx1509_edge_sense edge_sense;      // when to trigger an interrupt
        uint8_t intensity;      // PWM value for output when ON
        uint8_t intensity_off;  // PWM value for output when OFF
        uint8_t time_on;        // applicable to blinking lights: time the output is active
        uint8_t time_off;       // applicable to blinking lights: time the output is non-active
        uint8_t fade_in;        // ramp time for turning a light on
        uint8_t fade_out;       // ramp time for turning a light off
        uint16_t flags_set;
        uint16_t flags_clear;
    };


    struct sx1509_cfg {
        uint8_t i2c_bus_num;
        uint8_t i2c_address;
        enum sx1509_clock_source clock_source;
        enum sx1509_oscio oscio;
        uint8_t oscio_frequency;
        uint8_t misc_reg;
        uint8_t debounce_time;
        uint8_t io_count;
        struct sx1509_io_cfg io[];
    };


    struct sx1509 {
        struct os_dev dev;
        struct sx1509_cfg *cfg;
    };

/**
 * Initialize the sx1509. This is normally used as an os_dev_create initialization function
 *
 * @param dev  Pointer to the sx1509_dev device descriptor
 * @param arg  Pointer to the sx1509 configuration struct
 *
 * @return 0 on success, non-zero on failure
 */
    int sx1509_init(struct os_dev *dev, void *arg);


    int sx1509_configure_pin(struct sx1509 *sx1509, struct sx1509_io_cfg io);


    int sx1509_read(struct sx1509 *sx1509, uint8_t reg, uint8_t * buffer, uint8_t len);

    int sx1509_read8(struct sx1509 *sx1509, uint8_t reg, uint8_t * value);

    int sx1509_read16(struct sx1509 *sx1509, uint8_t reg, uint16_t * value);

    int sx1509_write8(struct sx1509 *sx1509, uint8_t reg, uint8_t value);

    int sx1509_write16(struct sx1509 *sx1509, uint8_t reg, uint16_t value);

    int sx1509_write16(struct sx1509 *sx1509, uint8_t reg, uint16_t value);

    int sx1509_modify8(struct sx1509 *sx1509, uint8_t reg, uint8_t set, uint8_t clear);

    int sx1509_modify16(struct sx1509 *sx1509, uint8_t reg, uint16_t set, uint16_t clear);

    int sx1509_reset(struct sx1509 *sx1509);

    int sx1509_set_clock(struct sx1509 *sx1509,
                         enum sx1509_clock_source source,
                         enum sx1509_oscio oscio, uint8_t oscio_frequency);

    int sx1509_get_clock(struct sx1509 *sx1509,
                         enum sx1509_clock_source *source,
                         enum sx1509_oscio *oscio, uint8_t * oscio_frequency);


    int sx1509_get_data(struct sx1509 *sx1509, uint16_t * data);
    int sx1509_modify_data(struct sx1509 *sx1509, uint16_t, uint16_t clear);

    int sx1509_get_input_disable(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_input_disable(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_long_slew(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_long_slew(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_low_drive(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_low_drive(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_pullup(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_pull_up(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_pulldown(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_pull_down(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_opendrain(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_opendrain(struct sx1509 *sx1509, uint16_t set, uint16_t clear);


    int sx1509_get_polarity(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_polarity(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_direction(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_direction(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_interrupt_mask(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_interrupt_mask(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_sense(struct sx1509 *sx1509, uint32_t * data);

    int sx1509_modify_sense(struct sx1509 *sx1509, uint32_t set, uint32_t clear);

    int sx1509_get_interrupt_source(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_interrupt_source(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_event_status(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_event_status(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_level_shifter(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_level_shifter(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_led_driver_enabled(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_led_driver_enabled(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_debounce_enabled(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_debounce_enabled(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_get_high_input(struct sx1509 *sx1509, uint16_t * data);

    int sx1509_modify_high_input(struct sx1509 *sx1509, uint16_t set, uint16_t clear);

    int sx1509_set_intensity(struct sx1509 *sx1509, uint8_t pin, uint8_t intensity);

    int sx1509_get_intensity(struct sx1509 *sx1509, uint8_t pin, uint8_t * intensity);

    int sx1509_set_time_on(struct sx1509 *sx1509, uint8_t pin, uint8_t on_time);

    int sx1509_set_time_off(struct sx1509 *sx1509, uint8_t pin,
                            uint8_t off_time, uint8_t off_intensity);

    int sx1509_set_rise_time(struct sx1509 *sx1509, uint8_t pin, uint8_t rise_time);

    int sx1509_set_fall_time(struct sx1509 *sx1509, uint8_t pin, uint8_t fall_time);

    int sx1509_set_misc(struct sx1509 *sx1509,
                        int bankA,
                        int bankB,
                        uint8_t freq,
                        int nreset,
                        int auto_incr,
                        int auto_clear);

    int sx1509_set_debounce_time(struct sx1509 *sx1509, uint8_t debounce_time);

    int sx1509_get_debounce_time(struct sx1509 *sx1509, uint8_t * debounce_time);

    int sx1509_set_output(struct sx1509 *sx1509, uint8_t pin, uint8_t state);

    int sx1509_set_output_level(struct sx1509 *sx1509, uint8_t pin, uint8_t level);

    int sx1509_get_output_level(struct sx1509 *sx1509, uint8_t pin, uint8_t * level);

    int sx1509_support_breathe(uint8_t pin);

    int sx1509_set_sense(struct sx1509 *sx1509, uint8_t pin, enum sx1509_edge_sense edge_sense);



#if MYNEWT_VAL(SX1509_CLI)
/**
 * Initialize the SX1509 shell commands -- called via sysinit
 *
 * @return 0 on success, non-zero otherwise
 */
    int sx1509_shell_init(void);
#endif



#ifdef __cpluscplus
}
#endif
#endif                          /* H_SX1509 */
