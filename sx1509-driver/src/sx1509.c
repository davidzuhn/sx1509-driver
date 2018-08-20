/*
 * Support for the Semtech SX150[789] series of GPIO expanders
 *
 * This software is tested only on the SX1509, but is designed to
 * work with the entire family
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

#include <string.h>

#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "hal/hal_gpio.h"

#include "sx1509/sx1509.h"

#if MYNEWT_VAL(SX1509_LOG)
#include "log/log.h"
#endif

#if MYNEWT_VAL(SX1509_LOG)
#define LOG_MODULE_SX1509 (LOG_MODULE_DEFAULT)
#define SX1509_INFO(...)  LOG_INFO(&_log, LOG_MODULE_SX1509, __VA_ARGS__)
#define SX1509_ERR(...)   LOG_ERROR(&_log, LOG_MODULE_SX1509, __VA_ARGS__)
static struct log _log;
#else
#define SX1509_INFO(...)
#define SX1509_ERR(...)
#endif


static uint8_t reg_intensity[] = SX1509_I_ON;

static uint8_t reg_time_on[] = SX1509_T_ON;

static uint8_t reg_time_off[] = SX1509_T_OFF;

static uint8_t reg_rise_time[] = SX1509_T_RISE;

static uint8_t reg_fall_time[] = SX1509_T_FALL;




/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this GPIO expander
 * @param An sx1509_cfg structure
 *
 * @return 0 on success, non-zero error on failure.
 */
int
sx1509_init(struct os_dev *dev, void *arg)
{
    struct sx1509 *sx1509;
    int rc;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    sx1509 = (struct sx1509 *) dev;

    sx1509->cfg = (struct sx1509_cfg *) arg;

    rc = sx1509_reset(sx1509);
    assert(rc == 0);

    rc = sx1509_set_clock(sx1509,
                          sx1509->cfg->clock_source,
                          sx1509->cfg->oscio, sx1509->cfg->oscio_frequency);
    assert(rc == 0);

    rc = sx1509_set_debounce_time(sx1509, sx1509->cfg->debounce_time);
    assert(rc == 0);

    rc = sx1509_write8(sx1509, SX1509_REG_MISC, sx1509->cfg->misc_reg);
    assert(rc == 0);

    for (int i = 0; i < sx1509->cfg->io_count; i++) {
        rc = sx1509_configure_pin(sx1509, sx1509->cfg->io[i]);
        assert(rc == 0);
    }



    SX1509_INFO("SX1509 initialized at i2c:0x%02:0x02x\n",
                sx1509->cfg->i2c_bus_num, sx1509->cfg->i2c_address);
    return (0);
  err:
    SX1509_ERR("Error initializing SX1509: %d\n", rc);
    return (rc);
}


#define set_registers(PINNAME, FUNCTION_NAME) \
        if (io.flags_set & PINNAME) { \
            rc = sx1509_modify_##FUNCTION_NAME(sx1509, 1 << io.pin, 0); \
        } else if (io.flags_clear & PINNAME) { \
            rc = sx1509_modify_##FUNCTION_NAME(sx1509, 0, 1 << io.pin); \
        }


int
sx1509_configure_pin(struct sx1509 *sx1509, struct sx1509_io_cfg io)
{
    int rc = OS_EINVAL;

    if (io.pin <= SX1509_PIN_COUNT) {

        set_registers(SX1509_PIN_INPUT_DISABLE, input_disable);
        set_registers(SX1509_PIN_INCREASE_SLEW, long_slew);
        set_registers(SX1509_PIN_LOW_DRIVE_ENABLE, low_drive);
        set_registers(SX1509_PIN_PULL_UP_ENABLE, pull_up);
        set_registers(SX1509_PIN_PULL_DOWN_ENABLE, pull_down);
        set_registers(SX1509_PIN_OPENDRAIN, opendrain);
        set_registers(SX1509_PIN_INVERT_POLARITY, polarity);
        set_registers(SX1509_PIN_INPUT, direction);
        set_registers(SX1509_PIN_INTERRUPT_MASK, interrupt_mask);
        set_registers(SX1509_PIN_LED_DRIVER_ENABLE, led_driver_enabled);
        set_registers(SX1509_PIN_DEBOUNCE_ENABLE, debounce_enabled);
        set_registers(SX1509_PIN_HIGH_INPUT_ENABLE, high_input);

        // Every pin is capable of blinking
        rc = sx1509_set_time_on(sx1509, io.pin, io.time_on);
        assert(rc == 0);
        rc = sx1509_set_time_off(sx1509, io.pin, io.time_off, io.intensity_off & 0b111);
        assert(rc == 0);

        if (io.flags_clear & SX1509_PIN_INTERRUPT_MASK) {
            rc = sx1509_set_sense(sx1509, io.pin, io.edge_sense);
            assert(rc == 0);
        }

        if (io.flags_set & SX1509_PIN_LED_DRIVER_ENABLE && sx1509_support_breathe(io.pin)) {
            rc = sx1509_set_rise_time(sx1509, io.pin, io.fade_in);
            assert(rc == 0);
            rc = sx1509_set_fall_time(sx1509, io.pin, io.fade_out);
            assert(rc == 0);
        }


        rc = sx1509_set_intensity(sx1509, io.pin, io.intensity);
        assert(rc == 0);

        rc = sx1509_set_output(sx1509, io.pin, io.value);
        assert(rc == 0);

    }


    return rc;
}



/**
 * Reads a single byte from the specified register
 *
 * @param The SX1509 device struct
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
sx1509_read8(struct sx1509 *sx1509, uint8_t reg, uint8_t * value)
{
    int rc;
    uint8_t payload;

    struct hal_i2c_master_data data_struct = {
        .address = sx1509->cfg->i2c_address,
        .len = 1,
        .buffer = &payload
    };

    /* Register write */
    payload = reg;
    rc = hal_i2c_master_write(sx1509->cfg->i2c_bus_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        SX1509_ERR("I2C register write failed at address 0x%02X:0x%02X\n",
                   data_struct.address, reg);
        goto err;
    }

    /* Read one byte back */
    payload = 0;
    rc = hal_i2c_master_read(sx1509->cfg->i2c_bus_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    *value = payload;
    if (rc) {
        SX1509_ERR("Failed to read from 0x%02X:0x%02X\n", data_struct.address, reg);
    }

  err:
    return rc;
}





/**
 * Read a variable amount of data from the SX1509 registers
 *
 * @param The SX1509 structure
 * @param Register to start reading from
 * @param Buffer to read into
 * @param Length of the buffer
 *
 * @return 0 on success and non-zero on failure
 */
int
sx1509_read(struct sx1509 *sx1509, uint8_t reg, uint8_t * buffer, uint8_t len)
{
#define READ_BUFFER_LEN (23)
    int rc;
    uint8_t payload[READ_BUFFER_LEN] = { reg, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0
    };

    struct hal_i2c_master_data data_struct = {
        .address = sx1509->cfg->i2c_address,
        .len = 1,
        .buffer = payload
    };

    if (len > READ_BUFFER_LEN) {
        rc = OS_INVALID_PARM;
        goto err;
    }


    /* Clear the supplied buffer */
    memset(buffer, 0, len);

    /* Register write */
    rc = hal_i2c_master_write(sx1509->cfg->i2c_bus_num, &data_struct, OS_TICKS_PER_SEC / 10, 0);
    if (rc) {
        SX1509_ERR("I2C access failed at address 0x%02X\n", data_struct.address);
        goto err;
    }

    /* Read len bytes back */
    memset(payload, 0, sizeof(payload));
    data_struct.len = len;
    rc = hal_i2c_master_read(sx1509->cfg->i2c_bus_num, &data_struct, OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        SX1509_ERR("Failed to read from 0x%02X:0x%02X\n", data_struct.address, reg);
#if MYNEWT_VAL(SX1509_STATS)
        STATS_INC(g_sx1509stats, errors);
#endif
        goto err;
    }

    /* Copy the I2C results into the supplied buffer */
    memcpy(buffer, payload, len);

    return 0;
  err:
    return rc;
}



int
sx1509_read16(struct sx1509 *sx1509, uint8_t reg, uint16_t * value)
{
    int rc;
    uint16_t retval;
    uint8_t tmp;
    rc = sx1509_read(sx1509, reg, &tmp, 1);
    assert(rc == 0);
    retval = tmp << 8;
    rc = sx1509_read(sx1509, reg + 1, &tmp, 1);
    assert(rc == 0);
    retval += tmp;

    *value = retval;

    return rc;
}


int
sx1509_read32(struct sx1509 *sx1509, uint8_t reg, uint32_t * value)
{
    int rc;
    uint32_t retval = 0;
    uint8_t tmp;
    rc = sx1509_read(sx1509, reg, &tmp, 1);
    assert(rc == 0);
    retval += tmp * 0x01000000;

    rc = sx1509_read(sx1509, reg + 1, &tmp, 1);
    assert(rc == 0);
    retval += tmp * 0x00010000;

    rc = sx1509_read(sx1509, reg + 2, &tmp, 1);
    assert(rc == 0);
    retval += tmp * 0x00000100;

    rc = sx1509_read(sx1509, reg + 3, &tmp, 1);
    assert(rc == 0);
    retval += tmp * 0x00000001;


    *value = retval;

    return rc;
}


/**
 * Writes a single byte to the specified register
 *
 * @param The SX1509 device structure
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
sx1509_write8(struct sx1509 *sx1509, uint8_t reg, uint8_t value)
{
    int rc;
    uint8_t payload[2] = { reg, value };

    struct hal_i2c_master_data data_struct = {
        .address = sx1509->cfg->i2c_address,
        .len = 2,
        .buffer = payload
    };

    rc = hal_i2c_master_write(sx1509->cfg->i2c_bus_num, &data_struct, OS_TICKS_PER_SEC, 1);
    if (rc) {
        SX1509_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02X\n",
                   data_struct.address, reg, value);
    }
    return rc;
}


int
sx1509_write16(struct sx1509 *sx1509, uint8_t reg, uint16_t value)
{
    int rc;

    rc = sx1509_write8(sx1509, reg, (uint8_t) (value >> 8));
    rc = sx1509_write8(sx1509, reg + 1, (uint8_t) (value & 0x00FF));
    return rc;
}


int
sx1509_modify8(struct sx1509 *sx1509, uint8_t reg, uint8_t set, uint8_t clear)
{
    uint8_t current;
    int rc;

    rc = sx1509_read8(sx1509, reg, &current);
    if (rc == 0) {
        uint8_t newvalue = current;
        newvalue |= set;
        newvalue &= ~(clear);

        rc = sx1509_write8(sx1509, reg, newvalue);
    }

    return rc;
}


int
sx1509_modify16(struct sx1509 *sx1509, uint8_t reg, uint16_t set, uint16_t clear)
{
    uint16_t current;
    int rc;

    rc = sx1509_read16(sx1509, reg, &current);
    if (rc == 0) {
        uint16_t newvalue = current;
        newvalue |= set;
        newvalue &= ~(clear);

        rc = sx1509_write16(sx1509, reg, newvalue);
    }

    return rc;
}





/**
 * Reset the SX1509 to Power On State
 *
 * @param The SX1509 device structure
 *
 * @return 0 on success, non-zero error on failure.
 */
int
sx1509_reset(struct sx1509 *sx1509)
{
    int rc;
    rc = sx1509_write8(sx1509, SX1509_REG_RESET, 0x12);
    if (rc == 0) {
        rc = sx1509_write8(sx1509, SX1509_REG_RESET, 0x34);

        if (rc == 0) {
            uint32_t ticks;
            rc = os_time_ms_to_ticks(1, &ticks);
            if (rc == 0) {
                os_time_delay(ticks);
            }
        }
    }

    return rc;
}




int
sx1509_set_clock(struct sx1509 *sx1509,
                 enum sx1509_clock_source source, enum sx1509_oscio oscio, uint8_t oscio_frequency)
{
    int rc = 0;
    uint8_t value;

    value = (source << SX1509_CLOCK_SOURCE_OFFSET) |
        (oscio << SX1509_CLOCK_OSCIO_OFFSET) | (oscio_frequency << SX1509_CLOCK_FREQ_OFFSET);

    rc = sx1509_write8(sx1509, SX1509_REG_CLOCK, value);

    return rc;
}

int
sx1509_get_clock(struct sx1509 *sx1509,
                 enum sx1509_clock_source *source,
                 enum sx1509_oscio *oscio, uint8_t * oscio_frequency)
{
    int rc = 0;

    uint8_t value;

    rc = sx1509_read8(sx1509, SX1509_REG_CLOCK, &value);
    if (rc == 0) {
        if (oscio_frequency != NULL) {
            uint8_t tmp = (value >> SX1509_CLOCK_FREQ_OFFSET) & SX1509_CLOCK_FREQ_WIDTH;
            *oscio_frequency = tmp;
        }

        if (oscio != NULL) {
            uint8_t tmp = (value >> SX1509_CLOCK_OSCIO_OFFSET) & SX1509_CLOCK_OSCIO_WIDTH;
            *oscio = tmp;
        }

        if (source != NULL) {
            uint8_t tmp = (value >> SX1509_CLOCK_SOURCE_OFFSET) & SX1509_CLOCK_SOURCE_WIDTH;
            *source = tmp;

        }
    }

    return rc;
}


int
sx1509_get_data(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_DATA, data);
    return rc;
}


int
sx1509_modify_data(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_DATA, set, clear);
    return rc;
}



int
sx1509_get_input_disable(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);
    int rc = sx1509_read16(sx1509, SX1509_REG_INPUTDISABLE, data);
    return rc;
}

int
sx1509_modify_input_disable(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_INPUTDISABLE, set, clear);
    return rc;
}


int
sx1509_get_long_slew(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_LONGSLEW, data);
    return rc;
}

int
sx1509_modify_long_slew(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_LONGSLEW, set, clear);
    return rc;
}

int
sx1509_get_low_drive(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_LOWDRIVE, data);
    return rc;
}


int
sx1509_modify_low_drive(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_LOWDRIVE, set, clear);
    return rc;
}

int
sx1509_get_pullup(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_PULLUP, data);
    return rc;
}


int
sx1509_modify_pull_up(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_PULLUP, set, clear);
    return rc;
}

int
sx1509_get_pulldown(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_PULLDOWN, data);
    return rc;
}

int
sx1509_modify_pull_down(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_PULLDOWN, set, clear);
    return rc;
}


int
sx1509_get_opendrain(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_OPENDRAIN, data);
    return rc;
}

int
sx1509_modify_opendrain(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_OPENDRAIN, set, clear);
    return rc;
}


int
sx1509_get_polarity(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_POLARITY, data);
    return rc;
}

int
sx1509_modify_polarity(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_POLARITY, set, clear);
    return rc;
}


int
sx1509_get_direction(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_DIR, data);
    return rc;
}

int
sx1509_modify_direction(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_DIR, set, clear);
    return rc;
}

int
sx1509_get_interrupt_mask(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_INTERRUPTMASK, data);
    return rc;
}

int
sx1509_modify_interrupt_mask(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_INTERRUPTMASK, set, clear);
    return rc;
}

int
sx1509_get_interrupt_source(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_INTERRUPTSRC, data);
    return rc;
}

int
sx1509_modify_interrupt_source(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_INTERRUPTSRC, set, clear);
    return rc;
}

int
sx1509_get_event_status(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_EVENTSTATUS, data);
    return rc;
}

int
sx1509_modify_event_status(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_EVENTSTATUS, set, clear);
    return rc;
}

int
sx1509_get_level_shifter(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_LEVELSHIFTER, data);
    return rc;
}

int
sx1509_modify_level_shifter(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_LEVELSHIFTER, set, clear);
    return rc;
}

int
sx1509_get_led_driver_enabled(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_LEDDRIVERENABLE, data);
    return rc;
}

int
sx1509_modify_led_driver_enabled(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_LEDDRIVERENABLE, set, clear);
    return rc;
}

int
sx1509_get_debounce_enabled(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_DEBOUNCEENABLE, data);
    return rc;
}

int
sx1509_modify_debounce_enabled(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_DEBOUNCEENABLE, set, clear);
    return rc;
}


int
sx1509_get_high_input(struct sx1509 *sx1509, uint16_t * data)
{
    assert(data != NULL);

    int rc = sx1509_read16(sx1509, SX1509_REG_HIGHINPUT, data);
    return rc;
}

int
sx1509_modify_high_input(struct sx1509 *sx1509, uint16_t set, uint16_t clear)
{
    int rc = sx1509_modify16(sx1509, SX1509_REG_HIGHINPUT, set, clear);
    return rc;
}


int
sx1509_get_intensity(struct sx1509 *sx1509, uint8_t pin, uint8_t * intensity)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT) {
        rc = sx1509_read8(sx1509, reg_intensity[pin], intensity);
    }

    return rc;
}


int
sx1509_set_intensity(struct sx1509 *sx1509, uint8_t pin, uint8_t intensity)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT) {
        rc = sx1509_write8(sx1509, reg_intensity[pin], intensity);
    }

    return rc;
}


int
sx1509_set_time_on(struct sx1509 *sx1509, uint8_t pin, uint8_t on_time)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT && on_time <= 0b00011111) {
        rc = sx1509_write8(sx1509, reg_time_on[pin], on_time);
    }

    return rc;
}


int
sx1509_set_time_off(struct sx1509 *sx1509, uint8_t pin, uint8_t off_time, uint8_t off_intensity)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT && off_time <= 0b00011111 && off_intensity <= 0b00000111) {
        uint8_t value = (off_time << 3) | off_intensity;
        rc = sx1509_write8(sx1509, reg_time_off[pin], value);
    }

    return rc;
}


int
sx1509_set_rise_time(struct sx1509 *sx1509, uint8_t pin, uint8_t rise_time)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT && sx1509_support_breathe(pin)
        && rise_time <= 0b00011111) {
        rc = sx1509_write8(sx1509, reg_rise_time[pin], rise_time);
    }

    return rc;
}

int
sx1509_set_fall_time(struct sx1509 *sx1509, uint8_t pin, uint8_t fall_time)
{
    int rc = OS_EINVAL;
    if (pin <= SX1509_PIN_COUNT && sx1509_support_breathe(pin)
        && fall_time <= 0b00011111) {
        rc = sx1509_write8(sx1509, reg_fall_time[pin], fall_time);
    }

    return rc;
}


int
sx1509_set_misc(struct sx1509 *sx1509,
                int bankA, int bankB, uint8_t freq, int nreset, int auto_incr, int auto_clear)
{
    int rc = OS_EINVAL;
    if (freq <= 0b00000111) {  /* 3 bit value, 0-7 */

        uint8_t value = 0;
        value |= ((bankA ? 1 : 0) << 7);
        value |= (freq << 4);
        value |= ((bankB ? 1 : 0) << 3);
        value |= ((nreset ? 1 : 0) << 2);
        value |= ((auto_incr ? 1 : 0) << 1);
        value |= ((auto_clear ? 1 : 0));

        rc = sx1509_write8(sx1509, SX1509_REG_MISC, value);
    }

    return rc;
}



int
sx1509_set_debounce_time(struct sx1509 *sx1509, uint8_t debounce_time)
{
    int rc = OS_EINVAL;
    if (debounce_time <= 0b00000111) { /* 0-7 */
        rc = sx1509_write8(sx1509, SX1509_REG_DEBOUNCECFG, debounce_time);
    }

    return rc;
}

int
sx1509_get_debounce_time(struct sx1509 *sx1509, uint8_t * debounce_time)
{
    assert(debounce_time != NULL);

    int rc = sx1509_read8(sx1509, SX1509_REG_DEBOUNCECFG, debounce_time);
    return rc;
}


int
sx1509_set_output(struct sx1509 *sx1509, uint8_t pin, uint8_t state)
{
    int rc;
    if (state) {
        rc = sx1509_modify_data(sx1509, 1 << pin, 0);
    }
    else {
        rc = sx1509_modify_data(sx1509, 0, 1 << pin);
    }
    return rc;
}

int
sx1509_set_output_level(struct sx1509 *sx1509, uint8_t pin, uint8_t level)
{
    int rc;
    if (level == 0) {
        rc = sx1509_set_output(sx1509, pin, 0);
    }
    else {
        rc = sx1509_set_output(sx1509, pin, 1);
        assert(rc == 0);
        rc = sx1509_set_intensity(sx1509, pin, level);
    }
    return rc;
}


int
sx1509_get_output_level(struct sx1509 *sx1509, uint8_t pin, uint8_t * level)
{
    int rc = OS_EINVAL;
    uint16_t data;

    if (pin <= SX1509_PIN_COUNT && level != NULL) {
        rc = sx1509_get_data(sx1509, &data);
        assert(rc == 0);

        if (data & (1 << pin)) {
            rc = sx1509_get_intensity(sx1509, pin, level);
            assert(rc == 0);
            if (*level == 0) {
                *level = 1;
            }
        }
    }

    return rc;
}


/**
 * Indicate whether a pin supports breathing (or not)
 *
 * @param pin SX1509 pin to check
 *
 * @return 1 if breathing support, 0 otherwise
 */
int
sx1509_support_breathe(uint8_t pin)
{
    int rc = 0;
    if (pin <= SX1509_PIN_COUNT && reg_fall_time[pin] != 0) {
        rc = 1;
    }
    return rc;

}



int
sx1509_set_sense(struct sx1509 *sx1509, uint8_t pin, enum sx1509_edge_sense edge_sense)
{
    int rc = OS_EINVAL;
    int base_reg = SX1509_REG_SENSE;

    int shift;

    if (pin <= SX1509_PIN_COUNT) {
        if (pin < 8) {
            shift = pin;
            base_reg += 2;
        }
        else {
            shift = (pin - 8);
        }
        uint16_t set_mask = (edge_sense << (shift * 2));
        uint16_t clr_mask = ((~edge_sense) << (shift * 2));

        rc = sx1509_modify16(sx1509, base_reg, set_mask, clr_mask);
        assert(rc == 0);

        uint16_t verify;
        rc = sx1509_read16(sx1509, base_reg, &verify);
    }

    return rc;
}
