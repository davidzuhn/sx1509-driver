/*
 * Support for the Semtech SX150[789] series of GPIO expanders
 *
 * This is the CLI interpreter
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
#include <errno.h>

#include "os/mynewt.h"
#include "console/console.h"
#include "shell/shell.h"

#include "sx1509/sx1509.h"

#include "sensor/sensor.h"
#include "parse/parse.h"

#if MYNEWT_VAL(SX1509_CLI)

static int sx1509_shell_cmd(int argc, char **argv);

static struct shell_cmd sx1509_shell_cmd_struct = {
    .sc_cmd = "sx1509",
    .sc_cmd_func = sx1509_shell_cmd
};


static int
sx1509_shell_err_unknown_arg(char *cmd_name)
{
    console_printf("Error: unknown argument \"%s\"\n", cmd_name);
    return EINVAL;
}



static int
sx1509_shell_help(void)
{
    console_printf("%s cmd  [flags...]\n", sx1509_shell_cmd_struct.sc_cmd);
    console_printf("cmd:\n");
    console_printf("\tread [REGISTER]\n");
    console_printf("\treg dump\n");
    console_printf("\treg peek REG\n");
    console_printf("\treg poke REG VAL\n");
    console_printf("\treset\n");
    return 0;
}

static uint8_t
read_numeric_value(const char *s)
{
    char *endptr;
    uint8_t val = strtoumax(s, &endptr, 0);
    return val;
}


static int
sx1509_shell_reset(int argc, char **argv, struct sx1509 *sx1509)
{
    int rc;
    rc = sx1509_reset(sx1509);
    if (rc == 0) {
        console_printf("SX1509 has been reset\n");
    }

    return rc;
}



static int
sx1509_shell_clock(int argc, char **argv, struct sx1509 *sx1509)
{
    int rc = 0;

    // sx1509 clock set 2 1 14
    if (argc == 6 && strcmp(argv[2], "set") == 0) {
        enum sx1509_clock_source source = read_numeric_value(argv[3]);
        enum sx1509_oscio oscio = read_numeric_value(argv[4]);
        uint8_t frequency = read_numeric_value(argv[5]);

        rc = sx1509_set_clock(sx1509, source, oscio, frequency);
        console_printf("%s\n", (rc == 0) ? "ok" : "error");
    }
    else if (argc == 3 && strcmp(argv[2], "get") == 0) {
        enum sx1509_clock_source source;
        enum sx1509_oscio oscio;
        uint8_t frequency;

        rc = sx1509_get_clock(sx1509, &source, &oscio, &frequency);
        if (rc == 0) {
            console_printf("clock setting: source:%d oscio:%s frequency:%d\n",
                           source, oscio ? "output" : "input", frequency);
        }
    }
    else {
        rc = sx1509_shell_help();
    }
    return rc;
}


static int
sx1509_shell_reg(int argc, char **argv, struct sx1509 *sx1509)
{
    if (argc == 3 && strcmp(argv[2], "dump") == 0) {
        console_printf("sx1509 register values\n");
        for (int i = 0; i < SX1509_REGISTER_COUNT; i++) {
            uint8_t value;
            int rc = sx1509_read8(sx1509, i, &value);
            if (rc == 0) {
                console_printf("  0x%02x = 0x%02x\n", i, value);
            }
            else {
                return 1;
            }
        }

    }
    if (argc == 4 && strcmp(argv[2], "peek") == 0) {
        int reg = read_numeric_value(argv[3]);
        uint8_t value;
        int rc = sx1509_read8(sx1509, reg, &value);
        if (rc == 0) {
            console_printf("  register 0x%02x = 0x%02x\n", reg, value);
        }
        else {
            return -1;
        }
    }
    if (argc == 5 && strcmp(argv[2], "poke") == 0) {
        int reg = read_numeric_value(argv[3]);
        int value = read_numeric_value(argv[4]);

        int rc = sx1509_write8(sx1509, reg, value);
        if (rc == 0) {
            console_printf("  wrote register 0x%02x = 0x%02x\n", reg, value);
        }
        else {
            return -1;
        }
    }

    return 0;
}


static int
sx1509_shell_read(int argc, char **argv, struct sx1509 *sx1509)
{
    char *endptr;
    int name = 0;
    uint8_t reg = strtoumax(argv[2], &endptr, 0);
    uint16_t data;
    int rc;

    if (*endptr != '\0') {
        name = 1;
        if (strcmp(argv[2], "DATA") == 0) {
            rc = sx1509_get_data(sx1509, &data);
        }
        else if (strcmp(argv[2], "INPUTDISABLE") == 0) {
            rc = sx1509_get_input_disable(sx1509, &data);
        }
        else if (strcmp(argv[2], "LONGSLEW") == 0) {
            rc = sx1509_get_long_slew(sx1509, &data);
        }
        else if (strcmp(argv[2], "LOWDRIVE") == 0) {
            rc = sx1509_get_low_drive(sx1509, &data);
        }
        else if (strcmp(argv[2], "PULLUP") == 0) {
            rc = sx1509_get_pullup(sx1509, &data);
        }
        else if (strcmp(argv[2], "PULLDOWN") == 0) {
            rc = sx1509_get_pulldown(sx1509, &data);
        }
        else if (strcmp(argv[2], "OPENDRAIN") == 0) {
            rc = sx1509_get_opendrain(sx1509, &data);
        }
        else if (strcmp(argv[2], "POLARITY") == 0) {
            rc = sx1509_get_polarity(sx1509, &data);
        }
        else if (strcmp(argv[2], "DIRECTION") == 0) {
            rc = sx1509_get_direction(sx1509, &data);
        }
        else if (strcmp(argv[2], "INTMASK") == 0) {
            rc = sx1509_get_interrupt_mask(sx1509, &data);
        }
        else if (strcmp(argv[2], "INTSRC") == 0) {
            rc = sx1509_get_interrupt_source(sx1509, &data);
        }
        else if (strcmp(argv[2], "EVENTSTATUS") == 0) {
            rc = sx1509_get_event_status(sx1509, &data);
        }
        else if (strcmp(argv[2], "LEVELSHIFTER") == 0) {
            rc = sx1509_get_level_shifter(sx1509, &data);
        }
        else if (strcmp(argv[2], "LEDDRIVER") == 0) {
            rc = sx1509_get_led_driver_enabled(sx1509, &data);
        }
        else if (strcmp(argv[2], "DEBOUNCE") == 0) {
            rc = sx1509_get_debounce_enabled(sx1509, &data);
        }
        else {
            console_printf("unknown register name '%s'\n", argv[2]);
            rc = 1;
        }

    }
    else {
        rc = sx1509_read16(sx1509, reg, &data);
    }


    if (rc == 0) {
        if (name) {
            console_printf("  register %s is 0x%04x\n", argv[2], data);
        }
        else {
            console_printf("  register 0x%02x,0x%02x is 0x%04x\n", reg, reg + 1, data);
        }
    }

    return rc;
}


static int
sx1509_shell_cmd(int argc, char **argv)
{
    struct os_dev *dev;
    struct sx1509 *sx1509;

    dev = os_dev_open("sx1509_0", OS_TIMEOUT_NEVER, NULL);
    if (dev == NULL) {
        console_printf("failed to open sx1509_0 device\n");
        return ENODEV;
    }

    sx1509 = (struct sx1509 *) dev;

    if (argc == 1) {
        return sx1509_shell_help();
    }

    if (argc > 1 && strcmp(argv[1], "reg") == 0) {
        return sx1509_shell_reg(argc, argv, sx1509);
    }

    if (argc == 2 && strcmp(argv[1], "reset") == 0) {
        return sx1509_shell_reset(argc, argv, sx1509);
    }

    if (argc >= 2 && strcmp(argv[1], "clock") == 0) {
        return sx1509_shell_clock(argc, argv, sx1509);
    }

    if (argc == 3 && strcmp(argv[1], "read") == 0) {
        return sx1509_shell_read(argc, argv, sx1509);
    }

    return sx1509_shell_err_unknown_arg(argv[1]);
}


#endif                          /* SX1509_CLI */

int
sx1509_shell_init(void)
{
    int rc = 0;

#if MYNEWT_VAL(SX1509_CLI)
    rc = shell_cmd_register(&sx1509_shell_cmd_struct);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    return rc;
}
