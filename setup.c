#include "os/mynewt.h"
#include "hal/hal_gpio.h"
#include "sx1509/sx1509.h"

#include "buttons.h"
#include "bsp/bsp.h"


struct DEMO_Light g_led[MYNEWT_VAL(DEMO_THROTTLE_LIGHT_COUNT)] = {
    { .dev = "gpio",     .pin = LED_1, .label = "Red LED",  .color="r", .activelow = false},
    { .dev = "gpio",     .pin = LED_2, .label = "Blue LED", .color="b", .activelow = false},
    { .dev = "sx1509_0", .pin = 15,    .label = "Light 1",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 14,    .label = "Light 2",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 0,     .label = "Light 3",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 1,     .label = "Light 4",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 2,     .label = "Light 5",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 3,     .label = "Light 6",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 4,     .label = "Light 7",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 5,     .label = "Light 8",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 6,     .label = "Light 9",  .color="G", .activelow = false},
    { .dev = "sx1509_0", .pin = 7,     .label = "Light 10", .color="G", .activelow = false},
};


struct DEMO_Button g_button[MYNEWT_VAL(DEMO_THROTTLE_BUTTON_COUNT)] = {
    { .dev = "gpio",     .pin = BUTTON_1, .label = "DFU" },
    { .dev = "sx1509_0", .pin = 13,       .label = "Brake" },
    { .dev = "sx1509_0", .pin = 11,       .label = "Button 1" },
    { .dev = "sx1509_0", .pin = 10,       .label = "Button 2" },
    { .dev = "sx1509_0", .pin =  9,       .label = "Button 3" },
    { .dev = "sx1509_0", .pin =  8,       .label = "Button 4" },
};


struct sx1509 sx1509_0;

struct sx1509_cfg sx1509_0_cfg = {
    .i2c_bus_num = 0,
    .i2c_address = 0x3E,
    .clock_source = sx1509_clock_internal,
    .oscio = sx1509_oscio_output,
    .misc_reg = 0x40,
    .debounce_time = 0b0101,  // 16ms
    .io_count = 16,  // initialize everything, even if NC
    .io = {
        { // OUTPUT
          .pin = 0,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 1,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 2,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 3,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 4,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 5,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 6,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin = 7,
          .value = 0,
          .intensity = 0xff,
          .fade_in = 3,
          .fade_out = 2,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_BREATHE |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // INPUT
            .pin         = 8,
            .edge_sense  = sx1509_edge_both,
            .flags_set   = SX1509_PIN_INPUT |
                           SX1509_PIN_PULL_UP_ENABLE |
                           SX1509_PIN_INVERT_POLARITY |
                           SX1509_PIN_DEBOUNCE_ENABLE,
            .flags_clear = SX1509_PIN_INTERRUPT_MASK |
                           SX1509_PIN_INPUT_DISABLE,
        },
        { // INPUT
            .pin         = 9,
            .edge_sense  = sx1509_edge_both,
            .flags_set   = SX1509_PIN_INPUT |
                           SX1509_PIN_PULL_UP_ENABLE |
                           SX1509_PIN_INVERT_POLARITY |
                           SX1509_PIN_DEBOUNCE_ENABLE,
            .flags_clear = SX1509_PIN_INTERRUPT_MASK |
                           SX1509_PIN_INPUT_DISABLE,
        },
        { // INPUT
            .pin         = 10,
            .edge_sense  = sx1509_edge_both,
            .flags_set   = SX1509_PIN_INPUT |
                           SX1509_PIN_PULL_UP_ENABLE |
                           SX1509_PIN_INVERT_POLARITY |
                           SX1509_PIN_DEBOUNCE_ENABLE,
            .flags_clear = SX1509_PIN_INTERRUPT_MASK |
                           SX1509_PIN_INPUT_DISABLE,
        },
        { // INPUT
            .pin         = 11,
            .edge_sense  = sx1509_edge_both,
            .flags_set   = SX1509_PIN_INPUT |
                           SX1509_PIN_PULL_UP_ENABLE |
                           SX1509_PIN_INVERT_POLARITY |
                           SX1509_PIN_DEBOUNCE_ENABLE,
            .flags_clear = SX1509_PIN_INTERRUPT_MASK |
                           SX1509_PIN_INPUT_DISABLE,
        },
        { // this pin is not used
            .pin         = 12,
            .flags_set   = SX1509_PIN_INPUT_DISABLE,
        },
        { // INPUT
            .pin         = 13,
            .edge_sense  = sx1509_edge_both,
            .flags_set   = SX1509_PIN_INPUT |
                           SX1509_PIN_PULL_UP_ENABLE |
                           SX1509_PIN_INVERT_POLARITY |
                           SX1509_PIN_DEBOUNCE_ENABLE,
            .flags_clear = SX1509_PIN_INTERRUPT_MASK |
                           SX1509_PIN_INPUT_DISABLE,
        },
        { // OUTPUT
          .pin = 14,
          .value = 0,
          .intensity = 0xff,
          .flags_set = SX1509_PIN_INPUT_DISABLE |
                       SX1509_PIN_OPENDRAIN |
                       SX1509_PIN_INVERT_POLARITY |
                       SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },
        { // OUTPUT
          .pin         = 15,
          .value       = 0,
          .intensity   = 0xff,
          .fade_in     = 3,
          .fade_out    = 3,
          .flags_set   = SX1509_PIN_INPUT_DISABLE |
                         SX1509_PIN_OPENDRAIN |
                         SX1509_PIN_INVERT_POLARITY |
                         SX1509_PIN_LED_DRIVER_ENABLE,
          .flags_clear = SX1509_PIN_INPUT
        },

    }
};


static struct os_event gpio_intr_event;

static struct os_event sx_intr_event;

static void
sx_event_handler(struct os_event *ev)
{
    char *sx1509_dev_name = (char *) ev->ev_arg;
    struct os_dev *dev = os_dev_open(sx1509_dev_name, OS_TIMEOUT_NEVER, NULL);
    if (dev != NULL) {
        struct sx1509 *sx1509 = (struct sx1509 *) dev;

        uint16_t intr_source;
        uint16_t io_data;
        int rc;

        rc = sx1509_get_interrupt_source(sx1509, &intr_source);
        assert(rc==0);

        rc = sx1509_get_data(sx1509, &io_data);
        assert(rc==0);

        for (int i = 0; i < 16; i++) {
            if (intr_source & (1 << i)) {
                for (int b = 0; b < MYNEWT_VAL(DEMO_THROTTLE_BUTTON_COUNT); b++) {
                    if (strcmp(g_button[b].dev, sx1509_dev_name) == 0 && (g_button[b].pin == i)) {
                        demo_button_read(b);
                        break;
                    }
                }
            }
        }
    }
}




static void
sx_irq_handler(void *arg)
{
    sx_intr_event.ev_arg = arg;
    sx_intr_event.ev_cb = sx_event_handler;
    os_eventq_put(os_eventq_dflt_get(), &sx_intr_event);
}





static void
gpio_event_handler(struct os_event *ev)
{
    uint16_t idx = ((int) (ev->ev_arg)) & 0xFFFF;
    demo_button_read(idx);
}

static void
gpio_irq_handler(void *arg)
{
    gpio_intr_event.ev_arg = arg;
    gpio_intr_event.ev_cb = gpio_event_handler;
    os_eventq_put(os_eventq_dflt_get(), &gpio_intr_event);
}



void
init_buttons()
{
    for (int i=0; i < MYNEWT_VAL(DEMO_THROTTLE_BUTTON_COUNT); i++) {
        if (strcmp(g_button[i].dev, "gpio") == 0) {
            hal_gpio_irq_init(g_button[i].pin,
                              gpio_irq_handler,
                              (void*) i,
                              HAL_GPIO_TRIG_BOTH,
                              HAL_GPIO_PULL_UP);
            hal_gpio_irq_enable(g_button[i].pin);
        }
        else {
            // SX1509 buttons configured via init_sx1509 device creation
        }

        demo_button_read(i);  // prime the values in the Throttle service
    }
}



void
init_lights()
{
    for (int i=0; i < MYNEWT_VAL(DEMO_THROTTLE_LIGHT_COUNT); i++) {
        hal_gpio_init_out(g_led[i].pin, g_led[i].activelow);
    }
}




void
init_sx1509()
{
    int rc;

    rc = os_dev_create((struct os_dev *) &sx1509_0, "sx1509_0",
                       OS_DEV_INIT_PRIMARY, 0, sx1509_init, (void *)&sx1509_0_cfg);
    assert(rc == 0);

    hal_gpio_irq_init(SX1509_INTR,
                      sx_irq_handler,  // callback function
                      "sx1509_0",      // argument to callback
                      HAL_GPIO_TRIG_FALLING,
                      HAL_GPIO_PULL_NONE);
    hal_gpio_irq_enable(SX1509_INTR);
}







void
demo_button_read(uint16_t idx)
{
    Button_State state = BUTTON_UNKNOWN;
    if (strcmp(g_button[idx].dev, "gpio") == 0) {
        int value = hal_gpio_read(g_button[idx].pin);
        state = (value == 0) ? BUTTON_PRESSED : BUTTON_RELEASED;
    }
    else if (strncmp(g_button[idx].dev, "sx1509_", 6) == 0) {
        struct os_dev *dev = os_dev_open("sx1509_0", OS_TIMEOUT_NEVER, NULL);
        if (dev != NULL) {
            struct sx1509 *sx1509 = (struct sx1509 *) dev;
            uint16_t data;

            int rc = sx1509_get_data(sx1509, &data);
            assert(rc==0);
            state = (data & (1 << g_button[idx].pin)) ? BUTTON_PRESSED : BUTTON_RELEASED;
        }
    }
    ble_svc_throttle_button_set(idx, state);
}



static void
change_light(char *devname, uint8_t idx, uint8_t led_on)
{
    struct os_dev *dev = os_dev_open(devname, OS_TIMEOUT_NEVER, NULL);
    if (dev != NULL) {
        struct sx1509 *sx1509 = (struct sx1509 *) dev;
        sx1509_set_output_level(sx1509, idx, led_on);
    }
}



void
demo_light_write(uint16_t idx, uint32_t value)
{
    if (strcmp(g_led[idx].dev, "gpio") == 0) {
        hal_gpio_write(g_led[idx].pin, value == g_led[idx].activelow ? 0 : 1);
    }
    else if (strncmp(g_led[idx].dev, "sx1509_", 6) == 0) {
        change_light(g_led[idx].dev, g_led[idx].pin, value & 0xFF);
    }
}



int
demo_light_read(uint16_t idx, uint32_t *value)
{
    int rc = 0;

    if (value != NULL) {

        if (strcmp(g_led[idx].dev, "gpio") == 0) {
            *value = (hal_gpio_read(g_led[idx].pin) != g_led[idx].activelow);
            rc = 1;
        }
        else if (strncmp(g_led[idx].dev, "sx1509_", 6) == 0) {
            uint8_t v;
            struct os_dev *dev = os_dev_open(g_led[idx].dev, OS_TIMEOUT_NEVER, NULL);
            if (dev != NULL) {
                struct sx1509 *sx1509 = (struct sx1509 *) dev;
                rc = sx1509_get_output_level(sx1509, g_led[idx].pin, &v);
                assert(rc==0);

                *value = v;
                rc = 1;
            }
        }
    }

    return rc;
}
