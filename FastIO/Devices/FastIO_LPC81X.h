#ifdef TARGET_LPC81X

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    __I  uint32_t *reg_in;
    uint32_t mask;
} fastio_vars;

#define PINMASK         (1 << ((int)pin & 0x1F))
static void gpio_enable(void);

#define INIT_PIN        container.mask = PINMASK; container.reg_in = &LPC_GPIO_PORT->PIN0; gpio_enable(); pin_function(pin, 0)
#define DESTROY_PIN

#define SET_DIR_INPUT   (LPC_GPIO_PORT->DIR0 &= ~PINMASK)
#define SET_DIR_OUTPUT  (LPC_GPIO_PORT->DIR0 |= PINMASK)
#define SET_MODE(pull)  (pin_mode(pin, pull))

#define WRITE_PIN_SET   (LPC_GPIO_PORT->SET0 = PINMASK)
#define WRITE_PIN_CLR   (LPC_GPIO_PORT->CLR0 = PINMASK)

#define READ_PIN        ((*container.reg_in & container.mask) ? 1 : 0)

static int  gpio_enabled = 0;
static void gpio_enable(void)
{
    if (!gpio_enabled) {
        gpio_enabled = 1;

        /* Enable AHB clock to the GPIO domain. */
        LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

        /* Peripheral reset control to GPIO and GPIO INT, a "1" bring it out of reset. */
        LPC_SYSCON->PRESETCTRL &= ~(0x1<<10);
        LPC_SYSCON->PRESETCTRL |=  (0x1<<10);
    }
}

#endif
