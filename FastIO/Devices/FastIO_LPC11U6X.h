#ifdef TARGET_LPC11U6X

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PORT            ((unsigned int)pin >> PORT_SHIFT)
#define PINMASK         ( 1 << ((int)pin >> PIN_SHIFT & 0x1F) )
static inline void initpin(PinName pin);

#define INIT_PIN        container.mask = PINMASK; initpin(pin)
#define DESTROY_PIN

#define SET_DIR_INPUT   (LPC_GPIO_PORT->DIR[PORT] &= ~PINMASK)
#define SET_DIR_OUTPUT  (LPC_GPIO_PORT->DIR[PORT] |= PINMASK)
#define SET_MODE(pull)  (pin_mode(pin, pull))

#define WRITE_PIN_SET   (LPC_GPIO_PORT->SET[PORT] = PINMASK)
#define WRITE_PIN_CLR   (LPC_GPIO_PORT->CLR[PORT] = PINMASK)

#define READ_PIN        ((LPC_GPIO_PORT->PIN[PORT] & container.mask) ? 1 : 0)

static inline void initpin(PinName pin) {
    int f =  ((pin == P0_0)  ||
              (pin == P0_10) ||
              (pin == P0_11) ||
              (pin == P0_12) ||
              (pin == P0_13) ||
              (pin == P0_14) ||
              (pin == P0_15)) ? (1) : (0);
    pin_function(pin, f);
}

#endif
