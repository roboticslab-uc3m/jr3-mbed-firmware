#ifdef TARGET_LPC11UXX

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PORT            ((unsigned int)pin >> PORT_SHIFT)
#define PINMASK         (1 << ((int)pin & 0x1F))
static inline void initpin(PinName pin);

#define INIT_PIN        container.mask = PINMASK; initpin(pin)
#define DESTROY_PIN

#define SET_DIR_INPUT   (LPC_GPIO->DIR[PORT] &= ~PINMASK)
#define SET_DIR_OUTPUT  (LPC_GPIO->DIR[PORT] |= PINMASK)
#define SET_MODE(pull)  (pin_mode(pin, pull))

#define WRITE_PIN_SET   (LPC_GPIO->SET[PORT] = PINMASK)
#define WRITE_PIN_CLR   (LPC_GPIO->CLR[PORT] = PINMASK)

#define READ_PIN        ((LPC_GPIO->PIN[PORT] & container.mask) ? 1 : 0)

static inline void initpin(PinName pin) {
    int f = ((pin == P0_0)  ||
    (pin == P0_10) ||
    (pin == P0_11) ||
    (pin == P0_12) ||
    (pin == P0_13) ||
    (pin == P0_14) ||
    (pin == P0_15)) ? (1) : (0);

    pin_function(pin, f);
}

#endif
