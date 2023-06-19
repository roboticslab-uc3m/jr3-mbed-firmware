#ifdef TARGET_LPC43XX

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PORT            (((unsigned int)pin >> PORT_SHIFT) & 0x0000000F)
#define PINMASK         (1 << ((int)pin & 0x1F))
static inline void initpin(PinName pin);

#define INIT_PIN        container.mask = PINMASK; initpin(pin)
#define DESTROY_PIN

#define SET_DIR_INPUT   ( ((LPC_GPIO_T *)(LPC_GPIO_PORT_BASE))->DIR[PORT] &= ~PINMASK )
#define SET_DIR_OUTPUT  ( ((LPC_GPIO_T *)(LPC_GPIO_PORT_BASE))->DIR[PORT] |= PINMASK )
#define SET_MODE(pull)  (pin_mode(pin, pull))

#define WRITE_PIN_SET   ( ((LPC_GPIO_T *)(LPC_GPIO_PORT_BASE))->SET[PORT] = PINMASK )
#define WRITE_PIN_CLR   ( ((LPC_GPIO_T *)(LPC_GPIO_PORT_BASE))->CLR[PORT] = PINMASK )

#define READ_PIN        ( (((LPC_GPIO_T *)(LPC_GPIO_PORT_BASE))->PIN[PORT] & container.mask) ? 1 : 0 )

static inline void initpin(PinName pin) {
    unsigned int port = (unsigned int)MBED_GPIO_PORT(pin);
    int f = SCU_PINIO_FAST | ((port > 4) ? (4) : (0));
    pin_function(pin, f);
}

#endif
