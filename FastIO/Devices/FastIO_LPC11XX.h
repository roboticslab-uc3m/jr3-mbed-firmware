#ifdef TARGET_LPC11XX_11CXX

#include "mbed.h"
#include "pinmap.h"
#include "reserved_pins.h"

typedef struct {
} fastio_vars;

#define PORT            ((LPC_GPIO_TypeDef *) (LPC_GPIO0_BASE + (((pin & 0xF000) >> PORT_SHIFT) * 0x10000)))
#define PIN             ((pin & 0x0F00) >> 8 )
#define MASKDATA        (*((volatile uint32_t*) ((uint32_t)PORT + (1 << (PIN + 2)))))
static inline void initpin(PinName pin);
static const PinName reserved_pins[] = TARGET_RESERVED_PINS;

#define INIT_PIN        initpin(pin)
#define DESTROY_PIN

#define SET_DIR_INPUT   (PORT->DIR &= ~(1 << PIN))
#define SET_DIR_OUTPUT  (PORT->DIR |= (1<<PIN))
#define SET_MODE(pull)  (pin_mode(pin, pull))

#define WRITE_PIN_SET   (MASKDATA = 0xFFFF)
#define WRITE_PIN_CLR   (MASKDATA = 0)

#define READ_PIN        ((MASKDATA == 0) ? 0 : 1)

static inline void initpin(PinName pin) {
    unsigned i;
    int f = 0;

    for (i = 0; i < sizeof(reserved_pins) / sizeof(PinName); i ++) {
        if (pin == reserved_pins[i]) {
            f = 1;
            break;
        }
    }
    pin_function(pin, f);
}

#endif
