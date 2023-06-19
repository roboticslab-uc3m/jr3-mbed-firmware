#ifdef TARGET_LPC1768

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define LPC_GPIO            ((LPC_GPIO_TypeDef*)(pin & ~0x1F))
#define PINMASK             (1UL << (((pin) - P0_0)%32))

#define PINSELREG           (*(volatile uint32_t*)(LPC_PINCON_BASE + 4*(((pin) - P0_0)/16)))
#define PINSELMASK          (0x03 << (((pin - P0_0)%16)*2) )

#define INIT_PIN            container.mask = PINMASK; (PINSELREG &= ~PINSELMASK)
#define DESTROY_PIN

#define SET_DIR_INPUT       (LPC_GPIO->FIODIR &= ~PINMASK)
#define SET_DIR_OUTPUT      (LPC_GPIO->FIODIR |= PINMASK)
#define SET_MODE(pull)      (pin_mode(pin, pull))

#define WRITE_PIN_SET       (LPC_GPIO->FIOSET = PINMASK)
#define WRITE_PIN_CLR       (LPC_GPIO->FIOCLR = PINMASK)

#define READ_PIN            ((LPC_GPIO->FIOPIN & container.mask) ? 1 : 0)

#endif
