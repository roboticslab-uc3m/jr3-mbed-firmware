#ifdef TARGET_KLXX

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PORT_BASE       ((FGPIO_Type *)(FPTA_BASE + ((unsigned int)pin >> PORT_SHIFT) * 0x40))
#define PINMASK         (1 << ((pin & 0x7F) >> 2))
#define PCR             ((__IO uint32_t*)(PORTA_BASE + pin))

#define INIT_PIN        container.mask = PINMASK; pin_function(pin, 1)
#define DESTROY_PIN

#define SET_DIR_INPUT   (PORT_BASE->PDDR &= ~PINMASK)
#define SET_DIR_OUTPUT  (PORT_BASE->PDDR |= PINMASK)
#define SET_MODE(pull)  (*PCR = (*PCR & ~0x3) | pull)

#define WRITE_PIN_SET   (PORT_BASE->PSOR |= PINMASK)
#define WRITE_PIN_CLR   (PORT_BASE->PCOR |= PINMASK)

#define READ_PIN        ((PORT_BASE->PDIR & container.mask) ? 1 : 0)

#endif
