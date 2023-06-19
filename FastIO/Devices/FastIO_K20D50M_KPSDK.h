#if defined(TARGET_K20D50M) | defined(TARGET_KPSDK_MCUS)

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

//K20D50M and KPSDK use slightly different pinnames, shifted 2
#ifdef TARGET_KPSDK_MCUS
#define PORT_SHIFT      GPIO_PORT_SHIFT
#define PINMASK         (1 << (pin & 0x1F))
#define PCR             ((__IO uint32_t*)(PORTA_BASE + ((pin & 0x1F) << 2) + ((pin >> PORT_SHIFT) << PORT_SHIFT)  ))
#else
#define PINMASK         (1 << ((pin & 0x7F) >> 2))
#define PCR             ((__IO uint32_t*)(PORTA_BASE + pin))
#endif

#define PORT_BASE       ((GPIO_Type *)(PTA_BASE + ((unsigned int)pin >> PORT_SHIFT) * 0x40))

#define INIT_PIN        container.mask = PINMASK; pin_function(pin, 1)
#define DESTROY_PIN

#define SET_DIR_INPUT   (PORT_BASE->PDDR &= ~PINMASK)
#define SET_DIR_OUTPUT  (PORT_BASE->PDDR |= PINMASK)
#define SET_MODE(pull)  (*PCR = (*PCR & ~0x3) | pull)

#define WRITE_PIN_SET   (PORT_BASE->PSOR |= container.mask)
#define WRITE_PIN_CLR   (PORT_BASE->PCOR |= container.mask)

#define READ_PIN        ((PORT_BASE->PDIR & container.mask) ? 1 : 0)

#endif
