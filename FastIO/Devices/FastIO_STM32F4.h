#if defined(TARGET_STM32F4)

#include "mbed.h"
#include "pinmap.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PINMASK             (1 << STM_PIN(pin))
#define PINMASK_CLR         ((1<<16) << STM_PIN(pin))
#define PORT                ((GPIO_TypeDef *)(GPIOA_BASE + 0x0400 * STM_PORT(pin)))

#define INIT_PIN            RCC->AHB1ENR |= (1 << STM_PORT(pin)); (PORT->MODER &= ~(GPIO_MODER_MODER0_1 << (STM_PIN(pin) * 2))); container.mask = PINMASK
#define DESTROY_PIN

#define SET_DIR_INPUT       (PORT->MODER &= ~(GPIO_MODER_MODER0_0 << (STM_PIN(pin) * 2)))
#define SET_DIR_OUTPUT      (PORT->MODER |= (GPIO_MODER_MODER0_0 << (STM_PIN(pin) * 2)))
#define SET_MODE(pull)      pin_mode(pin, pull);

#define WRITE_PIN_SET       (PORT->BSRR = PINMASK)
#define WRITE_PIN_CLR       (PORT->BSRR = PINMASK_CLR)

#define READ_PIN            ((PORT->IDR & container.mask) ? 1 : 0)

#endif
