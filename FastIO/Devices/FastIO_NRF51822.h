#ifdef TARGET_MCU_NRF51822

#include "mbed.h"
#include "gpio_api.h"

typedef struct {
    uint32_t mask;
} fastio_vars;

#define PINMASK         (1 << pin)


#define INIT_PIN        container.mask = PINMASK; NRF_GPIO->PIN_CNF[pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
                                        | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) \
                                        | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) \
                                        | (NRF_GPIO->PIN_CNF[pin] & 0x01);
#define DESTROY_PIN

#define SET_DIR_INPUT   NRF_GPIO->DIRCLR = PINMASK
#define SET_DIR_OUTPUT  NRF_GPIO->DIRSET = PINMASK
#define SET_MODE(pull)  NRF_GPIO->PIN_CNF[pin] = (NRF_GPIO->PIN_CNF[pin] & (~GPIO_PIN_CNF_PULL_Msk)) | pull

#define WRITE_PIN_SET   NRF_GPIO->OUTSET = PINMASK
#define WRITE_PIN_CLR   NRF_GPIO->OUTCLR = PINMASK

#define READ_PIN        ((NRF_GPIO->IN & container.mask) ? 1 : 0)

#endif
