#if defined(TARGET_EFM32)

#include "mbed.h"
#include "pinmap.h"
#include "em_cmu.h"

typedef struct {
    uint32_t mask;
    uint8_t input_mode;
    uint8_t output_mode;
} fastio_vars;

#define PIN_INDEX       (pin & 0xF)
#define PINMASK         (1 << PIN_INDEX)
#define PORT_INDEX      (pin >> 4)

//Mode_reg is either high or low, depending on first 8 bit or second 8-bit of a port
#define MODE_REG        (*((&GPIO->P[PORT_INDEX].MODEL) + PIN_INDEX / 8))
#define MODE_SHIFT      ((PIN_INDEX * 4) % 32)

#define INIT_PIN        this->container.mask = PINMASK; this->container.input_mode = PullDefault; this->container.output_mode = PushPull; CMU_ClockEnable(cmuClock_HFPER, true); CMU_ClockEnable(cmuClock_GPIO, true)
#define DESTROY_PIN

#define SET_DIR_INPUT   uint32_t temp = MODE_REG & ~(0xF << MODE_SHIFT); MODE_REG = temp + ((this->container.input_mode & 0xF) << MODE_SHIFT); if (this->container.input_mode > 0x10) WRITE_PIN_SET; else WRITE_PIN_CLR
#define SET_DIR_OUTPUT  uint32_t temp = MODE_REG & ~(0xF << MODE_SHIFT); MODE_REG = temp + (container.output_mode << MODE_SHIFT)
#define SET_MODE(pull)  if ((pull <= 3) || (pull > 0x10)) {this->container.input_mode = pull; SET_DIR_INPUT; } else {this->container.output_mode = pull; SET_DIR_OUTPUT;}

#define WRITE_PIN_SET   GPIO->P[PORT_INDEX].DOUTSET = PINMASK
#define WRITE_PIN_CLR   GPIO->P[PORT_INDEX].DOUTCLR = PINMASK

#define READ_PIN        ((GPIO->P[PORT_INDEX].DIN & this->container.mask) ? 1 : 0)

#endif
