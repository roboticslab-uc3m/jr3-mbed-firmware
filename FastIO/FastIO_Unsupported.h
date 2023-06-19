#include "mbed.h"

typedef struct {
    DigitalInOut *_pin;
} fastio_vars;

#define INIT_PIN        this->container._pin = new DigitalInOut(pin)
#define DESTROY_PIN     delete(this->container._pin)

#define SET_DIR_INPUT   this->container._pin->input()
#define SET_DIR_OUTPUT  this->container._pin->output()
#define SET_MODE(pull)  this->container._pin->mode(pull)

#define WRITE_PIN_SET   this->container._pin->write(1)
#define WRITE_PIN_CLR   this->container._pin->write(0)

#define READ_PIN        this->container._pin->read()
