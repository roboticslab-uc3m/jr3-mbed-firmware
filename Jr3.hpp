#ifndef __JR3_HPP__
#define __JR3_HPP__

#include "mbed.h"
#include "LPC17xx.h"

// reimplemented from: https://github.com/ARMmbed/mbed-os/blob/f9c0cd2/targets/TARGET_NXP/TARGET_LPC176X/port_api.c
// inspiration: https://os.mbed.com/users/igorsk/notebook/fast-gpio-with-c-templates/

template <PortName portName, PinName clockPin, PinName dataPin>
class Jr3
{
public:
    Jr3();
    uint32_t readFrame();

private:
    enum pin_state
    {
        DATA_LOW_CLOCK_LOW = 0,
        DATA_LOW_CLOCK_HIGH = 1UL << ((clockPin - P0_0) % 32),
        DATA_HIGH_CLOCK_LOW = 1UL << ((dataPin - P0_0) % 32),
        // we can use the next one as the pin mask for port-related stuff
        DATA_HIGH_CLOCK_HIGH = (1UL << ((clockPin - P0_0) % 32)) | (1UL << ((dataPin - P0_0) % 32))
    };

    void awaitNextFrame();
    pin_state readPins();
    bool readClock();
    bool readData();

    volatile uint32_t * port_in;

    static const unsigned int FRAME_SIZE = 20;
};

template <PortName portName, PinName clockPin, PinName dataPin>
inline Jr3<portName, clockPin, dataPin>::Jr3()
{
    LPC_GPIO_TypeDef * port_reg = reinterpret_cast<LPC_GPIO_TypeDef *>(LPC_GPIO0_BASE + ((int)portName * 0x20));

    for (int i = 0; i < 32; i++)
    {
        if (DATA_HIGH_CLOCK_HIGH & (1 << i))
        {
            gpio_set(port_pin(portName, i));
        }
    }

    port_reg->FIODIR &= ~DATA_HIGH_CLOCK_HIGH; // input
    port_in = &port_reg->FIOPIN;
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline uint32_t Jr3<portName, clockPin, dataPin>::readFrame()
{
    pin_state pins;
    uint32_t frame = 0;

    awaitNextFrame();

    for (int i = FRAME_SIZE - 1; i >= 0; i--)
    {
        // await end of previous bit, if necessary
        while (readClock()) {}

        // await rising edge on clock signal
        while (((pins = readPins()) & DATA_LOW_CLOCK_HIGH) == 0) {}

        if ((pins & DATA_HIGH_CLOCK_LOW) == DATA_HIGH_CLOCK_LOW)
        {
            frame |= (1U << i);
        }
    }

    return frame;
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline void Jr3<portName, clockPin, dataPin>::awaitNextFrame()
{
    pin_state pins;

    while (true)
    {
        // await next interval between frames
        while (readPins() != DATA_HIGH_CLOCK_HIGH) {}

        // await beginning of start pulse
        while ((pins = readPins()) == DATA_HIGH_CLOCK_HIGH) {}

        if (pins != DATA_LOW_CLOCK_HIGH)
        {
            continue; // this is not a start pulse, retry
        }

        // await rising edge of start pulse
        while ((pins = readPins()) == DATA_LOW_CLOCK_HIGH) {}

        if (pins != DATA_HIGH_CLOCK_HIGH)
        {
            continue; // this is not a start pulse, retry
        }

        break; // start pulse completed, we can start processing a new frame
    }
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline Jr3<portName, clockPin, dataPin>::pin_state Jr3<portName, clockPin, dataPin>::readPins()
{
    return static_cast<pin_state>(*port_in & DATA_HIGH_CLOCK_HIGH);
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline bool Jr3<portName, clockPin, dataPin>::readClock()
{
    return static_cast<pin_state>(*port_in & DATA_LOW_CLOCK_HIGH);
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline bool Jr3<portName, clockPin, dataPin>::readData()
{
    return static_cast<pin_state>(*port_in & DATA_HIGH_CLOCK_LOW);
}

#endif // __JR3_HPP__
