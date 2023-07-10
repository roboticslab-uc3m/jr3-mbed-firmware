#ifndef __JR3_PORT_REGISTER_HPP__
#define __JR3_PORT_REGISTER_HPP__

#include "mbed.h"

// https://github.com/ARMmbed/mbed-os/blob/f9c0cd2a94fc1353946b321aad7f8bca2bfe1b40/targets/TARGET_NXP/TARGET_LPC176X/port_api.c
// https://os.mbed.com/users/igorsk/notebook/fast-gpio-with-c-templates/
// https://os.mbed.com/users/igorsk/code/FastIO//file/b8e40f2a0aac/FastIO.h/

template <PortName portName, PinName clockPin, PinName dataPin>
class JR3PortRegister
{
public:
    JR3PortRegister()
        // : port(portName, (1UL << ((clockPin - P0_0) % 32)) | (1UL << ((dataPin - P0_0) % 32))),
        //   DATA_LOW_CLOCK_LOW(0),
        //   DATA_LOW_CLOCK_HIGH(1UL << ((clockPin - P0_0) % 32)),
        //   DATA_HIGH_CLOCK_LOW(1UL << ((dataPin - P0_0) % 32)),
        //   DATA_HIGH_CLOCK_HIGH((1UL << ((clockPin - P0_0) % 32)) | (1UL << ((dataPin - P0_0) % 32))),
        //   CLOCK_LOW()
    {
        LPC_GPIO_TypeDef * port_reg = (LPC_GPIO_TypeDef *)(LPC_GPIO0_BASE + ((int)portName * 0x20));

        gpio_set(port_pin(portName, 0));
        gpio_set(port_pin(portName, 1));

        port_reg->FIODIR &= ~0x00000003;

        port_in = &port_reg->FIOPIN;

        // for (i = 0; i < 32; i++) {
        //     if (obj->mask & (1 << i))
        //     {
        //         gpio_set(port_pin(portName, i));
        //     }
        // }
    }

    uint32_t readFrame();

private:
    void awaitNextFrame();

    volatile uint32_t * port_in;

    // const int DATA_LOW_CLOCK_LOW;
    // const int DATA_LOW_CLOCK_HIGH;
    // const int DATA_HIGH_CLOCK_LOW;
    // const int DATA_HIGH_CLOCK_HIGH;

    // const int CLOCK_LOW;
    // const int CLOCK_HIGH;
    // const int DATA_LOW;
    // const int DATA_HIGH;

    static const unsigned int FRAME_SIZE = 20;
};

template <PortName portName, PinName clockPin, PinName dataPin>
inline uint32_t JR3PortRegister<portName, clockPin, dataPin>::readFrame()
{
    int pins;
    uint32_t frame = 0;

    awaitNextFrame();

    for (int i = FRAME_SIZE - 1; i >= 0; i--)
    {
        while ((*port_in & 0x00000001) == 1) {}

        // await rising edge on clock signal
        while (((pins = (*port_in & 0x00000003)) & 0x00000001) == 0) {}

        if ((pins & 0x00000002) == 0x00000002)
        {
            frame |= (1U << i);
        }
    }

    return frame;
}

template <PortName portName, PinName clockPin, PinName dataPin>
inline void JR3PortRegister<portName, clockPin, dataPin>::awaitNextFrame()
{
    int pins;

    while (true)
    {
        // await next interval between frames
        // while (!(data.read() == 1 && clock.read() == 1)) {}
        while ((*port_in & 0x00000003) != 0x00000003) {}

        // await beginning of start pulse
        // while (data.read() == 1 && clock.read() == 1) {}
        while ((pins = (*port_in & 0x00000003)) == 0x00000003) {}

        // if (data.read() != 0 || clock.read() != 1)
        if (pins != 0x00000001)
        {
            continue; // no start pulse, retry
        }

        // await rising edge of start pulse
        // while (data.read() == 0 && clock.read() == 1) {}
        while ((pins = (*port_in & 0x00000003)) == 0x00000001) {}

        // if (data.read() != 1 || clock.read() != 1)
        if (pins != 0x00000003)
        {
            continue; // no start pulse, retry
        }

        // // await end of start pulse
        // // while (data.read() == 1 && clock.read() == 1) {} &= ~obj->mas &= ~obj->mas
        // while ((pins = (*port_in & 0x00000003)) == 0x00000003) {}

        // // if (clock.read() != 0)
        // if ((pins & 0x00000001) != 0)
        // {
        //     continue; // this is not the first bit, try again
        // }

        break; // start pulse completed, we can start processing a new frame
    }
}

#endif // __JR3_PORT_REGISTER_HPP__
