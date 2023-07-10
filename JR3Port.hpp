#ifndef __JR3_PORT_HPP__
#define __JR3_PORT_HPP__

#include "mbed.h"

// https://github.com/ARMmbed/mbed-os/blob/f9c0cd2a94fc1353946b321aad7f8bca2bfe1b40/targets/TARGET_NXP/TARGET_LPC176X/port_api.c
// https://os.mbed.com/users/igorsk/notebook/fast-gpio-with-c-templates/
// https://os.mbed.com/users/igorsk/code/FastIO//file/b8e40f2a0aac/FastIO.h/

class JR3Port
{
public:
    JR3Port(PortName portName, PinName clockPin, PinName dataPin)
        : port(portName, 0x00000003)
        // : port(portName, (1UL << ((clockPin - P0_0) % 32)) | (1UL << ((dataPin - P0_0) % 32))),
        //   DATA_LOW_CLOCK_LOW(0),
        //   DATA_LOW_CLOCK_HIGH(1UL << ((clockPin - P0_0) % 32)),
        //   DATA_HIGH_CLOCK_LOW(1UL << ((dataPin - P0_0) % 32)),
        //   DATA_HIGH_CLOCK_HIGH((1UL << ((clockPin - P0_0) % 32)) | (1UL << ((dataPin - P0_0) % 32))),
        //   CLOCK_LOW()
    {}

    uint32_t readFrame();

private:
    void awaitNextFrame();

    PortIn port;

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

inline uint32_t JR3Port::readFrame()
{
    int pins;
    uint32_t frame = 0;

    awaitNextFrame();

    for (int i = FRAME_SIZE - 1; i >= 0; i--)
    {
        // await end of previous pulse
        while ((port.read() & 0x00000001) == 1) {}

        // await rising edge on clock signal
        while (((pins = port.read()) & 0x00000001) == 0) {}

        if ((pins & 0x00000002) == 0x00000002)
        {
            frame |= (1U << i);
        }
    }

    return frame;
}

inline void JR3Port::awaitNextFrame()
{
    int pins;

    while (true)
    {
        // await next interval between frames
        // while (!(data.read() == 1 && clock.read() == 1)) {}
        while (port.read() != 0x00000003) {}

        // await beginning of start pulse
        // while (data.read() == 1 && clock.read() == 1) {}
        while ((pins = port.read()) == 0x00000003) {}

        // if (data.read() != 0 || clock.read() != 1)
        if (pins != 0x00000001)
        {
            continue; // no start pulse, retry
        }

        // await rising edge of start pulse
        // while (data.read() == 0 && clock.read() == 1) {}
        while ((pins = port.read()) == 0x00000001) {}

        // if (data.read() != 1 || clock.read() != 1)
        if (pins != 0x00000003)
        {
            continue; // no start pulse, retry
        }

        // // await end of start pulse
        // // while (data.read() == 1 && clock.read() == 1) {}
        // while ((pins = port.read()) == 0x00000003) {}

        // // if (clock.read() != 0)
        // if ((pins & 0x00000001) != 0)
        // {
        //     continue; // this is not the first bit, try again
        // }

        break; // start pulse completed, we can start processing a new frame
    }
}

#endif // __JR3_PORT_HPP__
