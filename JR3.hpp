#ifndef __JR3_HPP__
#define __JR3_HPP__

#include "FastIO.h"

template <PinName clockPin, PinName dataPin>
class JR3
{
public:
    uint32_t readFrame();

private:
    void awaitNextFrame();

    static const unsigned int FRAME_SIZE = 20;

    FastIn<clockPin> clock;
    FastIn<dataPin> data;
};

template <PinName clockPin, PinName dataPin>
inline uint32_t JR3<clockPin, dataPin>::readFrame()
{
    uint32_t frame = 0;

    awaitNextFrame();

    for (int i = FRAME_SIZE - 1; i >= 0; i--)
    {
        // await end of previous pulse
        while (clock.read() == 1) {}

        // await rising edge on clock signal
        while (clock.read() == 0) {}

        if (data.read())
        {
            frame |= (1 << i);
        }
    }

    return frame;
}

template <PinName clockPin, PinName dataPin>
inline void JR3<clockPin, dataPin>::awaitNextFrame()
{
    int clockState, dataState;

    while (true)
    {
        // await next interval between frames
        while (!(data.read() == 1 && clock.read() == 1)) {}

        // await beginning of start pulse
        while ((dataState = data.read()) == 1 && (clockState = clock.read()) == 1) {}

        if (dataState != 0 || clockState != 1)
        {
            continue; // no start pulse, retry
        }

        // await rising edge of start pulse
        while ((dataState = data.read()) == 0 && (clockState = clock.read()) == 1) {}

        if (dataState != 1 || clockState != 1)
        {
            continue; // no start pulse, retry
        }

        // // await end of start pulse
        // while (data.read() == 1 && (clockState = clock.read()) == 1) {}

        // if (clockState != 0)
        // {
        //     continue; // this is not the first bit, try again
        // }

        break; // start pulse completed, we can start processing a new frame
    }
}

#endif // __JR3_HPP__
