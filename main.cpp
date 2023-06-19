/*
 * Copyright (c) 2017-2020 Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "LPC17xx.h"

static const unsigned FRAME_LENGTH = 20;
static const uint32_t SIGNAL_BIT = 0x10000000U; // don't use 0x80000000U as it is `osFlagsError`

volatile unsigned int _countClock;
volatile unsigned int _countData;

volatile bool isTransmittingFrame = false;
volatile bool isStartPulse = false; // pulse high-low-high on DATA while DLCK is high
volatile bool clockState = true;
volatile bool dataState = true;
volatile unsigned int index = FRAME_LENGTH;
volatile uint32_t buffer = SIGNAL_BIT;
volatile unsigned int frames = 0;

extern "C" void EINT3_IRQHandler(void)
{
    // Clear the interrupt.
    LPC_GPIOINT->IO0IntClr = ~((uint32_t) 0);

    if ((LPC_GPIOINT->IO0IntStatR & (1 << 0)) == (1 << 0))
    {
        _countClock++;

        clockState = true;

        if (isTransmittingFrame)
        {
            buffer |= (dataState ? 1 : 0) << --index;

            if (index == 0)
            {
                isTransmittingFrame = false;
                frames++;
                // thread.flags_set(buffer);
                // sem.release();
                index = FRAME_LENGTH;
            }
        }
        else if (isStartPulse)
        {
            // invalidate, we are amid a frame transmission (which is not being accounted)
            isStartPulse = false;
        }
    }
    else if ((LPC_GPIOINT->IO0IntStatF & (1 << 0)) == (1 << 0))
    {
        clockState = false;
    }
    else if ((LPC_GPIOINT->IO0IntStatR & (1 << 1)) == (1 << 1))
    {
        _countData++;

        dataState = true;

        if (!isTransmittingFrame && isStartPulse && clockState)
        {
            isStartPulse = false;
            isTransmittingFrame = true;
            buffer = SIGNAL_BIT;
        }
    }
    else if ((LPC_GPIOINT->IO0IntStatF & (1 << 1)) == (1 << 1))
    {
        dataState = false;

        if (!isTransmittingFrame && clockState)
        {
            isStartPulse = true;
        }
    }
}

class Counter
{
public:
    Counter(unsigned int clockPin, unsigned int dataPin)
    {
        __disable_irq();

        NVIC_DisableIRQ(EINT3_IRQn);

        // Power up GPIO.
        // LPC_SC->PCONP |= (1 << 15);

        // GPIO P0.0 and P0.1 as input.
        LPC_PINCON->PINSEL0 &= ~((1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
        // LPC_PINCON->PINMODE1 &= ~(0x3 << 0);
        // LPC_PINCON->PINMODE1 |= (0x3 << 0);
        LPC_GPIO0->FIODIR &= ~((1 << 1) | (1 << 0));

        LPC_GPIOINT->IO0IntEnR |= ((1 << 1) | (1 << 0));
        LPC_GPIOINT->IO0IntEnF |= ((1 << 1) | (1 << 0));

        LPC_SC->EXTINT = 1;

        NVIC_EnableIRQ(EINT3_IRQn);

        __enable_irq();
    }
};

int main()
{
    Counter counter(9, 10);

    while (true)
    {
        printf("Count so far: cticks=%d dticks=%d frames=%d data=0x%08X\n", _countClock, _countData, frames, buffer);
        ThisThread::sleep_for(2000);
    }

    return 0;
}
