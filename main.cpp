/**
 * Copyright: 2023 (C) Universidad Carlos III de Madrid
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE
 **/

#include "mbed.h"
#include "Motor.h"

Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);
CAN can(p30, p29); // rd, td
Motor m(p22, p23, p24); // pwm, fwd, rev
const unsigned int id = 64; // right-hand: ID 65, left-hand: ID 64

bool receive(float * value)
{
    CANMessage msg;

    if (can.read(msg))
    {
        if (id == msg.id)
        {
            pc.printf("Message received (ID: %d) (size: %d)\n", msg.id, msg.len);

            if (msg.len == 4)
            {
                pc.printf("Data: %x %x %x %x\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
                *value = *((float *)msg.data);
                led1 = !led1;
                return true;
            }
        }
    }

    return false;
}

int main()
{
    // set up the serial USB connection (PC)
    pc.baud(115200);
    pc.printf("main()\n");

    // set up the CAN bus
    can.frequency(1000000);
    can.reset();

    // message received
    float data;

    while (true)
    {
        if (receive(&data))
        {
            pc.printf("value received: %5.2f\n", data);

            if (data >= -100.0f && data <= 100.0f)
            {
                m.speed(data / 100.0f);
            }
        }
    }
}
