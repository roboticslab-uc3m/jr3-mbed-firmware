#include "mbed.h"
#include "Jr3.hpp"
#include "Jr3Controller.hpp"

constexpr auto JR3_PORT = Port0;
constexpr auto CLOCK_PIN = p9;
constexpr auto DATA_PIN = p10;

constexpr auto CAN_RD_PIN = p30;
constexpr auto CAN_TD_PIN = p29;
constexpr auto CAN_BAUDRATE = 1000000;
constexpr auto CAN_ID = 0x01;

enum jr3_can_ops : uint8_t
{
    JR3_START = 1,   // 0x080
    JR3_STOP,        // 0x100
    JR3_ZERO_OFFS,   // 0x180
    JR3_SET_FILTER,  // 0x200
    JR3_GET_FORCES,  // 0x280
    JR3_GET_MOMENTS, // 0x300
    JR3_ACK          // 0x380
};

int main()
{
    ThisThread::sleep_for(5s);

    printf("ready\n");

    CAN can(CAN_RD_PIN, CAN_TD_PIN, CAN_BAUDRATE);
    can.filter(CAN_ID, 0x007F, CANStandard);
    can.reset();

    CANMessage msg_in;
    CANMessage msg_out_forces, msg_out_moments, msg_out_ack;

    msg_out_forces.len = msg_out_moments.len = 6;
    msg_out_forces.id = (JR3_GET_FORCES << 7) + CAN_ID;
    msg_out_moments.id = (JR3_GET_MOMENTS << 7) + CAN_ID;

    msg_out_ack.len = 0;
    msg_out_ack.id = (JR3_ACK << 7) + CAN_ID;

    using Jr3Reader = Jr3<JR3_PORT, CLOCK_PIN, DATA_PIN>;
    Jr3Controller<Jr3Reader> controller;

    uint16_t data[6];

    while (true)
    {
        if (can.read(msg_in))
        {
            switch ((msg_in.id & 0x0780) >> 7)
            {
            case JR3_START:
                printf("received JR3 start command\n");
                controller.start();
                can.write(msg_out_ack);
                break;
            case JR3_STOP:
                printf("received JR3 stop command\n");
                controller.stop();
                can.write(msg_out_ack);
                break;
            case JR3_ZERO_OFFS:
                printf("received JR3 zero offsets command\n");
                controller.calibrate();
                can.write(msg_out_ack);
                break;
            case JR3_SET_FILTER:
                printf("received JR3 set filter command\n");
                // TODO
                break;
            }
        }

        if (controller.acquire(data))
        {
            memcpy(msg_out_forces.data, data, 6); // fx, fy, fz
            can.write(msg_out_forces);

            memcpy(msg_out_moments.data, data + 3, 6); // mx, my, mz
            can.write(msg_out_moments);
        }

        wait_us(1); // this spins the CPU
        // ThisThread::sleep_for(1ms); // this actually sleeps the thread
    }
}
