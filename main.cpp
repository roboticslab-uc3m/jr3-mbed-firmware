#include "mbed.h"
#include "Jr3.hpp"
#include "Jr3Controller.hpp"
#include "Motor.h"

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

float parseCutOffFrequency(const CANMessage & msg)
{
    if (msg.len == sizeof(float)) // 4
    {
        float temp;
        memcpy(&temp, msg.data, sizeof(float));
        return temp;
    }
    else
    {
        return 0.0f;
    }
}

int main()
{
    ThisThread::sleep_for(5s);

    printf("ready\n");

    CAN can_jr3(MBED_CONF_APP_CAN_JR3_RD_PIN, MBED_CONF_APP_CAN_JR3_TD_PIN);
    can_jr3.frequency(MBED_CONF_APP_CAN_JR3_BAUDRATE);
    can_jr3.filter(MBED_CONF_APP_CAN_JR3_ID, 0x007F, CANStandard);
    can_jr3.reset();

    CAN can_lacquey(MBED_CONF_APP_CAN_LACQUEY_RD_PIN, MBED_CONF_APP_CAN_LACQUEY_TD_PIN);
    can_jr3.frequency(MBED_CONF_APP_CAN_LACQUEY_BAUDRATE);
    can_lacquey.filter(MBED_CONF_APP_CAN_LACQUEY_ID, 0x007F, CANStandard);
    can_lacquey.reset();

    CANMessage msg_in;
    CANMessage msg_out_forces, msg_out_moments, msg_out_ack;

    msg_out_forces.len = msg_out_moments.len = 6;
    msg_out_forces.id = (JR3_GET_FORCES << 7) + MBED_CONF_APP_CAN_JR3_ID;
    msg_out_moments.id = (JR3_GET_MOMENTS << 7) + MBED_CONF_APP_CAN_JR3_ID;

    msg_out_ack.len = 0;
    msg_out_ack.id = (JR3_ACK << 7) + MBED_CONF_APP_CAN_JR3_ID;

    using Jr3Reader = Jr3<MBED_CONF_APP_JR3_PORT, MBED_CONF_APP_JR3_CLOCK_PIN, MBED_CONF_APP_JR3_DATA_PIN>;
    Jr3Controller<Jr3Reader> controller;

    Motor motor(MBED_CONF_APP_LACQUEY_PWM_PIN, MBED_CONF_APP_LACQUEY_FWD_PIN, MBED_CONF_APP_LACQUEY_REV_PIN);

    uint16_t data[6];
    float pwm;

    while (true)
    {
        if (can_jr3.read(msg_in))
        {
            switch ((msg_in.id & 0x0780) >> 7)
            {
            case JR3_START:
                printf("received JR3 start command\n");
                controller.start(parseCutOffFrequency(msg_in));
                can_jr3.write(msg_out_ack);
                break;
            case JR3_STOP:
                printf("received JR3 stop command\n");
                controller.stop();
                can_jr3.write(msg_out_ack);
                break;
            case JR3_ZERO_OFFS:
                printf("received JR3 zero offsets command\n");
                controller.calibrate();
                can_jr3.write(msg_out_ack);
                break;
            case JR3_SET_FILTER:
                printf("received JR3 set filter command\n");
                controller.setFilter(parseCutOffFrequency(msg_in));
                break;
            }
        }

        if (controller.acquire(data))
        {
            memcpy(msg_out_forces.data, data, 6); // fx, fy, fz
            can_jr3.write(msg_out_forces);

            memcpy(msg_out_moments.data, data + 3, 6); // mx, my, mz
            can_jr3.write(msg_out_moments);
        }

        if (can_lacquey.read(msg_in) && msg_in.len == sizeof(float))
        {
            memcpy(&pwm, msg_in.data, sizeof(float));
            printf("PWM value received: %f\n", pwm);

            if (pwm >= -100.0f && pwm <= 100.0f)
            {
                motor.speed(pwm / 100.0f);
            }
        }

        wait_us(1); // this spins the CPU
        // ThisThread::sleep_for(1ms); // this actually sleeps the thread
    }
}
