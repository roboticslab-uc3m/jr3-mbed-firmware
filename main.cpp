#include "mbed.h"

#include "Jr3.hpp"
#include "Jr3Controller.hpp"

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
# include "Motor.h"
#endif

enum can_ops : uint8_t
{
    JR3_START = 1,   // 0x080
    JR3_STOP,        // 0x100
    JR3_ZERO_OFFS,   // 0x180
    JR3_SET_FILTER,  // 0x200
    JR3_GET_FORCES,  // 0x280
    JR3_GET_MOMENTS, // 0x300
    JR3_ACK,         // 0x380
#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    GRIPPER_PWM = 14 // 0x700
#endif
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

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
void processGripperCommand(const CANMessage & msg, Motor & motor)
{
    if (msg.len == sizeof(float))
    {
        float pwm;
        memcpy(&pwm, msg.data, sizeof(float));
        printf("received gripper PWM value: %f\n", pwm);

        if (pwm >= -100.0f && pwm <= 100.0f)
        {
            motor.speed(pwm / 100.0f);
        }
    }
}
#endif

int main()
{
    ThisThread::sleep_for(5s);

    printf("ready\n");

    CAN can(MBED_CONF_APP_CAN_RD_PIN, MBED_CONF_APP_CAN_TD_PIN);
    can.frequency(MBED_CONF_APP_CAN_BAUDRATE);
    can.reset();

#if MBED_CONF_APP_CAN2_ENABLE
    CAN can2(MBED_CONF_APP_CAN2_RD_PIN, MBED_CONF_APP_CAN2_TD_PIN);
    can2.frequency(MBED_CONF_APP_CAN2_BAUDRATE);
    can2.reset();
#endif

    CANMessage msg_in;
    CANMessage msg_out_forces, msg_out_moments, msg_out_ack;

    msg_out_forces.len = msg_out_moments.len = 6;
    msg_out_forces.id = (JR3_GET_FORCES << 7) + MBED_CONF_APP_CAN_ID;
    msg_out_moments.id = (JR3_GET_MOMENTS << 7) + MBED_CONF_APP_CAN_ID;

    msg_out_ack.len = 0;
    msg_out_ack.id = (JR3_ACK << 7) + MBED_CONF_APP_CAN_ID;

    using Jr3Reader = Jr3<MBED_CONF_APP_JR3_PORT, MBED_CONF_APP_JR3_CLOCK_PIN, MBED_CONF_APP_JR3_DATA_PIN>;
    Jr3Controller<Jr3Reader> controller;

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    Motor motor(MBED_CONF_APP_LACQUEY_PWM_PIN, MBED_CONF_APP_LACQUEY_FWD_PIN, MBED_CONF_APP_LACQUEY_REV_PIN);
#endif

    uint16_t data[6];

    while (true)
    {
        if (can.read(msg_in) && (msg_in.id & 0x07F) == MBED_CONF_APP_CAN_ID)
        {
            switch ((msg_in.id & 0x0780) >> 7)
            {
            case JR3_START:
                printf("received JR3 start command\n");
                controller.start(parseCutOffFrequency(msg_in));
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
                controller.setFilter(parseCutOffFrequency(msg_in));
                break;
#if MBED_CONF_APP_CAN_USE_GRIPPER
            case GRIPPER_PWM:
                processGripperCommand(msg_in, motor);
                break;
#endif
            default:
                printf("unsupported command: %d\n", (msg_in.id & 0x0780) >> 7);
                break;
            }
        }

        if (controller.acquire(data))
        {
            // there is no need to put a delay between these two writes since the NXP has a triple transmit buffer

            memcpy(msg_out_forces.data, data, 6); // fx, fy, fz
            can.write(msg_out_forces);

            memcpy(msg_out_moments.data, data + 3, 6); // mx, my, mz
            can.write(msg_out_moments);
        }

#if MBED_CONF_APP_CAN2_ENABLE
        if (can2.read(msg_in) && (msg_in.id & 0x07F) == MBED_CONF_APP_CAN2_ID && (msg_in.id & 0x0780) >> 7 == GRIPPER_PWM)
        {
            processGripperCommand(msg_in, motor);
        }
#endif

        wait_us(1); // this spins the CPU
        // ThisThread::sleep_for(1ms); // this actually sleeps the thread
    }
}
