#include "mbed.h"

#include "Jr3.hpp"
#include "Jr3Controller.hpp"

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
# include "Motor.h"
#endif

enum can_ops : uint8_t
{
    SYNC = 1,           // 0x080
    JR3_START_SYNC,     // 0x100
    JR3_START_ASYNC,    // 0x180
    JR3_STOP,           // 0x200
    JR3_ZERO_OFFS,      // 0x280
    JR3_SET_FILTER,     // 0x300
    JR3_GET_FORCES,     // 0x380
    JR3_GET_MOMENTS,    // 0x400
    JR3_ACK,            // 0x480
#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    GRIPPER_PWM = 14    // 0x700
#endif
};

float parseCutOffFrequency(const CANMessage & msg)
{
    if (msg.len == 4)
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

void sendData(CAN & can, CANMessage & msg_forces, CANMessage & msg_moments, uint16_t * data)
{
    memcpy(msg_forces.data, data, 6); // fx, fy, fz
    can.write(msg_forces);

    // there is no need to put a delay between these two writes since the NXP has a triple transmit buffer

    memcpy(msg_moments.data, data + 3, 6); // mx, my, mz
    can.write(msg_moments);
}

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
    Jr3Reader jr3;
    Jr3Controller controller({&jr3, &Jr3Reader::readFrame});

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    Motor motor(MBED_CONF_APP_LACQUEY_PWM_PIN, MBED_CONF_APP_LACQUEY_FWD_PIN, MBED_CONF_APP_LACQUEY_REV_PIN);
#endif

    while (true)
    {
        if (can.read(msg_in))
        {
            if (msg_in.id == SYNC << 7)
            {
                if (uint16_t data[6]; controller.acquire(data))
                {
                    sendData(can, msg_out_forces, msg_out_moments, data);
                }
            }
            else if ((msg_in.id & 0x07F) == MBED_CONF_APP_CAN_ID)
            {
                switch ((msg_in.id & 0x0780) >> 7)
                {
                case JR3_START_SYNC:
                    printf("received JR3 start command (synchronous)\n");
                    controller.setFilter(parseCutOffFrequency(msg_in));
                    controller.startSync();
                    can.write(msg_out_ack);
                    break;
                case JR3_START_ASYNC:
                    printf("received JR3 start command (asynchronous)\n");
                    controller.setFilter(parseCutOffFrequency(msg_in));
                    controller.startAsync([&can, &msg_out_forces, &msg_out_moments](uint16_t * data)
                    {
                        sendData(can, msg_out_forces, msg_out_moments, data);
                    }, 0.0f); // TODO
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
                    can.write(msg_out_ack);
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
        }

#if MBED_CONF_APP_CAN2_ENABLE
        if (can2.read(msg_in) && msg_in.id == (GRIPPER_PWM << 7) + MBED_CONF_APP_CAN2_ID)
        {
            processGripperCommand(msg_in, motor);
        }
#endif

        wait_us(1); // this spins the CPU
        // ThisThread::sleep_for(1ms); // this actually sleeps the thread
    }
}
