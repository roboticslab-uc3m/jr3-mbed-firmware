#include "mbed.h"

#include "Jr3.hpp"
#include "Jr3Controller.hpp"

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
# include "Motor.h"
#endif

#include "atomic_bool.h"

enum can_ops : uint8_t
{
    SYNC = 1,        // 0x080
    JR3_BOOTUP,      // 0x100
    JR3_ACK,         // 0x180
    JR3_START_SYNC,  // 0x200
    JR3_START_ASYNC, // 0x280
    JR3_STOP,        // 0x300
    JR3_ZERO_OFFS,   // 0x380
    JR3_SET_FILTER,  // 0x400
    JR3_GET_FS,      // 0x480
    JR3_RESET,       // 0x500
    JR3_FORCES,      // 0x580
    JR3_MOMENTS,     // 0x600
#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    GRIPPER_PWM = 15 // 0x780
#endif
};

using counter_t = uint16_t;

uint16_t parseCutOffFrequency(const mbed::CANMessage & msg, size_t offset = 0)
{
    if (msg.len >= sizeof(uint16_t) + offset)
    {
        uint16_t temp;
        memcpy(&temp, msg.data + offset, sizeof(uint16_t));
        return temp;
    }
    else
    {
        return 0;
    }
}

uint32_t parseAsyncPeriod(const mbed::CANMessage & msg, size_t offset = 0)
{
    if (msg.len >= sizeof(uint32_t) + offset)
    {
        uint32_t temp;
        memcpy(&temp, msg.data + offset, sizeof(uint32_t));
        return temp;
    }
    else
    {
        return 0;
    }
}

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
void processGripperCommand(const mbed::CANMessage & msg, Motor & motor)
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

void sendData(mbed::CAN & can, mbed::CANMessage & msg_forces, mbed::CANMessage & msg_moments, uint16_t * data)
{
    static counter_t counter = 0;

    memcpy(msg_forces.data, data, 6); // fx, fy, fz
    memcpy(msg_forces.data + 6, &counter, sizeof(counter));
    can.write(msg_forces);

    // there is no need to put a delay between these two writes since the NXP has a triple transmit buffer

    memcpy(msg_moments.data, data + 3, 6); // mx, my, mz
    memcpy(msg_moments.data + 6, &counter, sizeof(counter));
    can.write(msg_moments);

    counter++;
}

void sendAcknowledge(mbed::CAN & can, mbed::CANMessage & msg, const Jr3Controller & controller)
{
    static constexpr uint8_t OK = 0x00;
    static constexpr uint8_t ERROR = 0x01;

    msg.data[0] = controller.getState() == Jr3Controller::READY ? OK : ERROR;

    can.write(msg);
}

int main()
{
    rtos::ThisThread::sleep_for(2s);

    printf("booting\n");

    mbed::RawCAN can(MBED_CONF_APP_CAN_RD_PIN, MBED_CONF_APP_CAN_TD_PIN);
    can.frequency(MBED_CONF_APP_CAN_BAUDRATE);
    can.reset();

#if MBED_CONF_APP_CAN2_ENABLE
    mbed::CAN can2(MBED_CONF_APP_CAN2_RD_PIN, MBED_CONF_APP_CAN2_TD_PIN);
    can2.frequency(MBED_CONF_APP_CAN2_BAUDRATE);
    can2.reset();
#endif

    mbed::CANMessage msg_in;
    mbed::CANMessage msg_out_bootup, msg_out_ack, msg_out_forces, msg_out_moments;

    msg_out_bootup.len = 0;
    msg_out_bootup.id = (JR3_BOOTUP << 7) + MBED_CONF_APP_CAN_ID;

    msg_out_ack.len = 1;
    msg_out_ack.id = (JR3_ACK << 7) + MBED_CONF_APP_CAN_ID;

    msg_out_forces.len = msg_out_moments.len = 6 + sizeof(counter_t);
    msg_out_forces.id = (JR3_FORCES << 7) + MBED_CONF_APP_CAN_ID;
    msg_out_moments.id = (JR3_MOMENTS << 7) + MBED_CONF_APP_CAN_ID;

    using Jr3Reader = Jr3<MBED_CONF_APP_JR3_PORT, MBED_CONF_APP_JR3_CLOCK_PIN, MBED_CONF_APP_JR3_DATA_PIN>;
    Jr3Reader jr3;
    Jr3Controller controller({&jr3, &Jr3Reader::readFrame});

#if MBED_CONF_APP_CAN_USE_GRIPPER || MBED_CONF_APP_CAN2_ENABLE
    Motor motor(MBED_CONF_APP_GRIPPER_PWM_PIN, MBED_CONF_APP_GRIPPER_FWD_PIN, MBED_CONF_APP_GRIPPER_REV_PIN);
#endif

    mbed::CircularBuffer<mbed::CANMessage, 32> queue;
    AtomicBool syncReceived = false;

    can.attach([&can, &syncReceived, &queue]() {
        mbed::CANMessage msg;

        if (can.read(msg))
        {
            if (msg.id == SYNC << 7)
            {
                syncReceived = true;
            }
            else if ((msg.id & 0x07F) == MBED_CONF_APP_CAN_ID)
            {
                queue.push(msg);
            }
        }
    });

    if (jr3.isConnected())
    {
        printf("JR3 sensor is connected\n");
        controller.initialize(); // this blocks until the initialization is completed
        can.write(msg_out_bootup);
    }

    uint16_t data[6]; // helper buffer for misc FT data

    while (true)
    {
        if (syncReceived)
        {
            if (controller.acquire(data))
            {
                sendData(can, msg_out_forces, msg_out_moments, data);
            }

            syncReceived = false;
        }

        while (queue.pop(msg_in))
        {
            switch ((msg_in.id & 0x0780) >> 7)
            {
            case JR3_START_SYNC:
                printf("received JR3 start command (synchronous)\n");
                controller.setFilter(parseCutOffFrequency(msg_in));
                controller.startSync();
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_START_ASYNC:
                printf("received JR3 start command (asynchronous)\n");
                controller.setFilter(parseCutOffFrequency(msg_in));
                controller.startAsync([&can, &msg_out_forces, &msg_out_moments](uint16_t * data)
                {
                    sendData(can, msg_out_forces, msg_out_moments, data);
                }, parseAsyncPeriod(msg_in, sizeof(uint16_t)));
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_STOP:
                printf("received JR3 stop command\n");
                controller.stop();
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_ZERO_OFFS:
                printf("received JR3 zero offsets command\n");
                controller.calibrate();
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_SET_FILTER:
                printf("received JR3 set filter command\n");
                controller.setFilter(parseCutOffFrequency(msg_in));
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_GET_FS:
                printf("received JR3 get full scales command\n");
                controller.getFullScales(data);
                sendData(can, msg_out_forces, msg_out_moments, data);
                sendAcknowledge(can, msg_out_ack, controller);
                break;
            case JR3_RESET:
                printf("received JR3 reset command\n");
                controller.initialize();
                sendAcknowledge(can, msg_out_ack, controller);
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

#if MBED_CONF_APP_CAN2_ENABLE
        if (can2.read(msg_in) && msg_in.id == (GRIPPER_PWM << 7) + MBED_CONF_APP_CAN2_ID)
        {
            processGripperCommand(msg_in, motor);
        }
#endif

        // this is the minimum amount of time that actually sleeps the thread, use AccurateWaiter
        // for the microsecond scale; wait_us(), on the contrary, spins the CPU
        rtos::ThisThread::sleep_for(1ms);
    }
}
