#define LSB_MOTOR 0xA8
#define LSB_CONNECT 0xB9
#define CMD_M1 1
#define CMD_M2 2
#define CMD_M3 3
#define CMD_M4 4
#define CMD_M5 5
#define CMD_M6 6
#define MAX_CYCLES_PER_STEP 20
#define MAX_MOTOR_CMD 100.0
#define MOTOR_DELAY 1
#define false 0
#define true 1

typedef struct{
    int8_t payload[6];
    uint8_t new;
}MotorCmdFrame;
