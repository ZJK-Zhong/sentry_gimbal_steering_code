/* Run production empty-board CMD, DBUS, CANComm and pub/sub on ARM.
 * Physical CAN/UART and time are simulated; no gimbal hardware is initialized. */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../application/board_transport.c"
#include "../application/cmd/robot_cmd.c"
#include "../modules/remote/remote_control.c"

CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1, huart3, huart6;
static uint32_t test_tick = 100;
static CANInstance *test_can[16];
static unsigned test_can_count;
static USARTInstance test_uart;
static CANCommInstance *peer;
static Chassis_Upload_Data_s peer_feedback;
static int16_t test_dial;
static char test_heap[32768] __attribute__((aligned(8)));
static ptrdiff_t heap_used;

void *_sbrk(ptrdiff_t size)
{
    if (size < 0 || heap_used + size > sizeof(test_heap)) return (void *)-1;
    void *result = test_heap + heap_used;
    heap_used += size;
    return result;
}

static void host_call(uint32_t operation, const void *argument)
{
    register uint32_t op __asm__("r0") = operation;
    register const void *arg __asm__("r1") = argument;
    __asm__ volatile("bkpt 0xab" : "+r"(op) : "r"(arg) : "memory");
}
static void finish(uint32_t status)
{
    uint32_t args[2] = {0x20026, status};
    host_call(0x20, args);
    for (;;) {}
}
#define STR_(x) #x
#define STR(x) STR_(x)
#define CHECK(x) do { if (!(x)) { host_call(4, "FAIL line " STR(__LINE__) ": " #x "\n"); finish(__LINE__); } } while (0)

uint32_t HAL_GetTick(void) { return test_tick; }
float DWT_GetTimeline_ms(void) { return (float)(test_tick + 10000u); }
float DWT_GetDeltaT(uint32_t *counter) { *counter = test_tick * 168000u; return 0.002f; }
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{
    test_uart.usart_handle = config->usart_handle;
    test_uart.recv_buff_size = config->recv_buff_size;
    test_uart.module_callback = config->module_callback;
    return &test_uart;
}
void USARTServiceInit(USARTInstance *instance) {}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CHECK(test_can_count < 16);
    for (unsigned i = 0; i < test_can_count; ++i)
        CHECK(test_can[i]->can_handle != config->can_handle || test_can[i]->rx_id != config->rx_id);
    CANInstance *instance = calloc(1, sizeof(*instance));
    CHECK(instance != NULL);
    instance->can_handle = config->can_handle;
    instance->tx_id = instance->txconf.StdId = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->id = config->id;
    instance->can_module_callback = config->can_module_callback;
    test_can[test_can_count++] = instance;
    return instance;
}
void CANSetDLC(CANInstance *instance, uint8_t length) { instance->txconf.DLC = length; }
uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    CHECK(instance->txconf.StdId == BOARD_GIMBAL_TX_ID || instance->txconf.StdId == BOARD_CHASSIS_TX_ID);
    for (unsigned i = 0; i < test_can_count; ++i)
    {
        CANInstance *rx = test_can[i];
        if (rx->can_handle == instance->can_handle && rx->rx_id == instance->txconf.StdId)
        {
            rx->rx_len = instance->txconf.DLC;
            memcpy(rx->rx_buff, instance->tx_buff, rx->rx_len);
            rx->can_module_callback(rx);
        }
    }
    return 1;
}

static void remote_frame(uint8_t right, uint8_t left, int16_t x, int16_t y, int16_t yaw)
{
    uint16_t ch[5] = {y + 1024, x + 1024, yaw + 1024, 1024, test_dial + 1024};
    uint8_t *p = test_uart.recv_buff;
    memset(p, 0, 18);
    p[0] = ch[0]; p[1] = (ch[0] >> 8) | (ch[1] << 3);
    p[2] = (ch[1] >> 5) | (ch[2] << 6); p[3] = ch[2] >> 2;
    p[4] = (ch[2] >> 10) | (ch[3] << 1);
    p[5] = (ch[3] >> 7) | (right << 4) | (left << 6);
    p[16] = ch[4]; p[17] = ch[4] >> 8;
    test_uart.recv_len = 18;
    test_uart.module_callback();
}

/* Exercise the empty upper board with real DBUS, message center and CANComm.
 * No gimbal/shooter initialization or feedback is provided. */
static void bench_step(uint8_t right, uint8_t left, uint8_t feedback)
{
    test_tick += 5;
    remote_frame(right, left, 100, -50, 163);
    if (feedback) CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask();
}

static Chassis_Ctrl_Cmd_s decoded_command(void)
{
    Chassis_Ctrl_Cmd_s cmd;
    CHECK(CANCommSnapshot(peer, &cmd, 50));
    return cmd;
}

static void check_bench_stopped(void)
{
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_SAFE ||
          gimbal_control.control_mode == ROBOT_CONTROL_MODE_ZERO_FORCE);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ZERO_FORCE);
    Chassis_Ctrl_Cmd_s decoded = decoded_command();
    Chassis_Ctrl_Cmd_s zero = {0};
    CHECK(memcmp(&decoded, &zero, sizeof(zero)) == 0);
    CHECK(gimbal_control.gimbal_cmd.gimbal_mode == GIMBAL_ZERO_FORCE);
    CHECK(gimbal_control.shoot_cmd.shoot_mode == SHOOT_OFF);
    CHECK(gimbal_control.shoot_cmd.friction_mode == FRICTION_OFF);
    CHECK(gimbal_control.shoot_cmd.load_mode == LOAD_STOP);
    CHECK(!gimbal_control.gimbal_online && !gimbal_control.vision_online);
    CHECK(gimbal_control.bench_active == 1);
}

void test_main(void)
{
    RobotCMDInit();
    CHECK(test_uart.usart_handle == &huart3);
    CHECK(test_can_count == 1); // Only board link; no motor endpoints.
    CHECK(board_comm->can_ins->can_handle == &hcan1);
    CANComm_Init_Config_s config = {
        .can_config = {.can_handle = &hcan1, .tx_id = BOARD_CHASSIS_TX_ID, .rx_id = BOARD_GIMBAL_TX_ID},
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s), .daemon_count = 5};
    peer = CANCommInit(&config);
    // Online transport and motor/INS readiness are different facts.
    peer_feedback.motors_online = peer_feedback.ins_online = 0;
    test_tick += 10;
    RobotCMDTask(); check_bench_stopped();
    CHECK(!gimbal_control.rc_online && !gimbal_control.chassis_online && !gimbal_control.armed);
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 0);
    CHECK(!gimbal_control.armed);
    bench_step(RC_SW_DOWN, RC_SW_UP, 1); check_bench_stopped();
    CHECK(gimbal_control.chassis_online && !gimbal_control.armed);
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 1);
    CHECK(gimbal_control.armed && !gimbal_control.gimbal_online);

    const uint8_t pos[] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP};
    for (unsigned left = 0; left < 3; ++left)
        for (unsigned right = 0; right < 3; ++right)
        {
            bench_step(pos[right], pos[left], 1);
            // Refresh once more to cross the normal 10ms transport period.
            bench_step(pos[right], pos[left], 1);
            Chassis_Ctrl_Cmd_s decoded = decoded_command();
            CHECK(!gimbal_control.gimbal_online && !gimbal_control.vision_online);
            CHECK(gimbal_control.gimbal_cmd.gimbal_mode == GIMBAL_ZERO_FORCE);
            CHECK(gimbal_control.shoot_cmd.shoot_mode == SHOOT_OFF);
            if (left == 2 && right <= 1)
            {
                CHECK(fabsf(decoded.vx - 600.0f) < 0.001f);
                CHECK(fabsf(decoded.vy + 100.0f) < 0.001f);
                CHECK(decoded.offset_angle == 0);
                if (right == 0)
                {
                    CHECK(decoded.mode == 0 && decoded.chassis_mode == CHASSIS_REMOTE);
                    CHECK(fabsf(decoded.wz + 163 * 0.2f * DEGREE_2_RAD) < 0.001f);
                    CHECK(decoded.chassis_spin_speed == 0);
                }
                else
                {
                    CHECK(decoded.mode == 3 && decoded.chassis_mode == CHASSIS_ROTATE);
                    CHECK(fabsf(decoded.chassis_spin_speed - 120 * DEGREE_2_RAD) < 0.001f);
                    CHECK(decoded.wz == 0);
                }
            }
            else check_bench_stopped();
        }
    host_call(4, "PASS: empty upper board, nine switch combinations, manual chassis frame, spin entry frame, no gimbal/shooter commands\n");

    bench_step(RC_SW_DOWN, RC_SW_UP, 1);
    memset(&gimbal_control, 0xff, sizeof(gimbal_control));
    memset(&board_link_debug, 0xff, sizeof(board_link_debug));
    bench_step(RC_SW_DOWN, RC_SW_UP, 1);
    CHECK(gimbal_control.armed && gimbal_control.control_mode == ROBOT_CONTROL_MODE_MOVE);
    CHECK(gimbal_control.bench_active == 1 && !gimbal_control.gimbal_online);
    // A partial/invalid frame cannot be treated as a fresh complete command.
    peer->raw_sendbuf[peer->send_buf_len - 2] ^= 1;
    for (unsigned offset = 0; offset < peer->send_buf_len; offset += 8)
    {
        unsigned len = peer->send_buf_len - offset;
        if (len > 8) len = 8;
        CANSetDLC(peer->can_ins, len);
        memcpy(peer->can_ins->tx_buff, peer->raw_sendbuf + offset, len);
        CANTransmit(peer->can_ins, 1);
    }
    bench_step(RC_SW_DOWN, RC_SW_UP, 0); check_bench_stopped();
    CHECK(!gimbal_control.armed);
    bench_step(RC_SW_DOWN, RC_SW_UP, 1); check_bench_stopped();
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 1);
    CHECK(gimbal_control.armed);

    bench_step(RC_SW_MID, RC_SW_UP, 1);
    test_tick += 95;
    bench_step(RC_SW_MID, RC_SW_UP, 0); // Exactly 100ms since feedback.
    CHECK(gimbal_control.chassis_online && gimbal_control.armed);
    bench_step(RC_SW_MID, RC_SW_UP, 0); check_bench_stopped();
    CHECK(!gimbal_control.chassis_online && !gimbal_control.armed);
    bench_step(RC_SW_MID, RC_SW_UP, 1); check_bench_stopped();
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 1);
    bench_step(RC_SW_DOWN, RC_SW_UP, 1);
    test_tick += 101;
    CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask(); check_bench_stopped(); // RC stale despite fresh peer.
    CHECK(!gimbal_control.rc_online && !gimbal_control.armed);
    bench_step(RC_SW_DOWN, RC_SW_UP, 1); check_bench_stopped();
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 1);
    test_dial = 301;
    bench_step(RC_SW_DOWN, RC_SW_UP, 1); check_bench_stopped();
    CHECK(!gimbal_control.armed);
    test_dial = 0;
    bench_step(RC_SW_DOWN, RC_SW_UP, 1); check_bench_stopped();
    bench_step(RC_SW_DOWN, RC_SW_DOWN, 1);
    bench_step(0, RC_SW_UP, 1); check_bench_stopped();
    CHECK(!gimbal_control.armed);
    CHECK(test_can_count == 2); // Link and simulated peer only.
    host_call(4, "PASS: CAN CRC/freshness, RC timeout, dial stop, invalid switches, double-down rearm and watch isolation\n");
    finish(0);
}
