/* Run production CMD, DBUS, CANComm, gimbal/shooter apps, PID and pub/sub on ARM.
 * Physical CAN/UART/INS/DWT and vision input interfaces are simulated. */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "general_def.h"
#include "../application/cmd/robot_control_config.h"
#include "../application/board_transport.c"
#include "../application/cmd/robot_cmd.c"
#include "../application/gimbal/gimbal.c"
#include "../application/shoot/shoot.c"
#include "../modules/remote/remote_control.c"

CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1, huart3, huart6;
static uint32_t test_tick = 100;
static CANInstance *test_can[16];
static unsigned test_can_count;
static USARTInstance test_uart;
static attitude_t test_imu;
static uint8_t test_ins_online = 1;
static uint8_t current_frames[2][2][8];
static CANCommInstance *peer;
static Chassis_Upload_Data_s peer_feedback;
static int16_t friction_rpm;
static int16_t test_dial;
static uint8_t fail_board_transmit;
static Vision_Recv_s test_vision;
static uint8_t test_vision_new;
static uint32_t test_vision_rx_ms;
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
attitude_t *INS_Init(void) { return &test_imu; }
uint8_t INS_GetSnapshot(attitude_t *out) { *out = test_imu; return test_ins_online; }
Vision_Recv_s *VisionInit(UART_HandleTypeDef *uart) { CHECK(uart == &huart1); return &test_vision; }
uint8_t VisionCmdHasNewFrame(void) { uint8_t fresh = test_vision_new; test_vision_new = 0; return fresh; }
uint32_t VisionCmdLastRxTimeMs(void) { return test_vision_rx_ms; }
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
    if (fail_board_transmit && instance == board_link_instance->can_ins) return 0;
    unsigned bus = instance->can_handle == &hcan1 ? 0 : 1;
    if (instance->txconf.StdId == 0x200 || instance->txconf.StdId == 0x1ff)
        memcpy(current_frames[bus][instance->txconf.StdId == 0x1ff], instance->tx_buff, 8);
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

static void motor_frame(DJIMotorInstance *motor, uint16_t ecd, int16_t rpm)
{
    CANInstance *instance = motor->motor_can_instance;
    uint8_t *p = instance->rx_buff;
    memset(p, 0, 8);
    p[0] = ecd >> 8; p[1] = ecd;
    p[2] = (uint16_t)rpm >> 8; p[3] = rpm;
    instance->rx_len = 8;
    instance->can_module_callback(instance);
}
static void all_feedback(void)
{
    motor_frame(gimbal_yaw_motor, YAW_CHASSIS_ALIGN_ECD, 0);
    motor_frame(gimbal_pitch_motor, 1000, 0);
    motor_frame(friction_left_motor, 1000, -friction_rpm);
    motor_frame(friction_right_motor, 1000, friction_rpm);
    motor_frame(loader_motor, 1000, 0);
}
static void raw_packet(CANCommInstance *sender, unsigned length)
{
    for (unsigned offset = 0; offset < length; offset += 8)
    {
        unsigned part = length - offset > 8 ? 8 : length - offset;
        CANSetDLC(sender->can_ins, part);
        memcpy(sender->can_ins->tx_buff, sender->raw_sendbuf + offset, part);
        CANTransmit(sender->can_ins, 1);
    }
}
static void step(uint8_t right, uint8_t left, uint8_t send_feedback)
{
    test_tick += 5;
    remote_frame(right, left, 100, 0, 0);
    all_feedback();
    if (send_feedback) CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask(); GimbalTask(); ShootTask(); DJIMotorControl();
}
static void check_stopped(void)
{
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ZERO_FORCE);
    CHECK(gimbal_yaw_motor->stop_flag == MOTOR_STOP && gimbal_pitch_motor->stop_flag == MOTOR_STOP);
    CHECK(friction_left_motor->stop_flag == MOTOR_STOP && friction_right_motor->stop_flag == MOTOR_STOP);
    CHECK(loader_motor->stop_flag == MOTOR_STOP);
    for (unsigned b = 0; b < 2; ++b)
        for (unsigned g = 0; g < 2; ++g)
            for (unsigned i = 0; i < 8; ++i) CHECK(current_frames[b][g][i] == 0);
}

static void vision_frame(uint8_t tracking, uint8_t friction, uint8_t fire)
{
    memset(&test_vision, 0, sizeof(test_vision));
    test_vision.cmd.data.tracking.tracking = tracking;
    test_vision.cmd.data.shoot.fric_on = friction;
    test_vision.cmd.data.shoot.fire = fire;
    test_vision.cmd.data.speed_vector.vx = 0.5f;
    test_vision.cmd.data.speed_vector.vy = 0.25f;
    test_vision.cmd.data.speed_vector.wz = 0.5f;
    test_vision_rx_ms = (uint32_t)DWT_GetTimeline_ms();
    test_vision_new = 1;
}

#include "policy_context.h"

/** @brief 底盘权限不因途经工作档丢失，也不能在云台恢复时被提升为云台/发射权限。 */
static void TestChassisOnlyArmingTransitions(void)
{
    RobotControlState_s state;
    RobotControlInput_s input = {0};
    RobotControlInit(&state);
    input.remote_online = input.chassis_online = 1;
    input.remote.rc.switch_left = input.remote.rc.switch_right = RC_SW_DOWN;
    RobotControlUpdate(&state, &input);
    CHECK(state.armed && !state.gimbal_armed);

    const uint8_t left[] = {RC_SW_MID, RC_SW_UP, RC_SW_UP, RC_SW_DOWN, RC_SW_DOWN};
    const uint8_t right[] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP, RC_SW_MID, RC_SW_UP};
    // 云台缺失和后来恢复在线两种情况下，五个云台工作档都必须等待新的双下授权。
    for (unsigned online = 0; online < 2; ++online)
    {
        input.gimbal_online = online;
        for (unsigned mode = 0; mode < 5; ++mode)
        {
            input.remote.rc.switch_left = left[mode];
            input.remote.rc.switch_right = right[mode];
            RobotControlUpdate(&state, &input);
            CHECK(state.armed && !state.gimbal_armed && state.control_mode == ROBOT_CONTROL_MODE_SAFE);
            CHECK(state.chassis_cmd.chassis_mode == CHASSIS_ZERO_FORCE && state.chassis_cmd.vx == 0);
            CHECK(state.gimbal_cmd.gimbal_mode == GIMBAL_ZERO_FORCE && state.shoot_cmd.shoot_mode == SHOOT_OFF);
        }
        input.remote.rc.switch_left = RC_SW_UP;
        input.remote.rc.switch_right = RC_SW_DOWN;
        input.remote.rc.rocker_r1 = 100;
        RobotControlUpdate(&state, &input);
        CHECK(state.control_mode == ROBOT_CONTROL_MODE_MOVE && state.chassis_cmd.vx == 600);
    }

    input.remote.rc.switch_left = input.remote.rc.switch_right = RC_SW_DOWN;
    RobotControlUpdate(&state, &input);
    CHECK(state.armed && state.gimbal_armed);
    input.remote.rc.switch_left = RC_SW_UP;
    input.remote.rc.switch_right = RC_SW_MID;
    RobotControlUpdate(&state, &input);
    CHECK(state.control_mode == ROBOT_CONTROL_MODE_SPIN);
    input.gimbal_online = 0;
    RobotControlUpdate(&state, &input);
    CHECK(!state.armed && !state.gimbal_armed && state.control_mode == ROBOT_CONTROL_MODE_SAFE);
    input.gimbal_online = 1;
    RobotControlUpdate(&state, &input);
    CHECK(!state.armed && state.control_mode == ROBOT_CONTROL_MODE_SAFE); // 不自动恢复。

    for (unsigned fault = 0; fault < 3; ++fault)
    {
        input.remote.rc.switch_left = input.remote.rc.switch_right = RC_SW_DOWN;
        RobotControlUpdate(&state, &input);
        CHECK(state.armed && state.gimbal_armed);
        input.remote.rc.switch_left = RC_SW_UP;
        if (fault == 0) input.remote_online = 0;
        if (fault == 1) input.chassis_online = 0;
        if (fault == 2) input.remote.rc.dial = REMOTE_STOP_DIAL_THRESHOLD + 1;
        RobotControlUpdate(&state, &input);
        CHECK(!state.armed && !state.gimbal_armed && state.control_mode == ROBOT_CONTROL_MODE_SAFE);
        input.remote_online = input.chassis_online = 1;
        input.remote.rc.dial = 0;
        RobotControlUpdate(&state, &input);
        CHECK(!state.armed && !state.gimbal_armed && state.control_mode == ROBOT_CONTROL_MODE_SAFE);
    }
    host_call(4, "PASS: switch transit preserves chassis arming; gimbal recovery requires double-down; faults revoke permissions\n");
}

/** @brief 不注入本板电机反馈，仅通过真实DBUS/CAN入口运行普通固件链路。 */
static void StepWithoutGimbal(uint8_t right, uint8_t left, int16_t forward, int16_t lateral, int16_t yaw)
{
    test_tick += 5;
    remote_frame(right, left, forward, lateral, yaw);
    CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask(); GimbalTask(); ShootTask(); DJIMotorControl();
}

/** @brief 验证普通版无云台电机/有效INS时，双下解锁后仍能向底盘发送三轴速度。 */
static void TestDirectChassisWithoutGimbal(void)
{
    RobotControlInit(&robot_control_state);
    test_tick += 25; // 使上次所有本板电机反馈过期。
    test_ins_online = 0;
    test_imu.YawTotalAngle = test_imu.Pitch = NAN;
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, -100, 100);
    check_stopped(); // 左上右下不能替代双下解锁。
    CHECK(!gimbal_control.armed);
    for (unsigned i = 0; i < 2; ++i)
        StepWithoutGimbal(RC_SW_DOWN, RC_SW_DOWN, 0, 0, 0);
    CHECK(gimbal_control.armed && !gimbal_control.gimbal_online);
    // 模拟真实拨杆：左下 -> 左中(比赛) -> 左上，右开关保持下。
    // 即使停在中档多个通信周期，也不得撤销底盘权限或输出运动/发射命令。
    for (unsigned i = 0; i < 30; ++i)
    {
        StepWithoutGimbal(RC_SW_DOWN, RC_SW_MID, 100, -100, 100);
        check_stopped();
        CHECK(gimbal_control.selected_mode == ROBOT_CONTROL_MODE_COMPETITION);
        CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_SAFE && gimbal_control.armed);
        Chassis_Ctrl_Cmd_s stopped;
        CHECK(CANCommSnapshot(peer, &stopped, 50));
        CHECK(stopped.chassis_mode == CHASSIS_ZERO_FORCE && stopped.vx == 0 && stopped.vy == 0 && stopped.wz == 0);
    }
    for (unsigned i = 0; i < 2; ++i)
        StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, -100, 100);
    CHECK(!gimbal_control.bench_active && !gimbal_control.gimbal_online);
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_MOVE);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_REMOTE);
    Chassis_Ctrl_Cmd_s received;
    CHECK(CANCommSnapshot(peer, &received, 50));
    CHECK(received.mode == 0 && received.offset_angle == 0 && received.chassis_spin_speed == 0);
    CHECK(received.chassis_mode == CHASSIS_REMOTE);
    CHECK(fabsf(received.vx - 600) < 0.001f && fabsf(received.vy + 200) < 0.001f);
    CHECK(fabsf(received.wz + 20 * DEGREE_2_RAD) < 0.0001f);
    CHECK(gimbal_control.gimbal_cmd.gimbal_mode == GIMBAL_ZERO_FORCE);
    CHECK(gimbal_yaw_motor->stop_flag == MOTOR_STOP && gimbal_pitch_motor->stop_flag == MOTOR_STOP);
    CHECK(friction_left_motor->stop_flag == MOTOR_STOP && friction_right_motor->stop_flag == MOTOR_STOP);
    CHECK(loader_motor->stop_flag == MOTOR_STOP);
    for (unsigned b = 0; b < 2; ++b)
        for (unsigned g = 0; g < 2; ++g)
            for (unsigned i = 0; i < 8; ++i) CHECK(current_frames[b][g][i] == 0);

    // 中位死区与车体系速度限幅在普通版生效，且不依赖残留的云台角度。
    for (unsigned i = 0; i < 2; ++i)
        StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 5, -5, 5);
    CHECK(CANCommSnapshot(peer, &received, 50));
    CHECK(received.vx == 0 && received.vy == 0 && received.wz == 0);
    for (unsigned i = 0; i < 2; ++i)
        StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, -660, 660, -660);
    CHECK(CANCommSnapshot(peer, &received, 50));
    CHECK(received.vx == -BOARD_MAX_VX_MMPS && received.vy == 1320);
    CHECK(fabsf(received.wz - 132 * DEGREE_2_RAD) < 0.0001f);

    StepWithoutGimbal(RC_SW_MID, RC_SW_UP, 100, 0, 0); check_stopped();
    CHECK(gimbal_control.armed && !robot_control_state.gimbal_armed); // 保留底盘权限，但普通SPIN仍不可执行。
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, 0, 0);
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_MOVE);
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_DOWN, 0, 0, 0);
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, 0, 0);
    test_dial = 301;
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, 0, 0); check_stopped();
    CHECK(!gimbal_control.armed);
    test_dial = 0;
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, 0, 0); check_stopped();
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_DOWN, 0, 0, 0);
    StepWithoutGimbal(RC_SW_DOWN, RC_SW_UP, 100, 0, 0);
    test_tick += 101;
    CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask(); GimbalTask(); ShootTask(); DJIMotorControl(); check_stopped();
    CHECK(!gimbal_control.armed); // 遥控超时仍撤销解锁。
    // 重新接入云台后，从纯底盘档转入SPIN须捕获最新姿态，不能沿用旧云台目标。
    test_ins_online = 1;
    test_imu.YawTotalAngle = 123;
    test_imu.Pitch = 8;
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    test_imu.YawTotalAngle = 234;
    test_imu.Pitch = 12;
    step(RC_SW_DOWN, RC_SW_UP, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    step(RC_SW_MID, RC_SW_UP, 1);
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_SPIN);
    CHECK(gimbal_control.gimbal_cmd.yaw == -234 && gimbal_control.gimbal_cmd.pitch == -12);
    host_call(4, "PASS: normal firmware direct chassis CAN without gimbal/INS, limits, interlocks and safe gimbal reentry\n");
}

/** @brief 区分发送入邮箱失败、原始帧接收、CRC/长度错误和半包超时，不改变原协议。 */
static void TestCommunicationDiagnostics(void)
{
    Chassis_Ctrl_Cmd_s command = {0};
    command.chassis_mode = CHASSIS_REMOTE;
    uint32_t attempts = board_link_instance->diagnostics.tx_message_count;
    uint32_t queued = board_link_instance->diagnostics.tx_frame_queued_count;
    uint32_t failed = board_link_instance->diagnostics.tx_frame_fail_count;
    uint32_t peer_messages = peer->diagnostics.rx_message_count;
    fail_board_transmit = 1;
    BoardTransportSendCommand(&command, 1);
    CHECK(board_link_debug.traffic.tx_message_count == attempts + 1);
    CHECK(board_link_debug.traffic.tx_frame_queued_count == queued);
    CHECK(board_link_debug.traffic.tx_frame_fail_count == failed + 4);
    CHECK(peer->diagnostics.rx_message_count == peer_messages);
    fail_board_transmit = 0;
    BoardTransportSendCommand(&command, 1);
    CHECK(board_link_debug.traffic.tx_frame_queued_count == queued + 4);
    CHECK(peer->diagnostics.rx_message_count == peer_messages + 1);
    CHECK(board_link_debug.can_bus == 1 && board_link_debug.tx_id == 0x312 && board_link_debug.rx_id == 0x311);
    CHECK(board_link_debug.tx_data_len == 26 && board_link_debug.rx_data_len == 28);

    Chassis_Upload_Data_s feedback;
    CANCommSend(peer, (uint8_t *)&peer_feedback);
    CHECK(BoardTransportReadFeedback(&feedback));
    uint32_t frames = board_link_debug.traffic.rx_frame_count;
    uint32_t messages = board_link_debug.traffic.rx_message_count;
    uint32_t crc_errors = board_link_debug.traffic.rx_crc_error_count;
    peer->raw_sendbuf[peer->send_buf_len - 2] ^= 1;
    raw_packet(peer, peer->send_buf_len);
    CHECK(!BoardTransportReadFeedback(&feedback));
    CHECK(board_link_debug.traffic.rx_frame_count == frames + 4);
    CHECK(board_link_debug.traffic.rx_message_count == messages);
    CHECK(board_link_debug.traffic.rx_crc_error_count == crc_errors + 1);

    uint32_t format_errors = board_link_debug.traffic.rx_format_error_count;
    peer->raw_sendbuf[1] ^= 1;
    raw_packet(peer, 8);
    BoardTransportReadFeedback(&feedback);
    CHECK(board_link_debug.traffic.rx_format_error_count == format_errors + 1);
    peer->raw_sendbuf[1] ^= 1;
    raw_packet(peer, 8);
    uint32_t partial_timeouts = board_link_instance->diagnostics.rx_partial_timeout_count;
    test_tick += 21;
    CANCommSend(peer, (uint8_t *)&peer_feedback);
    CHECK(BoardTransportReadFeedback(&feedback));
    CHECK(board_link_debug.traffic.rx_partial_timeout_count == partial_timeouts + 1);
    CHECK(board_link_debug.traffic.rx_message_count == messages + 1);
    CHECK(board_link_debug.traffic.last_rx_dlc == 8);
    CHECK(memcmp(board_link_debug.traffic.last_rx_frame, peer->raw_sendbuf + 24, 8) == 0);
    host_call(4, "PASS: communication diagnostics separate queued/failed TX, raw RX, valid messages, CRC/format errors and partial timeouts\n");
}

void test_main(void)
{
    TestIndependentControlContexts();
    TestChassisOnlyArmingTransitions();
    test_imu.YawTotalAngle = 10;
    test_imu.Pitch = -5;
    RobotCMDInit(); GimbalInit(); ShootInit();
    CHECK(test_can_count == 6);
    CHECK(gimbal_yaw_motor->motor_can_instance->can_handle == &hcan1);
    CHECK(gimbal_yaw_motor->motor_can_instance->rx_id == 0x206);
    CHECK(gimbal_pitch_motor->motor_can_instance->can_handle == &hcan2);
    CHECK(gimbal_pitch_motor->motor_can_instance->rx_id == 0x205);
    CHECK(friction_right_motor->motor_can_instance->can_handle == &hcan2);
    CHECK(friction_right_motor->motor_can_instance->rx_id == 0x206);
    CHECK(friction_left_motor->motor_can_instance->can_handle == &hcan2);
    CHECK(friction_left_motor->motor_can_instance->rx_id == 0x207);
    CHECK(loader_motor->motor_can_instance->can_handle == &hcan2);
    CHECK(loader_motor->motor_can_instance->rx_id == 0x201);
    CHECK(board_link_instance->can_ins->can_handle == &hcan1);
    CANComm_Init_Config_s peer_config = {
        .can_config = {.can_handle = &hcan1, .tx_id = BOARD_CHASSIS_TX_ID, .rx_id = BOARD_GIMBAL_TX_ID},
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s), .daemon_count = 5};
    peer = CANCommInit(&peer_config);
    peer_feedback.motors_online = peer_feedback.ins_online = 1;
    CHECK(!CANCommSnapshot(board_link_instance, &peer_feedback, 100));
    step(RC_SW_DOWN, RC_SW_UP, 0); check_stopped(); // No first peer feedback.
    step(RC_SW_DOWN, RC_SW_UP, 1); check_stopped(); // Right-down alone is MOVE, not unlock.
    CHECK(gimbal_control.selected_mode == ROBOT_CONTROL_MODE_MOVE && !gimbal_control.armed);
    step(RC_SW_DOWN, RC_SW_DOWN, 1); check_stopped();
    CHECK(gimbal_control.armed);
    memset(&gimbal_control, 0xff, sizeof(gimbal_control)); // Watch writes must not change private unlock state.


    const uint8_t positions[3] = {RC_SW_DOWN, RC_SW_MID, RC_SW_UP};
    const RobotControlMode_e expected[3][3] = {
        {ROBOT_CONTROL_MODE_ZERO_FORCE, ROBOT_CONTROL_MODE_FRICTION_ONLY, ROBOT_CONTROL_MODE_FIRE},
        {ROBOT_CONTROL_MODE_COMPETITION, ROBOT_CONTROL_MODE_SAFE, ROBOT_CONTROL_MODE_SAFE},
        {ROBOT_CONTROL_MODE_MOVE, ROBOT_CONTROL_MODE_SPIN, ROBOT_CONTROL_MODE_GIMBAL_FIRE}};
    for (unsigned left = 0; left < 3; ++left)
    {
        for (unsigned right = 0; right < 3; ++right)
        {
            CHECK(SelectRobotControlMode(positions[left], positions[right]) == expected[left][right]);
            step(positions[right], positions[left], 1);
            RobotControlMode_e mode = expected[left][right];
            CHECK(gimbal_control.selected_mode == mode && gimbal_control.control_mode == mode);
            if (mode == ROBOT_CONTROL_MODE_SAFE || mode == ROBOT_CONTROL_MODE_ZERO_FORCE)
            { check_stopped(); continue; }
            CHECK(gimbal_control.gimbal_cmd.gimbal_mode ==
                  (mode == ROBOT_CONTROL_MODE_MOVE ? GIMBAL_ZERO_FORCE : GIMBAL_GYRO_MODE));
            CHECK(fabsf(gimbal_control.chassis_cmd.offset_angle) < 0.001f);
            if (mode == ROBOT_CONTROL_MODE_MOVE)
                CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_REMOTE && gimbal_control.chassis_cmd.mode == 0);
            else if (mode == ROBOT_CONTROL_MODE_SPIN)
                CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ROTATE);
            else
                CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_NO_FOLLOW);
            if (mode == ROBOT_CONTROL_MODE_COMPETITION || mode == ROBOT_CONTROL_MODE_GIMBAL_FIRE)
                CHECK(gimbal_control.chassis_cmd.vx == 0 && !gimbal_control.vision_online);
            else CHECK(fabsf(gimbal_control.chassis_cmd.vx - 600) < 0.001f);
            uint8_t shooting = mode == ROBOT_CONTROL_MODE_FRICTION_ONLY || mode == ROBOT_CONTROL_MODE_FIRE;
            CHECK(gimbal_control.shoot_cmd.shoot_mode == (shooting ? SHOOT_ON : SHOOT_OFF));
            CHECK(gimbal_control.shoot_cmd.friction_mode == (shooting ? FRICTION_ON : FRICTION_OFF));
            CHECK(gimbal_control.shoot_cmd.load_mode == (mode == ROBOT_CONTROL_MODE_FIRE ? LOAD_BURSTFIRE : LOAD_STOP));
        }
    }
    CHECK(SelectRobotControlMode(0, RC_SW_UP) == ROBOT_CONTROL_MODE_SAFE);
    CHECK(SelectRobotControlMode(RC_SW_DOWN, 0) == ROBOT_CONTROL_MODE_SAFE);
    step(RC_SW_MID, RC_SW_UP, 1);
    Chassis_Ctrl_Cmd_s decoded;
    CHECK(CANCommSnapshot(peer, &decoded, 50));
    CHECK(decoded.chassis_mode == CHASSIS_ROTATE && decoded.mode == 2);
    CHECK(fabsf(decoded.chassis_spin_speed - 120 * DEGREE_2_RAD) < 0.001f);
    float yaw_before_watch_write = gimbal_control.gimbal_cmd.yaw;
    memset(&gimbal_control, 0xff, sizeof(gimbal_control));
    memset(&board_link_debug, 0xff, sizeof(board_link_debug));
    step(RC_SW_MID, RC_SW_UP, 1);
    CHECK(gimbal_control.armed && gimbal_control.control_mode == ROBOT_CONTROL_MODE_SPIN);
    CHECK(gimbal_control.gimbal_cmd.yaw == yaw_before_watch_write);
    CHECK(board_link_debug.rx_online == 1);
    step(RC_SW_MID, RC_SW_MID, 1); check_stopped();
    CHECK(CANCommSnapshot(peer, &decoded, 50) && decoded.chassis_mode == CHASSIS_ZERO_FORCE);
    host_call(4, "PASS: rm-sentry nine switch combinations, mode cleanup, double-down unlock and real CAN commands\n");

    friction_rpm = 5000;
    for (unsigned i = 0; i < 70; ++i) step(RC_SW_MID, RC_SW_DOWN, 1); // Friction-only.
    CHECK(shoot_feedback.friction_ready && !shoot_feedback.loader_running);
    step(RC_SW_UP, RC_SW_DOWN, 1); // FIRE requested, but no actual referee heat.
    CHECK(!shoot_feedback.loader_running);
    peer_feedback.referee_online = 1;
    peer_feedback.heat_limit = 200;
    peer_feedback.current_heat = 20;
    step(RC_SW_UP, RC_SW_DOWN, 1);
    CHECK(shoot_feedback.loader_running && loader_motor->motor_controller.pid_ref == 10000);
    peer_feedback.current_heat = 180;
    step(RC_SW_UP, RC_SW_DOWN, 1);
    CHECK(loader_motor->stop_flag == MOTOR_STOP);
    step(RC_SW_DOWN, RC_SW_UP, 1); // MOVE closes the complete shooter.
    CHECK(friction_left_motor->stop_flag == MOTOR_STOP && friction_right_motor->stop_flag == MOTOR_STOP && loader_motor->stop_flag == MOTOR_STOP);
    step(RC_SW_DOWN, RC_SW_DOWN, 1); check_stopped();
    CHECK(CANCommSnapshot(peer, &decoded, 50) && decoded.chassis_mode == CHASSIS_ZERO_FORCE);
    host_call(4, "PASS: friction-only/fire separation, real heat interlock and all-five-motor zero force\n");

    step(RC_SW_DOWN, RC_SW_UP, 1);
    test_tick += 101;
    step(RC_SW_DOWN, RC_SW_UP, 0); check_stopped(); // CAN stale, RC fresh.
    step(RC_SW_DOWN, RC_SW_UP, 1); check_stopped(); // MOVE does not re-arm.
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    test_ins_online = 0;
    step(RC_SW_DOWN, RC_SW_UP, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_MOVE && !gimbal_control.gimbal_online);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_REMOTE);
    step(RC_SW_MID, RC_SW_UP, 1); check_stopped(); // SPIN仍要求云台权限，不能借用底盘权限运行。
    CHECK(gimbal_control.armed && !robot_control_state.gimbal_armed);
    test_ins_online = 1;
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    test_tick += 101;
    all_feedback(); CANCommSend(peer, (uint8_t *)&peer_feedback);
    RobotCMDTask(); GimbalTask(); ShootTask(); DJIMotorControl(); check_stopped(); // RC stale.
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_MID, RC_SW_DOWN, 1);
    test_tick += 21; all_feedback(); DJIMotorControl();
    for (unsigned b = 0; b < 2; ++b)
        for (unsigned g = 0; g < 2; ++g)
            for (unsigned i = 0; i < 8; ++i) CHECK(current_frames[b][g][i] == 0);
    test_dial = 301;
    step(RC_SW_DOWN, RC_SW_UP, 1); check_stopped();
    CHECK(!gimbal_control.armed);
    test_dial = 0;
    step(RC_SW_DOWN, RC_SW_UP, 1); check_stopped();
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(0, RC_SW_UP, 1); check_stopped(); // Invalid switch revokes arming.
    CHECK(!gimbal_control.armed);
    host_call(4, "PASS: CAN/RC/INS loss, dial emergency stop, invalid switches and independent target watchdog\n");

    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    step(RC_SW_DOWN, RC_SW_UP, 1);
    peer->raw_sendbuf[peer->send_buf_len - 2] ^= 1;
    raw_packet(peer, peer->send_buf_len);
    step(RC_SW_DOWN, RC_SW_UP, 0); check_stopped();
    step(RC_SW_DOWN, RC_SW_UP, 1); check_stopped();
    raw_packet(peer, 8);
    test_tick += 21;
    step(RC_SW_DOWN, RC_SW_DOWN, 1);
    CHECK(gimbal_control.chassis_online && gimbal_control.armed);
    uint32_t received_at = board_link_instance->last_rx_ms;
    test_tick += 5;
    Chassis_Upload_Data_s snapshot;
    CHECK(CANCommSnapshot(board_link_instance, &snapshot, 100));
    CHECK(board_link_instance->last_rx_ms == received_at);
    host_call(4, "PASS: corrupt CRC stop, dropped-fragment recovery and unchanged receive timestamp\n");

    peer_feedback.current_heat = 20;
    peer_feedback.current_hp = 100;
    vision_frame(1, 1, 1);
    step(RC_SW_DOWN, RC_SW_MID, 1); // COMPETITION.
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_COMPETITION && gimbal_control.vision_online);
    CHECK(gimbal_control.chassis_cmd.mode == 1 && gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ROTATE);
    CHECK(fabsf(gimbal_control.chassis_cmd.vx + 700) < 0.001f);
    CHECK(fabsf(gimbal_control.chassis_cmd.vy - 350) < 0.001f);
    CHECK(fabsf(gimbal_control.chassis_cmd.chassis_spin_speed - 1.0f) < 0.001f);
    CHECK(fabsf(gimbal_control.gimbal_cmd.yaw + 0.65f) < 0.001f);
    CHECK(fabsf(gimbal_control.gimbal_cmd.pitch - 16.5f) < 0.001f);
    CHECK(gimbal_control.shoot_cmd.load_mode == LOAD_BURSTFIRE);
    CHECK(!shoot_feedback.loader_running); // Newly enabled friction must spin up.
    test_imu.YawTotalAngle = 720;
    step(RC_SW_DOWN, RC_SW_MID, 1); // Publish the new feedback for CMD.
    vision_frame(1, 1, 1);
    test_vision.cmd.data.gimbal.yaw = 0.1f;
    step(RC_SW_UP, RC_SW_UP, 1); // GIMBAL_FIRE.
    CHECK(gimbal_control.control_mode == ROBOT_CONTROL_MODE_GIMBAL_FIRE);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ZERO_FORCE && gimbal_control.chassis_cmd.mode == 2);
    CHECK(CANCommSnapshot(peer, &decoded, 50) && decoded.chassis_mode == CHASSIS_ZERO_FORCE);
    CHECK(fabsf(gimbal_control.gimbal_cmd.yaw + 720) < 180);
    CHECK(gimbal_control.shoot_cmd.shoot_mode == SHOOT_ON);
    CHECK(gimbal_control.shoot_cmd.load_mode == LOAD_BURSTFIRE);
    vision_frame(1, 1, 1);
    test_vision.cmd.data.speed_vector.vx = NAN;
    step(RC_SW_DOWN, RC_SW_MID, 1);
    CHECK(!gimbal_control.vision_online && gimbal_control.chassis_cmd.vx == 0);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_NO_FOLLOW);
    CHECK(gimbal_control.shoot_cmd.shoot_mode == SHOOT_OFF);
    vision_frame(1, 1, 1);
    test_vision.cmd.data.speed_vector.vx = 100;
    test_vision.cmd.data.speed_vector.vy = -100;
    test_vision.cmd.data.speed_vector.wz = 100;
    test_vision.cmd.data.gimbal.pitch = 100;
    step(RC_SW_DOWN, RC_SW_MID, 1);
    CHECK(gimbal_control.chassis_cmd.vx == -BOARD_MAX_VX_MMPS);
    CHECK(gimbal_control.chassis_cmd.vy == -BOARD_MAX_VY_MMPS);
    CHECK(gimbal_control.chassis_cmd.chassis_spin_speed == BOARD_MAX_WZ_RADPS);
    CHECK(gimbal_control.gimbal_cmd.pitch == -GIMBAL_PITCH_LIMIT_DEG);
    vision_frame(1, 0, 0);
    step(RC_SW_DOWN, RC_SW_MID, 1);
    uint32_t vision_received_at = test_vision_rx_ms;
    float held_yaw = gimbal_control.gimbal_cmd.yaw;
    float held_pitch = gimbal_control.gimbal_cmd.pitch;
    for (unsigned i = 0; i < 23; ++i) step(RC_SW_DOWN, RC_SW_MID, 1);
    CHECK(gimbal_control.vision_online); // Exactly 120ms, with a different DWT/HAL epoch.
    step(RC_SW_DOWN, RC_SW_MID, 1);
    CHECK(!gimbal_control.vision_online && test_vision_rx_ms == vision_received_at);
    CHECK(gimbal_control.chassis_cmd.chassis_mode == CHASSIS_NO_FOLLOW);
    CHECK(gimbal_control.chassis_cmd.vx == 0 && gimbal_control.chassis_cmd.chassis_spin_speed == 0);
    CHECK(gimbal_control.shoot_cmd.shoot_mode == SHOOT_OFF && loader_motor->stop_flag == MOTOR_STOP);
    CHECK(gimbal_control.gimbal_cmd.yaw == held_yaw && gimbal_control.gimbal_cmd.pitch == held_pitch);
    vision_frame(0, 0, 0);
    step(RC_SW_DOWN, RC_SW_MID, 1);
    CHECK(gimbal_control.vision_online && gimbal_control.chassis_cmd.chassis_mode == CHASSIS_ROTATE);
    CHECK(fabsf(gimbal_control.gimbal_cmd.yaw - held_yaw - PATROL_YAW_STEP_DEG) < 0.001f);
    CHECK(fabsf(gimbal_control.gimbal_cmd.pitch - held_pitch - PATROL_PITCH_STEP_DEG) < 0.001f);
    step(RC_SW_DOWN, RC_SW_DOWN, 1); check_stopped();
    host_call(4, "PASS: competition/gimbal-fire vision routing, patrol, finite checks, limits and DWT-based timeout\n");
    TestDirectChassisWithoutGimbal();
    TestCommunicationDiagnostics();
    finish(0);
}
