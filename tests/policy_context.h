/* The policy must not share arming or continuous targets across instances.
 * CHECK is provided by the existing normal/bench integration harness. */
static void TestIndependentControlContexts(void)
{
    RobotControlState_s first;
    RobotControlState_s second;
    RobotControlInput_s input = {0};
    RobotControlInit(&first);
    RobotControlInit(&second);
    input.remote_online = 1;
    input.chassis_online = 1;
    input.gimbal_online = 1;
    input.gimbal_feedback.gimbal_imu_data.YawTotalAngle = 10;
    input.gimbal_feedback.gimbal_imu_data.Pitch = -5;
    input.remote.rc.switch_left = RC_SW_DOWN;
    input.remote.rc.switch_right = RC_SW_DOWN;
    RobotControlUpdate(&first, &input);
    CHECK(first.armed && !second.armed);

    input.remote.rc.switch_left = RC_SW_UP;
    input.remote.rc.switch_right = RC_SW_DOWN;
    input.remote.rc.rocker_r1 = 100;
    RobotControlUpdate(&second, &input);
    CHECK(second.control_mode == ROBOT_CONTROL_MODE_SAFE && !second.armed);
    unsigned char second_snapshot[sizeof(second)];
    memcpy(second_snapshot, &second, sizeof(second));
    RobotControlUpdate(&first, &input);
    CHECK(first.control_mode == ROBOT_CONTROL_MODE_MOVE);
    CHECK(first.chassis_cmd.vx == 600.0f);
    CHECK(memcmp(second_snapshot, &second, sizeof(second)) == 0);

    unsigned char first_snapshot[sizeof(first)];
    memcpy(first_snapshot, &first, sizeof(first));
    input.chassis_online = 0;
    RobotControlUpdate(&second, &input);
    CHECK(memcmp(first_snapshot, &first, sizeof(first)) == 0);
    CHECK(first.armed && !second.armed);
}
