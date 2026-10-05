/**
 * @file    gimbal_pitch.h
 * @brief   国赛机械Pitch的IMU内环、周期积分观测、握手及退出快照。
 */
#ifndef UP_GIMBAL_PITCH_H
#define UP_GIMBAL_PITCH_H

/* Includes ------------------------------------------------------------------*/
#include "pitch_config.h"
#include "board_pitch_protocol.h"

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    PITCH_PROBE, PITCH_WAIT, PITCH_ZEROING, PITCH_ENABLING, PITCH_ACTIVE, PITCH_STOPPING, PITCH_FAULT
} pitch_state_t; /**< 0探测、1待启动、2零力矩确认、3使能、4控制、5失能确认、6退出。 */
typedef enum {
    PITCH_LOOP_SPEED, PITCH_LOOP_ANGLE
} pitch_loop_mode_t; /**< 0直接给速度目标，1角度外环产生速度目标，两种模式共用速度内环。 */
typedef enum {
    PITCH_REASON_NONE, PITCH_REASON_INTERLOCK, PITCH_REASON_MOTOR, PITCH_REASON_SPEED,
    PITCH_REASON_PERIOD, PITCH_REASON_TIMEOUT, PITCH_REASON_CAN, PITCH_REASON_MODE, PITCH_REASON_IMU
} pitch_reason_t; /**< 保留本次退出原因，恢复通信不自动启动。 */
enum {
    PITCH_BLOCK_PARAMS = 1U, PITCH_BLOCK_FEEDBACK = 2U, PITCH_BLOCK_BOARD = 4U,
    PITCH_BLOCK_ENABLE = 8U, PITCH_BLOCK_SWITCH = 16U, PITCH_BLOCK_NEUTRAL = 32U,
    PITCH_BLOCK_LIMIT = 64U, PITCH_BLOCK_CAN = 128U, PITCH_BLOCK_IMU = 256U
}; /**< 参数/状态帧/板间帧/许可/拨杆/回中/限位/CAN阻塞位。 */
enum {
    PITCH_TX_QUERY, PITCH_TX_ZERO, PITCH_TX_ENABLE, PITCH_TX_TORQUE, PITCH_TX_DISABLE
}; /**< 用于区分真实发送确认的电机帧类型。 */
typedef struct {
    pitch_state_t state; /**< 进入失能处理前的阶段。 */
    pitch_loop_mode_t loop_mode; /**< 退出前已锁存的速度/角度模式。 */
    pitch_reason_t reason; /**< 触发此次失能的原因。 */
    uint32_t ms; /**< 进入失能处理的系统时刻，毫秒。 */
    uint32_t feedback_ms; /**< 所用电机反馈时刻，毫秒。 */
    float target_deg, actual_deg; /**< 退出前目标与实际角，度。 */
    float speed_target_dps, speed_dps; /**< 退出前速度目标与选定IMU反馈速度，度/秒。 */
    float speed_error_dps; /**< 退出前速度目标减回报，度/秒。 */
    float encoder_speed_dps; /**< 相邻有时间间隔反馈的角度差分平均速度，度/秒，仅用于诊断。 */
    float torque; /**< 清零前算法拟输出力矩，MIT标度，非已确认发送值。 */
    float angle_integral_dps; /**< 退出前角度积分输出，度/秒。 */
    float gravity_torque, speed_loop_torque; /**< 退出前补偿与已限幅速度PI贡献，电机原生MIT标度。 */
    float input_rate_dps; /**< 退出时下板目标角变化速率，度/秒。 */
    uint8_t encoder_speed_valid; /**< 角度差分具有非零且新鲜的时间间隔。 */
    uint8_t raw[8]; /**< 退出时电机状态帧，后续停止反馈不覆盖。 */
} pitch_exit_sample_t;
typedef struct {
    pitch_state_t state; /**< 当前阶段。 */
    pitch_loop_mode_t loop_mode; /**< 本次启动锁存的实际模式；启动后改变请求模式会退出。 */
    pitch_reason_t reason; /**< 最近退出原因。 */
    uint32_t blocks; /**< 当前阻塞位，按位相加。 */
    uint32_t exit_blocks; /**< 本次退出时阻塞位快照。 */
    uint32_t frames; /**< 合法电机状态帧累计数，不含参数回读。 */
    uint32_t parameter_frames; /**< 参数回读帧累计数。 */
    uint32_t board_frames; /**< 新序号的合法下板命令累计数。 */
    uint32_t last_ms; /**< 最近电机状态帧时刻，毫秒。 */
    uint32_t board_last_ms; /**< 最近新序号板间命令时刻，毫秒。 */
    uint32_t sessions; /**< 已接受的拨杆启动会话数。 */
    uint32_t active_command_frames; /**< 收到的合法ACTIVE命令累计数，停止档不清除。 */
    uint32_t unarmed_active_frames; /**< 待启动且尚未记录停止许可时收到ACTIVE的累计帧数，包含探测阶段。 */
    uint32_t start_attempts; /**< 已记录停止许可后检查ACTIVE启动条件的累计次数。 */
    uint32_t start_blocks; /**< 最近一次启动检查的阻塞位，停止后保留；位定义同blocks。 */
    uint32_t start_flags; /**< 最近一次启动检查时的下板flags，停止后保留。 */
    uint32_t active_entries; /**< 完成零输出/使能握手进入实际闭环的累计次数。 */
    float peak_input_rate_dps; /**< 合法ACTIVE命令中目标角变化速率的绝对峰值，度/秒，上电累计。 */
    float peak_torque; /**< 控制算法拟输出力矩的绝对峰值，MIT标度，非电机实测力矩，上电累计。 */
    uint32_t tx_confirmed; /**< 控制器报告的电机帧发送成功数。 */
    uint32_t tx_errors; /**< 电机帧发送失败数。 */
    uint32_t parameter_mask; /**< PMAX/VMAX/TMAX/MIT模式回读位，完整为15。 */
    uint32_t control_mode; /**< 电机CTRL_MODE回读值，MIT必须为1。 */
    float pmax; /**< 电机PMAX回读幅值，弧度。 */
    float vmax; /**< 电机VMAX回读幅值，弧度/秒。 */
    float tmax; /**< 电机TMAX回读幅值，协议力矩单位。 */
    float motor_rad; /**< 由实测协议幅值解码的原生电机角，弧度。 */
    float actual_deg; /**< 国赛零点/方向处理后的机械相对角，度。 */
    float target_deg; /**< 本次连续变化且限幅的目标机械角，度。 */
    float goal_deg; /**< 回平射的最终目标或手动目标，度；待机跟随实际角。 */
    float error_deg; /**< 目标减实际角，度。 */
    float speed_dps; /**< 内环反馈，当前为国赛变换后的IMU速度，度/秒。 */
    float motor_speed_dps; /**< 电机原生速度解码后取机械方向，仅对照，度/秒。 */
    float imu_pitch_deg, imu_speed_dps; /**< 国赛坐标IMU俯仰角和角速度，度、度/秒。 */
    uint32_t imu_last_ms, imu_updates; /**< 最近有效IMU时刻及有效采样次数，毫秒、次数。 */
    uint8_t imu_ready; /**< 最近样本通过校准/传输/有限值检查；还需检查新鲜度。 */
    float encoder_speed_dps; /**< 相邻反馈角差分速度，度/秒；诊断用，不参与控制或保护。 */
    float speed_target_dps; /**< 速度单环的输入目标或角度PI输出，度/秒。 */
    float speed_error_dps; /**< 速度目标减选定IMU反馈速度，度/秒。 */
    float torque; /**< 拟发送MIT力矩，按回读TMAX标度，带原生电机方向。 */
    float feedback_torque; /**< 电机协议力矩反馈，并非外部实测力矩。 */
    float integral; /**< 速度PI积分输出，按TMAX标度。 */
    float angle_integral_dps; /**< 角度PI积分输出，度/秒；速度模式、捕获目标及退出清零。 */
    float angle_integral_sum; /**< 国赛按周期累加的角误差，度乘采样次数。 */
    float speed_integral_sum; /**< 国赛按周期累加的速度误差，度/秒乘采样次数。 */
    float gravity_torque; /**< 加在原生电机方向上的重力补偿，MIT标度。 */
    float speed_loop_torque; /**< 已限幅速度PI的原生方向贡献，补偿相加前，MIT标度。 */
    uint8_t motor_state; /**< 反馈高半字节：0失能，1使能，8及以上错误。 */
    uint8_t mos_temperature; /**< 电机MOS反馈温度，摄氏度。 */
    uint8_t rotor_temperature; /**< 电机线圈反馈温度，摄氏度。 */
    uint8_t raw[8]; /**< 最近电机状态原始帧。 */
    uint8_t online; /**< 参数解码有效且电机状态新鲜。 */
    uint8_t board_online; /**< 下板新序号命令新鲜。 */
    uint8_t stop_unconfirmed; /**< 失能发送后仍无新鲜失能回复，继续尝试。 */
    uint8_t flat_return; /**< 中档首次进入控制后正在渐变目标至平射；手动输入或退出取消。 */
    uint8_t encoder_speed_valid; /**< 角差分速度的时间间隔有效。 */
    board_pitch_command_t command; /**< 最近合法下板命令，观察用。 */
    board_pitch_command_t last_active_command; /**< 最近合法ACTIVE命令快照，后续停止档不覆盖。 */
    pitch_exit_sample_t exit_sample; /**< 最近进入失能处理前的快照，保留至下一次失能或复位。 */
} gimbal_pitch_t;

/* Exported variables --------------------------------------------------------*/
extern gimbal_pitch_t gimbal_pitch; /**< 上板Watch观察对象。 */
extern volatile uint32_t gimbal_pitch_enable; /**< 输出许可，默认1；不绕过参数、反馈和拨杆检查。 */
extern volatile uint32_t gimbal_pitch_speed_guard_enable; /**< 电机速度阈值退出开关，上电取配置默认值；不影响其他退出条件。 */
extern volatile uint32_t gimbal_pitch_loop_mode; /**< 用户模式请求：0速度单环、1角度串级；停止档修改后重新启动。 */
extern volatile float pitch_scope_target_deg, pitch_scope_actual_deg, pitch_scope_error_deg; /**< J-Scope角度镜像，度。 */
extern volatile float pitch_scope_target_dps, pitch_scope_speed_dps, pitch_scope_torque; /**< J-Scope目标/反馈速度和拟输出力矩镜像。 */
extern volatile float pitch_scope_goal_deg, pitch_scope_encoder_dps; /**< J-Scope最终角目标及反馈角差分速度，度、度/秒。 */
extern volatile float pitch_scope_speed_error_dps; /**< J-Scope速度环目标减反馈误差，度/秒。 */
extern volatile float pitch_scope_angle_integral_dps; /**< J-Scope角度积分输出，度/秒。 */
extern volatile float pitch_scope_gravity_torque, pitch_scope_speed_loop_torque; /**< J-Scope补偿与已限幅速度PI贡献，原生MIT标度。 */
extern volatile float pitch_scope_motor_dps, pitch_scope_imu_deg; /**< 电机速度对照和IMU重力角，度/秒、度。 */
extern volatile uint32_t pitch_scope_imu_ready; /**< IMU样本已校准且新鲜，1可用、0不可用。 */

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化只读探测和配置默认许可，不立即使能电机。 */
void Gimbal_Pitch_Init(void);
/** @brief 发布国赛坐标IMU样本；未校准、无效或非有限样本立即撤销许可。 */
void Gimbal_Pitch_ImuUpdate(float pitch_deg, float speed_dps, uint32_t now, int ready);
/** @brief 接收精确ID的电机/参数或版本化下板帧；调用者应保护任务与IRQ的访问。 */
int Gimbal_Pitch_Receive(uint32_t id, const uint8_t *data, unsigned length, uint32_t now);
/** @brief 更新机械串级与故障退出，需由有界周期任务调用。 */
void Gimbal_Pitch_Update(uint32_t now, int can_ready);
/** @brief 生成当前到期电机帧，返回1；只读/使能/失能/纯力矩显式区分。 */
int Gimbal_Pitch_MakeFrame(uint32_t now, uint32_t *id, uint8_t data[8], uint8_t *kind);
/** @brief 成功入队后更新发送节拍，不能当作已到达电机。 */
void Gimbal_Pitch_Queued(uint32_t now, uint8_t kind);
/** @brief 记录实际发送完成，零力矩确认后才允许使能阶段。 */
void Gimbal_Pitch_TxComplete(uint32_t now, uint8_t kind, int success);
/** @brief 当前是否要求取消已排队的使能/非零输出帧。 */
int Gimbal_Pitch_NeedsStop(void);
#endif
