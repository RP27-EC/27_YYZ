/**
 * @file    gimbal.h
 * @brief   Yaw遥控角度/速度串级控制、独立测试及实时诊断接口。
 */
#ifndef DOWN_GIMBAL_H
#define DOWN_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gimbal_config.h"

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
    GIMBAL_IDLE, GIMBAL_ZEROING, GIMBAL_PULSE, GIMBAL_HOLD,
    GIMBAL_STOPPING, GIMBAL_DONE, GIMBAL_FAULT, GIMBAL_WAIT_REMOTE, GIMBAL_SPEED, GIMBAL_REMOTE
} gimbal_yaw_state_t; /**< 0空闲、1清零、2点动、3独立角度、4停止、5完成、6故障、7等待、8独立速度、9遥控控制。 */
typedef enum {
    GIMBAL_REASON_NONE, GIMBAL_REASON_COMPLETE, GIMBAL_REASON_DISABLED,
    GIMBAL_REASON_INTERLOCK, GIMBAL_REASON_FEEDBACK, GIMBAL_REASON_DIRECTION,
    GIMBAL_REASON_REQUEST, GIMBAL_REASON_SPEED, GIMBAL_REASON_TRAVEL,
    GIMBAL_REASON_CALIBRATION, GIMBAL_REASON_CAN, GIMBAL_REASON_ZERO_TIMEOUT,
    GIMBAL_REASON_PERIOD, GIMBAL_REASON_REMOTE_TIMEOUT
} gimbal_yaw_reason_t; /**< 退出原因；数值及操作见Yaw控制文档。 */
enum {
    GIMBAL_BLOCK_CHASSIS_ENABLE = 1U, /**< 底盘许可不符：遥控需1，独立测试需0。 */
    GIMBAL_BLOCK_CHASSIS_MODE = 2U,   /**< 底盘模式不符：遥控需mec，独立测试需sleep。 */
    GIMBAL_BLOCK_WHEEL_OUTPUT = 4U,   /**< 至少一个四轮电流指令未清零。 */
    GIMBAL_BLOCK_REMOTE = 8U,         /**< 遥控无有效帧或已超时。 */
    GIMBAL_BLOCK_SWITCH = 16U,        /**< 右拨杆不符：遥控需中档，独立测试需上/下。 */
    GIMBAL_BLOCK_STICKS = 32U,        /**< 任一摇杆偏离中位超过30原始值。 */
    GIMBAL_BLOCK_KEYS = 64U,          /**< 启动按键/鼠标未释放；独立测试要求WASD松开。 */
    GIMBAL_BLOCK_CAN = 128U,          /**< CAN状态或上次底盘发送异常。 */
    GIMBAL_BLOCK_INPUT_SOURCE = 256U  /**< 运动中切换遥控/键鼠输入来源。 */
};
typedef struct {
    gimbal_yaw_state_t state; /**< 当前状态。 */
    gimbal_yaw_reason_t reason; /**< 最近退出或启动拒绝原因。 */
    uint32_t request;         /**< 本次会话：1正点动、2负点动、3角度、4独立速度、5遥控自动启动。 */
    uint32_t interlock_block_reason; /**< 原联锁条件诊断位；非零本身不再禁止控制，输入源变化位仍用于退出。 */
    uint32_t start_block_reason; /**< 最近一次请求开始检查时的故障位快照。 */
    uint32_t exit_block_reason; /**< 最近会话因条件失效退出时的故障位快照。 */
    uint32_t wait_remote_ms;   /**< 本次启动等待已用时间，毫秒；结束后保留。 */
    uint32_t duration_ms;     /**< 本次锁存时长，毫秒；请求4/5允许0为持续运行。 */
    uint32_t active_ms;       /**< 本次运动控制阶段已运行时间，毫秒；退出后保留。 */
    uint32_t pulse_ms;        /**< 本次锁存的点动时长，毫秒；请求3/4为0。 */
    int16_t pulse_raw;        /**< 本次锁存的正点动幅度，原始值；请求3/4为0。 */
    uint32_t sessions;        /**< 成功开始清零的测试次数。 */
    uint32_t tx_queued;       /**< 运动帧入队次数。 */
    uint32_t tx_confirmed;    /**< 控制器报告运动帧发送成功次数。 */
    uint32_t tx_errors;       /**< 运动帧入队、传输或取消异常次数。 */
    uint32_t zero_confirmed;  /**< 本次清零阶段确认发送完成的零电流帧数。 */
    uint32_t nonzero_confirmed; /**< 本次非零A1电流帧发送确认次数；不是电机执行确认。 */
    uint32_t drive_reply_frames; /**< 非零帧开始入队后、运动阶段收到的A1回复数。 */
    uint32_t best_settled_ms;  /**< 本次闭环最长连续到位时间，毫秒，结束后保留。 */
    int16_t peak_command_raw; /**< 本次入队电流绝对值最大的一次指令，保留符号，原始值。 */
    int16_t peak_current_raw; /**< 运动阶段A1反馈电流绝对值最大的一次快照，保留符号。 */
    int16_t peak_speed_dps;   /**< 运动阶段A1反馈速度绝对值最大的一次快照，度/秒。 */
    float start_deg;          /**< 本次测试起始连续角，度。 */
    float actual_deg;         /**< 本周期机械连续角或陀螺仪IMU连续角，度。 */
    float target_deg;         /**< 连续目标角，度；独立角度经斜坡，遥控角度按输入累计。 */
    float goal_deg;           /**< 最终目标角，度；遥控角度与target_deg同步。 */
    float error_deg;          /**< 目标减实际角度，度。 */
    float speed_target_dps;   /**< 当前角度/独立速度/遥控速度环目标，度/秒。 */
    float speed_estimate_dps; /**< 机械模式编码器差分滤波速度，陀螺仪模式上板IMU速度，度/秒。 */
    float speed_integral_raw; /**< 速度环积分输出，原始电流值；停止后清零。 */
    float speed_setpoint_dps; /**< 请求4锁存目标或遥控/键鼠实时目标，度/秒。 */
    float speed_kp;           /**< 本轮锁存的速度Kp，原始值/(度/秒)。 */
    float speed_ki;           /**< 本轮锁存的速度Ki，原始值/度。 */
    uint8_t gyro_mode; /**< 本次使用上板IMU世界角和速度，机械模式为0。 */
    uint32_t imu_generation, car_session; /**< 本次IMU连续参考及整车启动代次。 */
    float angle_integral_sum; /**< 陀螺仪外环逐周期角误差累计，度乘次数。 */
    uint32_t turn_count; /**< 换头请求接受次数，默认每次目标增加180度。 */
    uint32_t turn_completed, turn_timeouts; /**< 换头到位及等待超时次数；超时不清除目标或失能。 */
    uint32_t angle_control;   /**< 本轮遥控是否启用角度外环，0直接速度/1角度串级。 */
    float angle_kp;           /**< 本轮角度外环Kp，1/秒。 */
    float angle_effective_kp; /**< 本周期角度外环实际Kp，1/秒；机械遥控随误差变化。 */
    float control_end_deg;    /**< 转入停止清零之前的连续角，度，结束后保留。 */
    float control_end_error_deg; /**< 转入停止时最终目标减实际角，度，结束后保留。 */
    float last_delta_deg;     /**< 上次退出时相对起点位移，度，用于方向确认。 */
    int16_t current_raw;      /**< 当前拟发送电流指令，不是安培。 */
    uint8_t tx_pending;       /**< 运动帧尚在发送/取消队列。 */
    uint8_t stop_unconfirmed; /**< 停止等待超过阈值仍未确认，继续零输出。 */
    uint8_t standby_pending; /**< 遥控离线后的零输出确认尚未完成。 */
    uint32_t feedback_age_ms, error_age_ms, imu_age_ms; /**< 最后有效反馈、错误状态和IMU年龄，毫秒。 */
    uint32_t feedback_warnings, period_errors, invalid_requests; /**< 数据过期、周期异常和丢弃非法请求的观察次数。 */
    uint32_t zero_timeouts, restart_attempts, reference_rebases; /**< 清零重试超时、自动启动及参考重建次数。 */
    uint32_t overspeed_ms, overspeed_samples, overspeed_trips; /**< 连续新鲜反馈超速时长毫秒、超速观察和停机次数。 */
    float overspeed_limit_dps; /**< 本次超速判断上限，度/秒；持续超限停机后等待有效反馈恢复。 */
} gimbal_yaw_t;

/* Exported variables --------------------------------------------------------*/
extern gimbal_yaw_t gimbal_yaw; /**< Watch中的Yaw控制状态。 */
extern volatile uint32_t gimbal_yaw_enable; /**< Yaw输出许可，默认1，只有1有效。 */
extern volatile uint32_t gimbal_yaw_remote_enable; /**< 遥控/键鼠入口，默认1；关闭后可使用独立测试。 */
extern volatile uint32_t gimbal_yaw_angle_enable; /**< 遥控角度外环许可，默认配置1；运行中改变会退出。 */
extern volatile uint32_t gimbal_yaw_request; /**< 一次触发请求，处理后清0。 */
extern volatile float gimbal_yaw_offset_deg; /**< 请求3的目标改变量，允许[-5,5]度。 */
extern volatile int32_t gimbal_yaw_output_direction; /**< 已验证电流方向，默认1，闭环需±1。 */
extern volatile int32_t gimbal_yaw_pulse_raw; /**< 点动幅度，允许1..500原始值，初始化200。 */
extern volatile uint32_t gimbal_yaw_pulse_ms; /**< 点动时长，允许20..1000毫秒，初始化500。 */
extern volatile float gimbal_yaw_speed_dps; /**< 请求4目标速度，无固定幅度上限，必须为有限数，度/秒，默认0。 */
extern volatile float gimbal_yaw_speed_kp; /**< 请求4的下一轮Kp，允许[0,50]，默认20。 */
extern volatile float gimbal_yaw_speed_ki; /**< 请求4的下一轮Ki，允许[0,100]，默认0。 */
extern volatile uint32_t gimbal_yaw_speed_ms; /**< 请求4时长，0持续运行，1..600000定时毫秒，默认0。 */
extern volatile float yaw_scope_target_dps; /**< J-Scope只读镜像：独立/遥控目标速度，度/秒。 */
extern volatile float yaw_scope_speed_dps; /**< J-Scope只读镜像：滤波反馈速度，度/秒。 */
extern volatile float yaw_scope_current_raw; /**< J-Scope只读镜像：拟发送电流原始指令。 */
extern volatile float yaw_scope_integral_raw; /**< J-Scope只读镜像：速度积分输出原始值。 */
extern volatile float yaw_scope_angle_target_deg; /**< J-Scope只读镜像：目标连续角，度。 */
extern volatile float yaw_scope_angle_actual_deg; /**< J-Scope只读镜像：实际连续角，度。 */
extern volatile float yaw_scope_angle_error_deg; /**< J-Scope只读镜像：目标减实际连续角，度。 */
extern volatile float yaw_scope_angle_kp; /**< J-Scope只读镜像：本周期角度外环实际Kp，1/秒。 */
extern volatile uint32_t yaw_scope_state; /**< J-Scope只读镜像：状态枚举数值。 */

/* Exported functions --------------------------------------------------------*/
/** @brief 初始化配置默认许可和方向；右拨杆停止档到中档后才启动遥控Yaw。 */
void Gimbal_Init(void);
/** @brief 撤销旧请求并立即清除软件会话；驱动继续请求零电流。 */
void Gimbal_Yaw_RequestStandby(uint32_t now_ms);
/** @brief 本次离线后的Yaw待命复位是否完成。 */
int Gimbal_Yaw_StandbyReady(void);
/** @brief 按入口模式更新控制；整车模式允许底盘与Yaw同时运动。 */
void Gimbal_Yaw_Update(uint32_t now_ms, int interlock);
/** @brief 是否处于整车遥控模式，含切换入口后的遥控停止确认阶段。 */
int Gimbal_Yaw_RemoteMode(void);
/** @brief 当前是否存在需要发送Yaw运动/清零帧的控制会话。 */
int Gimbal_Yaw_OwnsBus(void);
/** @brief 当前是否要求撤销已经排队的非零电流命令。 */
int Gimbal_Yaw_NeedsZero(void);
/** @brief 构造到期0xA1帧；失能时也持续零电流，低频插入0x9A查询。 */
int Gimbal_Yaw_MakeCommand(uint32_t now_ms, uint8_t out[8]);
/** @brief 记录一次实际成功入队的运动帧，含电流与时刻。 */
void Gimbal_Yaw_Queued(uint32_t now_ms, int16_t current, uint8_t command);
/** @brief 记录实际TXOK；发送失败仅记录并继续重试。 */
void Gimbal_Yaw_TxComplete(int16_t current, uint8_t command, int success);
/** @brief 只记录发送/取消错误，不改变运动许可或状态。 */
void Gimbal_Yaw_TxError(void);
#endif
