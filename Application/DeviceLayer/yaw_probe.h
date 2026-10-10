/**
 * @file    yaw_probe.h
 * @brief   Yaw反馈快照、角度观察与独立只读查询接口。
 */
#ifndef DOWN_YAW_PROBE_H
#define DOWN_YAW_PROBE_H

/* Includes ------------------------------------------------------------------*/
#include "kt_yaw_protocol.h"
#include "yaw_probe_config.h"

/* Exported typedef ----------------------------------------------------------*/
enum {
    YAW_PROBE_BLOCK_OUTPUT = 1U,    /**< 底盘输出许可尚未关闭。 */
    YAW_PROBE_BLOCK_CHASSIS = 2U,   /**< 底盘模式或电流输出尚未停止。 */
    YAW_PROBE_BLOCK_CAN = 4U        /**< CAN控制器或上一底盘发送异常。 */
};

typedef struct {
    kt_yaw_status_t status;     /**< 最近成功解码的反馈字段。 */
    uint32_t frames;            /**< 本ID收到的合法8字节帧总数。 */
    uint32_t state_frames;      /**< 0x9C及0xA1的角度/速度状态反馈数。 */
    uint32_t torque_frames;     /**< 0xA1电流命令状态回复数。 */
    uint32_t last_torque_ms;    /**< 最近0xA1回复时刻，毫秒。 */
    int16_t torque_current_raw; /**< 最近0xA1回复的实际电流原始值。 */
    int16_t torque_speed_dps;   /**< 最近0xA1回复的协议速度，度/秒。 */
    uint32_t error_frames;      /**< 0x9A状态1反馈数。 */
    uint32_t last_error_ms;     /**< 最近状态1回复时刻，毫秒。 */
    uint32_t unknown_frames;    /**< 本ID收到的其他命令帧数。 */
    uint32_t last_ms;           /**< 最近本ID收包时刻，毫秒。 */
    uint32_t last_state_ms;     /**< 最近状态2收包时刻，毫秒。 */
    uint8_t last_command;       /**< 最近接收帧的命令字。 */
    uint8_t raw[8];             /**< 最近接收帧原始8字节。 */
} yaw_probe_rx_t;

typedef struct {
    yaw_probe_rx_t feedback;    /**< 任务一次复制的接收快照。 */
    uint32_t tx_queued;         /**< 已加入发送队列的只读查询数。 */
    uint32_t tx_confirmed;      /**< 控制器报告发送完成的查询数。 */
    uint32_t tx_errors;         /**< 入队、传输、超时或取消调用失败次数。 */
    uint32_t tx_aborted;        /**< 已请求取消的查询数。 */
    uint32_t tx_busy_skips;     /**< 查询到期但发送队列占用次数。 */
    uint32_t block_reason;      /**< 查询许可故障位，可按位相加。 */
    float encoder_deg;          /**< 按65536计数/圈换算的编码角，未标定零点。 */
    float relative_deg;         /**< 暂定正前为0、俯视逆时针为正，范围[-180,180)度。 */
    float continuous_deg;       /**< 从本次跟踪起点累计的角度，度；仅连续有效时使用。 */
    uint8_t angle_valid;        /**< 至少建立过合法机械角；当前新鲜度由online和年龄另行诊断。 */
    uint8_t continuous_valid;   /**< 已建立最短角差参考；丢样不锁死，不保证长间隔完整圈数。 */
    uint8_t online;             /**< 250ms内收到状态2为1，不证明具体电机型号。 */
    uint8_t tx_pending;         /**< 本模块查询尚未完成发送或取消。 */
    uint32_t stale_samples, skipped_samples, ambiguous_samples, invalid_parameters; /**< 陈旧、跳帧、半圈歧义及非法标定观察次数。 */
} yaw_probe_t;

/* Exported variables --------------------------------------------------------*/
extern yaw_probe_t yaw_probe;                       /**< Watch使用的只读探测状态。 */
extern volatile uint32_t yaw_probe_enable;          /**< 设为1请求查询，默认0。 */
extern volatile uint32_t yaw_probe_zero_encoder;    /**< 暂定正前计数，合法范围0..65535。 */
extern volatile int32_t yaw_probe_direction;        /**< 角度符号，仅允许+1或-1。 */
extern volatile uint32_t yaw_probe_angle_reset;     /**< 设为1在新鲜反馈上重建连续角，执行后清0。 */

/* Exported functions --------------------------------------------------------*/
/** @brief 清空探测状态，并将查询许可恢复为关闭。 */
void Yaw_Probe_Init(void);
/** @brief 在CAN接收中断缓存本ID报文，成功接收返回1。 */
int Yaw_Probe_Receive(uint32_t can_id, const uint8_t *data, uint32_t size, uint32_t now_ms);
/** @brief 在屏蔽接收中断时复制反馈、更新在线状态和回绕角度。 */
void Yaw_Probe_Update(uint32_t now_ms);
/** @brief 检查独立探测许可和底盘停止/CAN条件，允许时返回1。 */
int Yaw_Probe_Allowed(int output_disabled, int chassis_stopped, int can_ready);
/** @brief 查询节拍到期时构造只读帧，成功返回1。 */
int Yaw_Probe_MakeQuery(uint32_t now_ms, uint8_t out[8]);
/** @brief 记录一次实际成功入队的查询和发送时刻。 */
void Yaw_Probe_QueryQueued(uint32_t now_ms);

#endif
