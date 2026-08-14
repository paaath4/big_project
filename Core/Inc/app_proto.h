/* app_proto.h - 主从板共享通信协议定义
 * 两份板子用同一个头文件, 保证协议一致
 */
#ifndef APP_PROTO_H
#define APP_PROTO_H

#include <stdint.h>

/* ==================== CAN 报文 ID 协议表 ==================== */
#define ID_BREATH_CTRL     0x001       /* 标准帧  主->从  呼吸灯控制 */
#define ID_SLAVE_FEEDBACK  0x002       /* 标准帧  从->主  100Hz 变化 float */
#define ID_BEEP_CMD        0x003       /* 标准帧  CANable->主/从  蜂鸣定次数 */
#define ID_SLAVE_STATUS    0x012       /* 标准帧  从->总线  状态报文 */
#define ID_MASTER_STATUS   0x02010101  /* 扩展帧  主->总线  状态报文 */

/* 呼吸灯控制帧 (0x001) 数据域 */
#define CTRL_BYTE_OFF  0x00
#define CTRL_BYTE_ON   0x01

/* ==================== VOFA+ 指令帧格式 ====================
 * A5 [开/关] [周期低] [周期高] 5A
 *   例: A5 01 E8 03 5A  -> 开, 周期 0x03E8 = 1000ms
 */
#define VOFA_FRAME_HEAD  0xA5
#define VOFA_FRAME_TAIL  0x5A

/* ==================== 通用 CAN 帧结构 ==================== */
typedef struct {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
} can_frame_t;

#endif /* APP_PROTO_H */
