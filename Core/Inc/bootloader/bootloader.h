#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H /* 防止协议核心头文件被重复包含。 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * STM32G431 Bootloader - transport independent core
 *
 * The upper Bootloader and the lower communication driver exchange ONLY
 * Boot_Message_t.  The SAME structure is used for RX and TX.
 *
 * The core does not include fdcan.h/usart.h/spi.h/i2c.h and does not know
 * CAN IDs, UART framing, SPI chip-selects, etc.
 * ======================================================================== */

/* -------------------------- Flash layout -------------------------------- */
#define BOOT_FLASH_BASE_ADDR          0x08000000UL /* 芯片 Flash 起始地址。 */
#define BOOT_FLASH_SIZE_BYTES         (128UL * 1024UL) /* 可用 Flash 总容量。 */
#define BOOT_FLASH_END_ADDR           (BOOT_FLASH_BASE_ADDR + BOOT_FLASH_SIZE_BYTES - 1UL) /* Flash 最后地址。 */
#define BOOT_FLASH_PAGE_SIZE          2048UL /* Flash 擦除页大小，单位为字节。 */

#define BOOT_BOOTLOADER_START_ADDR    0x08000000UL /* Bootloader 代码区起始地址。 */
#define BOOT_BOOTLOADER_SIZE_BYTES    (20UL * 1024UL) /* Bootloader 保留空间大小。 */
#define BOOT_BOOTLOADER_END_ADDR      (BOOT_BOOTLOADER_START_ADDR + BOOT_BOOTLOADER_SIZE_BYTES - 1UL) /* Bootloader 代码区末地址。 */

#define BOOT_APP_START_ADDR           0x08005000UL /* APP 镜像起始地址。 */
#define BOOT_CONFIG_PAGE_ADDR         0x0801F800UL /* 持久化配置页起始地址。 */
#define BOOT_CONFIG_PAGE_SIZE         2048UL /* 配置页大小，必须与 Flash 页一致。 */
#define BOOT_CONFIG_PAGE_END_ADDR     (BOOT_CONFIG_PAGE_ADDR + BOOT_CONFIG_PAGE_SIZE - 1UL) /* 配置页末地址。 */
#define BOOT_APP_END_ADDR             (BOOT_CONFIG_PAGE_ADDR - 1UL) /* APP 可写区域末地址。 */
#define BOOT_APP_MAX_SIZE             (BOOT_CONFIG_PAGE_ADDR - BOOT_APP_START_ADDR) /* APP 最大镜像长度。 */

#define BOOT_APP_FIRST_PAGE           ((BOOT_APP_START_ADDR - BOOT_FLASH_BASE_ADDR) / BOOT_FLASH_PAGE_SIZE) /* APP 首个 Flash 页号。 */
#define BOOT_APP_PAGE_COUNT           (BOOT_APP_MAX_SIZE / BOOT_FLASH_PAGE_SIZE) /* APP 可用页数。 */
#define BOOT_CONFIG_PAGE_INDEX        ((BOOT_CONFIG_PAGE_ADDR - BOOT_FLASH_BASE_ADDR) / BOOT_FLASH_PAGE_SIZE) /* 配置页号。 */

#define BOOT_SRAM_ALIAS_START_ADDR    0x20000000UL /* 主 SRAM 起始地址。 */
#define BOOT_SRAM_ALIAS_END_ADDR      0x20008000UL /* 主 SRAM 可接受的栈顶边界。 */
#define BOOT_CCM_START_ADDR           0x10000000UL /* CCM SRAM 起始地址。 */
#define BOOT_CCM_END_ADDR             0x10002800UL /* CCM SRAM 末地址。 */

/* -------------------------- Protocol ------------------------------------ */
#define BOOT_MAX_NODE_NUM             8U /* 集群允许的最大节点数。 */
#define BOOT_DEFAULT_NODE_ID          1U /* 未提供配置时使用的默认节点号。 */
#define BOOT_BROADCAST_ID             0xFFU /* 广播目标节点号。 */

#define BOOT_CONTROL_SIZE             8U /* 控制帧的固定字节数。 */
#define BOOT_DATA_SIZE                64U /* DATA 逻辑帧的固定字节数。 */
#define BOOT_DATA_PAYLOAD_SIZE        56U /* DATA 帧中扣除命令、序号、CRC 后的有效载荷。 */
#define BOOT_DATA_CMD_WRITE           0x01U /* DATA 帧的写入命令码。 */

#define BOOT_MAX_PACKET_COUNT         ((BOOT_APP_MAX_SIZE + BOOT_DATA_PAYLOAD_SIZE - 1UL) / BOOT_DATA_PAYLOAD_SIZE) /* 最大逻辑包数。 */
#define BOOT_BITMAP_SIZE_BYTES        ((BOOT_MAX_PACKET_COUNT + 7UL) / 8UL) /* 接收 Bitmap 所需字节数。 */

#define BOOT_VERSION_MAJOR            1U /* 协议主版本号。 */
#define BOOT_VERSION_MINOR            3U /* 协议次版本号。 */

/* Session / autonomous recovery policy. Session 0 keeps legacy behavior. */
#define BOOT_SESSION_FLAG_PEER_RECOVERY   0x01U /* 启用节点间缺包恢复。 */
#define BOOT_SESSION_FLAG_GUARD_ROLLBACK  0x02U /* 启用 Guard/回滚流程。 */
#define BOOT_SESSION_FLAG_COORD_COMMIT    0x04U /* 启用协调提交流程。 */
#define BOOT_COORD_ELECTION_DELAY_MS      200U /* 广播后开始选举前的等待时间。 */
#define BOOT_COORD_CLAIM_SLOT_MS          10U /* 每个节点的确定性竞选时隙。 */
#define BOOT_PEER_PHASE_TIMEOUT_MS        5000U /* 节点阶段等待超时时间。 */
#define BOOT_PROVIDER_TIMEOUT_MS          10000U /* Provider 响应超时时间。 */
#define BOOT_PEER_MISSING_ITEM_DELAY_MS   20U /* 节点缺包项发送的最小间隔。 */
#define BOOT_COMMIT_DELAY_MS              50U /* 提交执行前的稳定等待时间。 */
#define BOOT_COMMIT_REPEAT_COUNT          3U /* 提交广播的重复次数。 */
#define BOOT_MAX_REPAIR_ROUNDS            3U /* 最大自动修复轮数。 */
#define BOOT_VERSION_PATCH            1U /* 协议补丁版本号。 */
#define BOOT_VERSION_BUILD            0U /* 构建版本号。 */

#define BOOT_CTRL_CRC8_POLY           0x07U /* 控制帧 CRC8 多项式。 */
#define BOOT_CTRL_CRC8_INIT           0x00U /* 控制帧 CRC8 初值。 */
#define BOOT_CRC32_POLY               0x04C11DB7UL /* 镜像 CRC32 多项式。 */
#define BOOT_CRC32_INIT               0xFFFFFFFFUL /* 镜像 CRC32 初值。 */

#define BOOT_CONFIG_MAGIC             0x31474643UL /* 配置页魔数，ASCII 为 CFG1。 */
#define BOOT_CONFIG_VERSION           1U /* 配置页布局版本号。 */
#define BOOT_REQUEST_MAGIC            0x544F4F42UL /* 请求进入 Bootloader 的备份寄存器魔数。 */
#define BOOT_TRIAL_MAGIC              0x41495254UL /* 一次性 APP 试运行标记，ASCII 为 TRIA。 */

#define BOOT_WRITE_REGION_APP         0x00U /* 写入目标为 APP 区。 */
#define BOOT_WRITE_REGION_CONFIG      0x01U /* 写入目标为配置暂存区。 */
#define BOOT_WRITE_REGION_BOOTLOADER  0x02U /* Bootloader 区标识，始终受保护。 */

/* RX queue stores complete logical messages. Boot_Input() is intentionally
 * lightweight so a transport ISR may call it after it has assembled a full
 * message. Heavy protocol/Flash work remains in Boot_Task().
 */
#define BOOT_RX_QUEUE_SIZE            64U /* 中断到主循环之间的逻辑消息队列容量。 */

/*
 * 单节点Host下载使用两个SRAM窗口。每个窗口容纳64个逻辑DATA包，
 * 即64 * 56 = 3584字节固件数据；Flash提交成功后才更新Bitmap。
 */
#define BOOT_FLASH_WINDOW_PACKETS     64U /* 一个 SRAM 窗口最多缓存的 DATA 包数。 */
#define BOOT_FLASH_WINDOW_COUNT       2U  /* 可并行缓存的 SRAM 窗口数量。 */
#define BOOT_FLASH_WINDOW_BYTES       (BOOT_FLASH_WINDOW_PACKETS * BOOT_DATA_PAYLOAD_SIZE) /* 单窗口字节数。 */

/* -------------------- The ONLY link-layer exchange object --------------- */
/* 逻辑消息类别：决定适配层使用控制帧、DATA 帧还是节点间帧。 */
typedef enum
{
    BOOT_MESSAGE_CONTROL      = 0U, /* 主机请求或节点响应的 8 字节控制消息。 */
    BOOT_MESSAGE_DATA         = 1U, /* 固件数据逻辑消息，固定 64 字节。 */
    BOOT_MESSAGE_PEER_CONTROL = 2U, /* 节点间协调使用的 8 字节控制消息。 */
} Boot_MessageType_t;

/* 传输层与协议核心之间唯一交换的逻辑消息对象。 */
typedef struct
{
    uint8_t type;               /* Boot_MessageType_t：控制、数据或节点间控制。 */
    uint16_t len;               /* data 中有效字节数，不能超过 BOOT_DATA_SIZE。 */
    uint8_t data[BOOT_DATA_SIZE]; /* 逻辑消息的统一载荷缓冲区。 */
} Boot_Message_t;

/* 底层发送回调：返回 1 表示已接受，返回 0 表示忙或硬件错误。 */
typedef uint8_t (*Boot_SendCallback_t)(const Boot_Message_t *message,
                                       void *user);

/* 可选的发送排空回调：RESET/JUMP_APP 前等待物理发送完成，可为 NULL。 */
typedef void (*Boot_FlushCallback_t)(void *user, uint32_t timeout_ms);

/* -------------------------- Commands ------------------------------------ */
/*
 * Command families (also used by README / COMMAND_TEST_GUIDE.md):
 *
 * A) Basic query / runtime control : 0x01..0x04, 0x18, 0x20, 0x21, 0x30
 * B) Session / Guard              : 0x05..0x08
 * C) Flash transfer / Legacy      : 0x10..0x14 (+ logical DATA cmd 0x01)
 * D) Legacy Missing / Provider    : 0x15..0x17
 * E) Autonomous election / Repair : 0x19..0x1E
 * F) Verify / Guard / Rollback    : 0x22..0x2D
 * G) Prepare / Commit             : 0x2E, 0x2F, 0x31
 * H) Host window flow control     : 0x32
 *
 * Host CONTROL uses CAN ID 0x000 and gets RESPONSE on 0x500+NodeID.
 * Autonomous peer commands use CAN ID 0x600+SourceNode and generally do NOT
 * generate Host-style ACKs; their completion is observed through paired peer
 * frames, DATA traffic, timeout handling, or the next recovery phase.
 */
/* 主机控制面、数据面和节点自治流程共用的命令码表。 */
typedef enum
{
    /* A. Basic query / runtime control. */
    BOOT_CMD_GET_VERSION      = 0x01, /* 查询 Bootloader 版本。 */
    BOOT_CMD_GET_DEVICE_ID    = 0x02, /* 查询芯片设备 ID。 */
    BOOT_CMD_GET_INFO         = 0x03, /* 查询节点、APP 和配置摘要。 */
    BOOT_CMD_ENTER_BOOT       = 0x04, /* 请求本次启动保持在 Bootloader。 */

    /* B. Session / Guard transaction setup. */
    BOOT_CMD_SET_GUARD        = 0x05, /* 指定 Guard 节点。 */
    BOOT_CMD_RELEASE_GUARD    = 0x06, /* 释放 Guard 角色。 */
    BOOT_CMD_SESSION_BEGIN    = 0x07, /* 开始新的升级事务。 */
    BOOT_CMD_SESSION_CRC32    = 0x08, /* 写入本次镜像期望 CRC32。 */

    /* C. Flash transfer / Legacy verification. */
    BOOT_CMD_ERASE            = 0x10, /* 擦除 APP 或配置目标区域。 */
    BOOT_CMD_WRITE            = 0x11, /* 建立写会话并声明镜像大小。 */
    BOOT_CMD_READ             = 0x12, /* 异步读取 Flash 数据。 */
    BOOT_CMD_VERIFY           = 0x13, /* 校验 APP CRC32 与向量表。 */
    BOOT_CMD_WRITE_END        = 0x14, /* 结束广播写入并启动缺包扫描。 */

    /* D. Legacy Missing report / Host-directed Provider. */
    BOOT_CMD_MISSING_COUNT    = 0x15, /* 报告缺包总数。 */
    BOOT_CMD_MISSING_ITEM     = 0x16, /* 报告一个缺失序号。 */
    BOOT_CMD_PROVIDER_GRANT   = 0x17, /* 指定 Host Provider 和补包范围。 */

    /* A. Runtime control continuation. */
    BOOT_CMD_ABORT            = 0x18, /* 取消当前异步任务和写会话。 */

    /* E. Autonomous node-to-node election / Provider / Repair. */
    BOOT_CMD_COORDINATOR_CLAIM = 0x19, /* 节点声明 Coordinator 候选。 */
    BOOT_CMD_PROVIDER_ASSIGN    = 0x1A, /* Coordinator 指派 Provider。 */
    BOOT_CMD_PROVIDER_DONE      = 0x1B, /* Provider 报告补包完成。 */
    BOOT_CMD_REPAIR_ROUND_END   = 0x1C, /* 结束一轮自治修复。 */
    BOOT_CMD_RECOVERY_READY     = 0x1D, /* 节点报告已准备进入下一阶段。 */
    BOOT_CMD_RECOVERY_FAILED    = 0x1E, /* 节点报告自治恢复失败。 */

    /* F. Distributed verify / Guard update / Rollback. */
    BOOT_CMD_VERIFY_REQUEST      = 0x22, /* 请求节点执行分布式校验。 */
    BOOT_CMD_VERIFY_RESULT       = 0x23, /* 返回本地校验结果。 */
    BOOT_CMD_GUARD_UPDATE_BEGIN  = 0x24, /* 开始 Guard 元数据更新。 */
    BOOT_CMD_GUARD_UPDATE_READY  = 0x25, /* Guard 更新准备完成。 */
    BOOT_CMD_ROLLBACK_REQUEST    = 0x26, /* 请求执行旧镜像回滚。 */
    BOOT_CMD_ROLLBACK_SIZE_LO    = 0x27, /* 传输回滚镜像大小低 16 位。 */
    BOOT_CMD_ROLLBACK_SIZE_HI    = 0x28, /* 传输回滚镜像大小高 16 位。 */
    BOOT_CMD_ROLLBACK_CRC_LO     = 0x29, /* 传输回滚 CRC32 低 16 位。 */
    BOOT_CMD_ROLLBACK_CRC_HI     = 0x2A, /* 传输回滚 CRC32 高 16 位。 */
    BOOT_CMD_ROLLBACK_BEGIN      = 0x2B, /* 开始回滚数据流。 */
    BOOT_CMD_ROLLBACK_PREPARED   = 0x2C, /* 回滚镜像已准备完成。 */
    BOOT_CMD_FULL_STREAM         = 0x2D, /* 请求 Provider 发送完整镜像。 */

    /* G. Prepare / Commit. */
    BOOT_CMD_COMMIT_PREPARE      = 0x2E, /* 进入分布式提交准备阶段。 */
    BOOT_CMD_COMMIT_ACK          = 0x2F, /* 回复提交准备结果。 */
    BOOT_CMD_COMMIT_EXECUTE      = 0x31, /* 执行最终元数据提交。 */

    /* H. Host窗口流控：节点提交一个SRAM窗口后主动发送。 */
    BOOT_CMD_WINDOW_READY        = 0x32, /* 节点 SRAM 窗口写入完成通知。 */

    /* A. Runtime control continuation. */
    BOOT_CMD_JUMP_APP         = 0x20, /* 刷新发送队列后跳转 APP。 */
    BOOT_CMD_RESET            = 0x21, /* 刷新发送队列后复位 MCU。 */
    BOOT_CMD_GET_STATUS       = 0x30, /* 查询当前状态、错误和进度。 */
} Boot_Command_t;

/* 对外报告的 Bootloader 工作状态。 */
typedef enum
{
    BOOT_STATUS_IDLE          = 0x00, /* 空闲，未执行升级任务。 */
    BOOT_STATUS_ERASE         = 0x01, /* 正在擦除 APP 区域。 */
    BOOT_STATUS_WRITE         = 0x02, /* 正在接收或写入固件。 */
    BOOT_STATUS_VERIFY        = 0x03, /* 正在执行 CRC/向量校验。 */
    BOOT_STATUS_READY         = 0x04, /* APP 已验证，可跳转或提交。 */
    BOOT_STATUS_ERROR         = 0x05, /* 当前事务发生错误。 */
    BOOT_STATUS_REPAIR        = 0x06, /* 正在执行缺包修复。 */
    BOOT_STATUS_GUARD         = 0x07, /* 当前节点承担 Guard 角色。 */
} Boot_Status_t;

/* 对外报告的最近一次错误原因。 */
typedef enum
{
    BOOT_ERR_NONE             = 0x00, /* 无错误。 */
    BOOT_ERR_BAD_CRC          = 0x01, /* 控制帧 CRC8 错误。 */
    BOOT_ERR_BAD_LENGTH       = 0x02, /* 逻辑消息长度不符合命令要求。 */
    BOOT_ERR_BAD_ADDRESS      = 0x03, /* Flash 地址越界或未对齐。 */
    BOOT_ERR_BAD_STATE        = 0x04, /* 当前状态不允许该命令。 */
    BOOT_ERR_FLASH_ERASE      = 0x05, /* Flash 擦除失败。 */
    BOOT_ERR_FLASH_WRITE      = 0x06, /* Flash 编程失败。 */
    BOOT_ERR_CONFIG           = 0x07, /* 配置页校验或格式错误。 */
    BOOT_ERR_APP_INVALID      = 0x08, /* APP 镜像或向量表无效。 */
    BOOT_ERR_SIZE             = 0x09, /* 镜像大小超出允许范围。 */
    BOOT_ERR_GUARD_PROTECTED  = 0x0A, /* Guard 保护禁止修改。 */
    BOOT_ERR_SEQUENCE         = 0x0B, /* DATA 序号非法或缺失。 */
    BOOT_ERR_CRC_MISMATCH     = 0x0C, /* 镜像 CRC32 不匹配。 */
    BOOT_ERR_PROVIDER_SOURCE  = 0x0D, /* Provider 数据源不可信。 */
    BOOT_ERR_BUSY             = 0x0E, /* 资源或异步任务正在占用。 */
    BOOT_ERR_PROTECTED_REGION = 0x0F, /* 试图写入受保护区域。 */
    BOOT_ERR_ABORTED          = 0x10, /* 当前升级事务已取消。 */
    BOOT_ERR_RX_OVERFLOW      = 0x11, /* 接收队列溢出。 */
    BOOT_ERR_SESSION          = 0x12, /* Session ID 或事务边界错误。 */
    BOOT_ERR_COORDINATOR      = 0x13, /* Coordinator 状态或选举错误。 */
    BOOT_ERR_RECOVERY_FAILED  = 0x14, /* 节点间修复失败。 */
    BOOT_ERR_COMMIT           = 0x15, /* 分布式提交失败。 */
    BOOT_ERR_APP_TRIAL_TIMEOUT = 0x16, /* APP 试运行未在时限内确认。 */
} Boot_Error_t;

/* 节点自治恢复状态机的阶段标识。 */
typedef enum
{
    BOOT_RECOVERY_PHASE_IDLE          = 0x00, /* 未进入自治恢复。 */
    BOOT_RECOVERY_PHASE_NEW_REPAIR    = 0x01, /* 修复新镜像缺包。 */
    BOOT_RECOVERY_PHASE_NEW_VERIFY    = 0x02, /* 校验新镜像。 */
    BOOT_RECOVERY_PHASE_GUARD_UPDATE  = 0x03, /* 更新 Guard 元数据。 */
    BOOT_RECOVERY_PHASE_GUARD_REPAIR  = 0x04, /* 修复 Guard 镜像缺包。 */
    BOOT_RECOVERY_PHASE_ROLLBACK_META = 0x05, /* 准备回滚元数据。 */
    BOOT_RECOVERY_PHASE_ROLLBACK_PREP = 0x06, /* 准备回滚数据。 */
    BOOT_RECOVERY_PHASE_ROLLBACK_REPAIR = 0x07, /* 修复回滚镜像。 */
    BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY = 0x08, /* 校验回滚镜像。 */
    BOOT_RECOVERY_PHASE_COMMIT        = 0x09, /* 广播最终提交。 */
    BOOT_RECOVERY_PHASE_FAILED        = 0x0A, /* 自治流程失败。 */
} Boot_RecoveryPhase_t;

/* 持久化的板级校准参数、节点配置和 APP 元数据。 */
typedef struct
{
    uint32_t magic;             /* 配置页魔数，用于识别有效布局。 */
    uint16_t config_version;    /* 配置结构版本号。 */
    uint16_t length;            /* 本结构实际有效长度。 */

    uint8_t  node_id;           /* 本节点 CAN 地址。 */
    uint8_t  hardware_version;  /* 板卡硬件版本标识。 */
    uint16_t reserved0;         /* 保留，写入时必须清零。 */

    uint16_t current_offset_a;  /* A 相电流采样零点。 */
    uint16_t current_offset_b;  /* B 相电流采样零点。 */
    uint16_t current_offset_c;  /* C 相电流采样零点。 */
    uint16_t vbus_offset;       /* 母线电压采样零点。 */

    float current_gain_a;       /* A 相电流比例系数。 */
    float current_gain_b;       /* B 相电流比例系数。 */
    float current_gain_c;       /* C 相电流比例系数。 */
    float vbus_gain;            /* 母线电压比例系数。 */

    uint32_t app_size;          /* 已验证 APP 的真实字节数。 */
    uint32_t app_crc32;         /* 已验证 APP 的 CRC32。 */
    uint8_t  app_valid;         /* 1 表示 APP 元数据和向量表均有效。 */
    uint8_t  reserved1[3];      /* 对齐保留字节，写入时清零。 */

    uint32_t reserved[8];       /* 为后续配置扩展预留。 */
    uint32_t crc32;             /* 除本字段外全部内容的 CRC32。 */
} Boot_Config_t;

/* -------------------------- Public API ---------------------------------- */

/* 初始化协议核心、节点号、发送回调和传输上下文；应在 HAL/时钟初始化后调用一次。 */
void Boot_Init(uint8_t default_node_id,
               Boot_SendCallback_t send_cb,
               Boot_FlushCallback_t flush_cb,
               void *transport_user);

/* 底层向核心投递一条完整逻辑消息；只复制入队，可在短小 RX 回调/ISR 中调用。 */
uint8_t Boot_Input(const Boot_Message_t *message);

/* 主循环持续调用的非阻塞任务入口，处理接收队列、Flash 和自治状态机。 */
void Boot_Task(void);

/* APP 请求下次复位进入 Bootloader：写入备份寄存器魔数。 */
void Boot_RequestBootloader(void);

/* 判断持久化 APP 是否有效且允许安全跳转。 */
uint8_t Boot_ShouldJumpApp(void);
/* 清理外设/中断并跳转到 APP Reset_Handler。 */
void Boot_JumpApp(void);

/* 读取或保存与 APP 共用的持久化配置页。 */
uint8_t Boot_ConfigLoad(Boot_Config_t *cfg);
uint8_t Boot_ConfigSave(const Boot_Config_t *cfg);

uint8_t  Boot_GetNodeId(void);          /* 读取当前节点号。 */
uint8_t  Boot_GetStatus(void);           /* 读取当前工作状态。 */
uint8_t  Boot_GetLastError(void);        /* 读取最近一次错误码。 */
uint8_t  Boot_GetProgress(void);         /* 读取升级进度百分比。 */
uint32_t Boot_GetRxOverflowCount(void);  /* 读取接收队列溢出累计次数。 */

uint16_t Boot_GetSessionId(void);        /* 读取当前 Session ID。 */
uint8_t  Boot_GetCoordinatorId(void);   /* 读取当前 Coordinator 节点号。 */
uint8_t  Boot_IsCoordinator(void);      /* 判断当前节点是否为 Coordinator。 */
uint8_t  Boot_GetRepairRound(void);     /* 读取当前自治修复轮次。 */
uint8_t  Boot_GetRecoveryPhase(void);   /* 读取自治恢复阶段。 */

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
