/*
 * STM32G4 Bootloader 协议核心。
 *
 * 处理链路固定为：Boot_Input() 在接收上下文中快速入队，Boot_Task()
 * 在主循环中依次执行命令、Flash 操作、镜像校验和节点自治状态机；
 * 所有物理发送都通过 Boot_Message_t 和注册的回调离开本模块。
 * 因此本文件不应直接依赖具体 CAN 帧 ID 或传输层缓冲区布局。
 */
#include "bootloader.h"
#include "stm32g4xx_hal.h"

#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * 协议内部线缆布局，这些类型不是传输层对象。
 * CAN/UART/SPI/I2C 适配层唯一交换的对象是 Boot_Message_t。
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t target;    /* 目标节点号，广播时使用 BOOT_BROADCAST_ID。 */
    uint8_t cmd;       /* 控制命令码。 */
    uint8_t seq;       /* 帧内序号；节点间帧中承载源节点号。 */
    uint8_t param[4];  /* 命令专用参数，按小端字节序解释。 */
    uint8_t crc;       /* 前 7 字节的 CRC8 校验值。 */
} Boot_ControlFrame_t;

typedef struct
{
    uint8_t node;      /* 回复来源节点号。 */
    uint8_t cmd;       /* 对应的请求命令码。 */
    uint8_t status;    /* Boot_Status_t 状态值。 */
    uint8_t data[4];   /* 回复附加数据。 */
    uint8_t crc;       /* 前 7 字节的 CRC8 校验值。 */
} Boot_ControlResponse_t;

_Static_assert(sizeof(Boot_ControlFrame_t) == BOOT_CONTROL_SIZE,
               "Control frame must remain 8 bytes");
_Static_assert(sizeof(Boot_ControlResponse_t) == BOOT_CONTROL_SIZE,
               "Control response must remain 8 bytes");
_Static_assert((BOOT_FLASH_WINDOW_BYTES % 8U) == 0U,
               "Flash window must remain double-word aligned");

/* ---------------------- 传输层边界 --------------------------------------- */
static Boot_SendCallback_t g_send_cb = NULL; /* 底层发送回调。 */
static Boot_FlushCallback_t g_flush_cb = NULL; /* 跳转/复位前排空发送队列的回调。 */
static void *g_transport_user = NULL; /* 传给底层回调的用户上下文。 */

static Boot_Message_t g_rx_queue[BOOT_RX_QUEUE_SIZE]; /* 接收逻辑消息环形队列。 */
static volatile uint16_t g_rx_write = 0U; /* ISR 写入位置。 */
static volatile uint16_t g_rx_read = 0U; /* 主循环读取位置。 */
static volatile uint32_t g_rx_overflow = 0U; /* 队列满时丢弃消息的累计次数。 */

/* 组装并发送主机可识别的 8 字节控制响应。 */
static uint8_t Boot_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data);
/* 查询本地 DATA 接收 Bitmap 中某个逻辑包是否已经写入 Flash。 */
static uint8_t Boot_BitmapGet(uint16_t seq);

static uint8_t Boot_Output(const Boot_Message_t *message)
{
    /* 统一出口：协议核心只生成逻辑消息，实际发送由适配层决定。 */
    if ((message == NULL) || (g_send_cb == NULL))
    {
        return 0U;
    }
    return g_send_cb(message, g_transport_user);
}

static uint8_t Boot_SendControl(const uint8_t *data, uint16_t len)
{
    /* 控制面逻辑消息，通常对应 8 字节经典 CAN 帧。 */
    Boot_Message_t message;
    if ((data == NULL) || (len > BOOT_DATA_SIZE)) return 0U;
    memset(&message, 0, sizeof(message));
    message.type = (uint8_t)BOOT_MESSAGE_CONTROL;
    message.len = len;
    memcpy(message.data, data, len);
    return Boot_Output(&message);
}

static uint8_t Boot_SendData(const uint8_t *data, uint16_t len)
{
    /* 数据面逻辑消息，通常对应 64 字节 CAN FD 帧。 */
    Boot_Message_t message;
    if ((data == NULL) || (len > BOOT_DATA_SIZE)) return 0U;
    memset(&message, 0, sizeof(message));
    message.type = (uint8_t)BOOT_MESSAGE_DATA;
    message.len = len;
    memcpy(message.data, data, len);
    return Boot_Output(&message);
}

static uint8_t Boot_SendPeerControl(const uint8_t *data, uint16_t len)
{
    /* 节点间协调控制逻辑消息。 */
    Boot_Message_t message;
    if ((data == NULL) || (len != BOOT_CONTROL_SIZE)) return 0U;
    memset(&message, 0, sizeof(message));
    message.type = (uint8_t)BOOT_MESSAGE_PEER_CONTROL;
    message.len = len;
    memcpy(message.data, data, len);
    return Boot_Output(&message);
}

static void Boot_FlushTx(uint32_t timeout_ms)
{
    /* 可选回调为空时表示底层没有额外的发送排空动作。 */
    if (g_flush_cb != NULL)
    {
        g_flush_cb(g_transport_user, timeout_ms);
    }
}

/* ================= CRC ================= */
/*
 * 直接使用片上 CRC 外设，使 Bootloader 不依赖 CubeMX 生成的 hcrc 句柄。
 * 所有调用均来自主循环上下文；禁止与中断并发调用这些函数。
 */

static void Boot_CRCFeedBytes(const uint8_t *data, uint32_t len)
{
    /* CRC 外设按 32/16/8 位写入，以下变量分别用于遍历和尾部拼接。 */
    uint32_t i = 0U;
    uint16_t halfword; /* 尾部 2 字节暂存值。 */

    while ((len - i) >= 4U)
    {
        CRC->DR = ((uint32_t)data[i] << 24U) |
                  ((uint32_t)data[i + 1U] << 16U) |
                  ((uint32_t)data[i + 2U] << 8U) |
                  ((uint32_t)data[i + 3U]);
        i += 4U;
    }

    switch (len - i)
    {
    case 3U:
        halfword = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
        *(__IO uint16_t *)(__IO void *)&CRC->DR = halfword;
        *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i + 2U];
        break;

    case 2U:
        halfword = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
        *(__IO uint16_t *)(__IO void *)&CRC->DR = halfword;
        break;

    case 1U:
        *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i];
        break;

    default:
        break;
    }
}

/* 配置片上 CRC 外设的多项式、初值和宽度。 */
static void Boot_CRCConfigure(uint32_t polynomial,
                              uint32_t init_value,
                              uint32_t polynomial_size)
{
    __HAL_RCC_CRC_CLK_ENABLE();

    /* 输入和输出均不进行位反转。 */
    MODIFY_REG(CRC->CR,
               CRC_CR_POLYSIZE | CRC_CR_REV_IN | CRC_CR_REV_OUT,
               polynomial_size);

    CRC->POL = polynomial;
    CRC->INIT = init_value;
    CRC->CR |= CRC_CR_RESET;
}

/* 计算控制帧使用的 CRC8。 */
static uint8_t Boot_CRC8(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len != 0U))
    {
        return 0U;
    }

    Boot_CRCConfigure((uint32_t)BOOT_CTRL_CRC8_POLY,
                      (uint32_t)BOOT_CTRL_CRC8_INIT,
                       CRC_CR_POLYSIZE_1); /* 8 位多项式。 */

    if (len != 0U)
    {
        Boot_CRCFeedBytes(data, len);
    }

    return (uint8_t)(CRC->DR & 0xFFU);
}

/* 计算固件镜像使用的 CRC32。 */
static uint32_t Boot_CRC32(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len != 0U))
    {
        return 0U;
    }

    Boot_CRCConfigure(BOOT_CRC32_POLY,
                      BOOT_CRC32_INIT,
                      0U); /* 32 位多项式。 */

    if (len != 0U)
    {
        Boot_CRCFeedBytes(data, len);
    }

    return CRC->DR;
}

/* ================= 存储 ================= */
_Static_assert((BOOT_CONFIG_PAGE_SIZE % 8UL) == 0UL, "Config page must be double-word aligned");
_Static_assert((BOOT_APP_START_ADDR % 8UL) == 0UL, "APP start must be 8-byte aligned");
_Static_assert(sizeof(Boot_Config_t) < BOOT_CONFIG_PAGE_SIZE, "Persistent config too large");
_Static_assert(sizeof(Boot_Config_t) == 84U, "Unexpected persistent config layout");

typedef union
{
    uint64_t words[BOOT_CONFIG_PAGE_SIZE / 8UL]; /* 按 Flash 双字编程访问的视图。 */
    uint8_t  bytes[BOOT_CONFIG_PAGE_SIZE]; /* 按字节修改配置页的视图。 */
} Boot_ConfigPageBuffer_t;

static Boot_ConfigPageBuffer_t g_config_stage; /* 配置页擦写前的 RAM 暂存副本。 */
static uint8_t g_config_stage_active = 0U; /* 配置暂存事务是否正在进行。 */

/* 计算配置结构中 CRC 字段之前区域的 CRC32。 */
static uint32_t Boot_StorageConfigCRC(const Boot_Config_t *cfg)
{
    return Boot_CRC32((const uint8_t *)cfg,
                               (uint32_t)offsetof(Boot_Config_t, crc32));
}

/* 校验配置页魔数、版本、长度和节点号等头部字段。 */
static uint8_t Boot_StorageConfigHeaderValid(const Boot_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return 0U;
    }

    if (cfg->magic != BOOT_CONFIG_MAGIC)
    {
        return 0U;
    }

    if (cfg->config_version != BOOT_CONFIG_VERSION)
    {
        return 0U;
    }

    if (cfg->length != (uint16_t)sizeof(Boot_Config_t))
    {
        return 0U;
    }

    if ((cfg->node_id < 1U) || (cfg->node_id > BOOT_MAX_NODE_NUM))
    {
        return 0U;
    }

    return 1U;
}

/* 擦除指定 Flash 页并返回 HAL 结果。 */
static uint8_t Boot_StorageErasePage(uint32_t page_index)
{
    FLASH_EraseInitTypeDef erase = {0}; /* HAL Flash 擦除参数。 */
    uint32_t page_error = 0xFFFFFFFFUL; /* HAL 返回的错误页号。 */
    HAL_StatusTypeDef st; /* HAL 擦除结果。 */

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = page_index;
    erase.NbPages = 1U;

    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 1U : 0U;
}

/* 按双字写入整页暂存数据并回读比较。 */
static uint8_t Boot_StorageProgramPage(uint32_t address,
                                       const uint64_t *words,
                                       uint32_t word_count)
{
    uint32_t i; /* 当前待编程的双字索引。 */
    HAL_StatusTypeDef st = HAL_OK; /* 最近一次 HAL 编程结果。 */

    if ((words == NULL) || ((address & 0x7UL) != 0UL))
    {
        return 0U;
    }

    HAL_FLASH_Unlock();

    for (i = 0U; i < word_count; ++i)
    {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               address + i * 8UL,
                               words[i]);
        if (st != HAL_OK)
        {
            break;
        }
    }

    HAL_FLASH_Lock();

    if (st != HAL_OK)
    {
        return 0U;
    }

    if (memcmp((const void *)(uintptr_t)address, words, word_count * 8UL) != 0)
    {
        return 0U;
    }

    return 1U;
}

/* 构造带默认采样增益和节点号的配置。 */
static void Boot_StorageMakeDefaultConfig(Boot_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = BOOT_CONFIG_MAGIC;
    cfg->config_version = BOOT_CONFIG_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);
    cfg->node_id = BOOT_DEFAULT_NODE_ID;
    cfg->hardware_version = 0U;

    cfg->current_gain_a = 1.0f;
    cfg->current_gain_b = 1.0f;
    cfg->current_gain_c = 1.0f;
    cfg->vbus_gain = 1.0f;

    cfg->app_size = 0U;
    cfg->app_crc32 = 0U;
    cfg->app_valid = 0U;
    cfg->crc32 = Boot_StorageConfigCRC(cfg);
}

uint8_t Boot_ConfigLoad(Boot_Config_t *cfg)
{
    uint32_t expected_crc; /* 根据配置内容重新计算出的 CRC32。 */

    if (cfg == NULL)
    {
        return 0U;
    }

    memcpy(cfg, (const void *)BOOT_CONFIG_PAGE_ADDR, sizeof(*cfg));

    if (Boot_StorageConfigHeaderValid(cfg) == 0U)
    {
        return 0U;
    }

    expected_crc = Boot_StorageConfigCRC(cfg);
    if (expected_crc != cfg->crc32)
    {
        return 0U;
    }

    return 1U;
}

uint8_t Boot_ConfigSave(const Boot_Config_t *cfg)
{
    Boot_Config_t tmp; /* 强制补齐版本、长度和 CRC 后写入的副本。 */

    if (cfg == NULL)
    {
        return 0U;
    }

    /* 保留 Flash 最后一页中位于本结构之外的所有字节。 */
    memcpy(g_config_stage.bytes,
           (const void *)BOOT_CONFIG_PAGE_ADDR,
           BOOT_CONFIG_PAGE_SIZE);

    tmp = *cfg;
    tmp.magic = BOOT_CONFIG_MAGIC;
    tmp.config_version = BOOT_CONFIG_VERSION;
    tmp.length = (uint16_t)sizeof(tmp);

    if ((tmp.node_id < 1U) || (tmp.node_id > BOOT_MAX_NODE_NUM))
    {
        tmp.node_id = BOOT_DEFAULT_NODE_ID;
    }

    tmp.crc32 = Boot_StorageConfigCRC(&tmp);
    memcpy(g_config_stage.bytes, &tmp, sizeof(tmp));

    if (Boot_StorageErasePage(BOOT_CONFIG_PAGE_INDEX) == 0U)
    {
        return 0U;
    }

    return Boot_StorageProgramPage(BOOT_CONFIG_PAGE_ADDR,
                                   g_config_stage.words,
                                   (uint32_t)(BOOT_CONFIG_PAGE_SIZE / 8UL));
}

static uint8_t Boot_StorageInvalidateApp(uint8_t fallback_node_id)
{
    Boot_Config_t cfg; /* 将要更新 app_valid 的配置对象。 */

    if (Boot_ConfigLoad(&cfg) == 0U)
    {
        Boot_StorageMakeDefaultConfig(&cfg);
        if ((fallback_node_id >= 1U) && (fallback_node_id <= BOOT_MAX_NODE_NUM))
        {
            cfg.node_id = fallback_node_id;
        }
    }

    cfg.app_valid = 0U;
    return Boot_ConfigSave(&cfg);
}

static uint8_t Boot_StorageSaveAppMetadata(uint32_t app_size, uint32_t app_crc32, uint8_t app_valid)
{
    Boot_Config_t cfg; /* 保存 APP 长度、CRC 和有效标志的配置对象。 */

    if (Boot_ConfigLoad(&cfg) == 0U)
    {
        Boot_StorageMakeDefaultConfig(&cfg);
    }

    cfg.app_size = app_size;
    cfg.app_crc32 = app_crc32;
    cfg.app_valid = (app_valid != 0U) ? 1U : 0U;

    return Boot_ConfigSave(&cfg);
}

static uint8_t Boot_StorageEraseApp(void)
{
    FLASH_EraseInitTypeDef erase = {0}; /* APP 区整段擦除参数。 */
    uint32_t page_error = 0xFFFFFFFFUL; /* HAL 返回的错误页号。 */
    HAL_StatusTypeDef st; /* APP 区擦除结果。 */

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (uint32_t)BOOT_APP_FIRST_PAGE;
    erase.NbPages = (uint32_t)BOOT_APP_PAGE_COUNT;

    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    return (st == HAL_OK) ? 1U : 0U;
}

static uint8_t Boot_StorageProgramApp(uint32_t offset, const uint8_t *data, uint32_t valid_len)
{
    uint32_t address; /* 本次写入对应的 Flash 绝对地址。 */
    uint32_t consumed = 0U; /* 已从输入数据消费的字节数。 */
    uint8_t block[8]; /* 对齐到双字的临时编程缓冲区。 */
    uint32_t chunk; /* 本轮实际复制的字节数。 */
    uint64_t value; /* HAL 双字编程值。 */
    HAL_StatusTypeDef st = HAL_OK; /* 最近一次 Flash 编程结果。 */

    if ((data == NULL) || (valid_len == 0U))
    {
        return 0U;
    }

    if (offset >= BOOT_APP_MAX_SIZE)
    {
        return 0U;
    }

    if (valid_len > (BOOT_APP_MAX_SIZE - offset))
    {
        return 0U;
    }

    address = BOOT_APP_START_ADDR + offset;
    if ((address & 0x7UL) != 0UL)
    {
        return 0U;
    }

    HAL_FLASH_Unlock();

    while (consumed < valid_len)
    {
        memset(block, 0xFF, sizeof(block));
        chunk = valid_len - consumed;
        if (chunk > 8U)
        {
            chunk = 8U;
        }

        memcpy(block, &data[consumed], chunk);
        memcpy(&value, block, sizeof(value));

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                               address + consumed,
                               value);
        if (st != HAL_OK)
        {
            break;
        }

        if (memcmp((const void *)(uintptr_t)(address + consumed), block, sizeof(block)) != 0)
        {
            st = HAL_ERROR;
            break;
        }

        consumed += chunk;
    }

    HAL_FLASH_Lock();
    return (st == HAL_OK) ? 1U : 0U;
}

static uint8_t Boot_StorageIsFlashRangeValid(uint32_t address, uint32_t length)
{
    const uint32_t flash_limit = BOOT_FLASH_END_ADDR + 1UL; /* 半开区间的 Flash 上界。 */

    if (length == 0U)
    {
        return 0U;
    }

    if ((address < BOOT_FLASH_BASE_ADDR) || (address >= flash_limit))
    {
        return 0U;
    }

    if (length > (flash_limit - address))
    {
        return 0U;
    }

    return 1U;
}

/* 在范围检查通过后读取 Flash 数据到 RAM。 */
static uint8_t Boot_StorageReadFlash(uint32_t address, uint8_t *dst, uint32_t length)
{
    if ((dst == NULL) || (Boot_StorageIsFlashRangeValid(address, length) == 0U))
    {
        return 0U;
    }

    memcpy(dst, (const void *)(uintptr_t)address, length);
    return 1U;
}

static uint8_t Boot_StorageConfigStageBegin(void)
{
    Boot_Config_t cfg; /* 当前持久化配置或默认配置。 */

    if (Boot_ConfigLoad(&cfg) != 0U)
    {
        memcpy(g_config_stage.bytes,
               (const void *)BOOT_CONFIG_PAGE_ADDR,
               BOOT_CONFIG_PAGE_SIZE);
    }
    else
    {
        memset(g_config_stage.bytes, 0xFF, BOOT_CONFIG_PAGE_SIZE);
        Boot_StorageMakeDefaultConfig(&cfg);
        memcpy(g_config_stage.bytes, &cfg, sizeof(cfg));
    }

    g_config_stage_active = 1U;
    return 1U;
}

/* 将配置片段写入 RAM 暂存页，不立即擦写 Flash。 */
static uint8_t Boot_StorageConfigStageWrite(uint32_t offset, const uint8_t *data, uint32_t length)
{
    if ((g_config_stage_active == 0U) || (data == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (offset >= BOOT_CONFIG_PAGE_SIZE)
    {
        return 0U;
    }

    if (length > (BOOT_CONFIG_PAGE_SIZE - offset))
    {
        return 0U;
    }

    memcpy(&g_config_stage.bytes[offset], data, length);
    return 1U;
}

static uint8_t Boot_StorageConfigStageCommit(void)
{
    Boot_Config_t *cfg; /* 指向暂存页头部的可修改配置视图。 */

    if (g_config_stage_active == 0U)
    {
        return 0U;
    }

    cfg = (Boot_Config_t *)(void *)g_config_stage.bytes;
    cfg->magic = BOOT_CONFIG_MAGIC;
    cfg->config_version = BOOT_CONFIG_VERSION;
    cfg->length = (uint16_t)sizeof(*cfg);

    if ((cfg->node_id < 1U) || (cfg->node_id > BOOT_MAX_NODE_NUM))
    {
        cfg->node_id = BOOT_DEFAULT_NODE_ID;
    }

    cfg->crc32 = Boot_StorageConfigCRC(cfg);

    if (Boot_StorageErasePage(BOOT_CONFIG_PAGE_INDEX) == 0U)
    {
        g_config_stage_active = 0U;
        return 0U;
    }

    if (Boot_StorageProgramPage(BOOT_CONFIG_PAGE_ADDR,
                                g_config_stage.words,
                                (uint32_t)(BOOT_CONFIG_PAGE_SIZE / 8UL)) == 0U)
    {
        g_config_stage_active = 0U;
        return 0U;
    }

    g_config_stage_active = 0U;
    return 1U;
}

/* 放弃当前配置暂存事务。 */
static void Boot_StorageConfigStageAbort(void)
{
    g_config_stage_active = 0U;
}

/* ================= 运行时 ================= */
/* 打开备份寄存器写权限，兼容不同 HAL 时钟宏配置。 */
static void Boot_RuntimeEnableBackupWrite(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

#if defined(__HAL_RCC_RTCAPB_CLK_ENABLE)
    __HAL_RCC_RTCAPB_CLK_ENABLE();
#endif
}

/* 写入一次性备份寄存器标记，请求复位后进入 Bootloader。 */
void Boot_RequestBootloader(void)
{
    Boot_RuntimeEnableBackupWrite();
    TAMP->BKP0R = BOOT_REQUEST_MAGIC;
    __DSB();
}

/* 读取并清除 APP 返回 Bootloader 的一次性请求标记。 */
static uint8_t Boot_RuntimeConsumeBootRequest(void)
{
    uint8_t requested; /* 本次启动是否检测到 APP 返回请求。 */

    Boot_RuntimeEnableBackupWrite();
    requested = (TAMP->BKP0R == BOOT_REQUEST_MAGIC) ? 1U : 0U;

    if (requested != 0U)
    {
        TAMP->BKP0R = 0U;
        __DSB();
    }

    return requested;
}

/*
 * BKP1R 与 BKP0R 分开使用，避免两种状态互相覆盖：
 *   BKP0R 表示 APP 已主动请求返回 Bootloader；
 *   BKP1R 表示刚执行过一次 Trial Jump，正在等待试运行结果。
 * Boot_CoreInit() 每次只消费并清除一次 BKP1R，后续复位不会重复判定。
 */
/* 写入一次性 Trial 标记，供下一次启动判定 APP 是否成功返回。 */
static void Boot_RuntimeSetTrialPending(void)
{
    Boot_RuntimeEnableBackupWrite();
    TAMP->BKP1R = BOOT_TRIAL_MAGIC;
    __DSB();
}

/* 读取并清除一次性 Trial 判定标记。 */
static uint8_t Boot_RuntimeConsumeTrialPending(void)
{
    uint8_t pending; /* 本次启动是否处于 Trial 结果判定窗口。 */

    Boot_RuntimeEnableBackupWrite();
    pending = (TAMP->BKP1R == BOOT_TRIAL_MAGIC) ? 1U : 0U;

    if (pending != 0U)
    {
        TAMP->BKP1R = 0U;
        __DSB();
    }

    return pending;
}

/*
 * Trial 看门狗：LSI 标称 32 kHz，预分频 /32，重装值 2999。
 * 标称超时 = (2999 + 1) * 32 / 32000 = 3.0 秒。
 * 仅在 Trial Jump 前启动；Boot_Task() 和 APP 都不增加周期喂狗路径。
 */
/* 配置约 3 秒超时的独立看门狗，用于保护 Trial Jump。 */
static void Boot_RuntimeStartTrialWatchdog(void)
{
    IWDG->KR = 0x0000CCCCUL; /* 启动 IWDG。 */
    IWDG->KR = 0x00005555UL; /* 允许写 PR/RLR。 */
    IWDG->PR = 3UL;          /* 预分频 /32。 */
    IWDG->RLR = 2999UL;

    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0UL)
    {
        /* 等待预分频和重装值在数个 LSI 周期内更新完成。 */
    }

    IWDG->KR = 0x0000AAAAUL; /* 进入 APP 前重装一次计数器。 */
    __DSB();
}

/* 校验 APP 向量表的栈指针、Thumb 位和入口地址范围。 */
static uint8_t Boot_RuntimeVectorTableValid(void)
{
    uint32_t msp = *(const volatile uint32_t *)BOOT_APP_START_ADDR; /* APP 初始主栈指针。 */
    uint32_t reset_handler = *(const volatile uint32_t *)(BOOT_APP_START_ADDR + 4UL); /* APP 复位向量原值。 */
    uint32_t reset_address = reset_handler & ~1UL; /* 清除 Thumb 位后的入口地址。 */
    uint8_t stack_valid = 0U; /* 栈地址是否落在允许的 SRAM 区间。 */

    if ((msp >= BOOT_SRAM_ALIAS_START_ADDR) && (msp <= BOOT_SRAM_ALIAS_END_ADDR))
    {
        stack_valid = 1U;
    }
    else if ((msp >= BOOT_CCM_START_ADDR) && (msp <= BOOT_CCM_END_ADDR))
    {
        stack_valid = 1U;
    }

    if (stack_valid == 0U)
    {
        return 0U;
    }

    if ((reset_handler & 1UL) == 0UL)
    {
        return 0U;
    }

    if ((reset_address < BOOT_APP_START_ADDR) || (reset_address > BOOT_APP_END_ADDR))
    {
        return 0U;
    }

    return 1U;
}

/* 校验指定长度 APP 镜像的向量表和 CRC32。 */
static uint8_t Boot_RuntimeValidateImage(uint32_t app_size, uint32_t app_crc32)
{
    if ((app_size == 0U) || (app_size > BOOT_APP_MAX_SIZE)) return 0U;
    if (Boot_RuntimeVectorTableValid() == 0U) return 0U;
    return (Boot_CRC32((const uint8_t *)BOOT_APP_START_ADDR, app_size) == app_crc32) ? 1U : 0U;
}

/* 按持久化 app_valid 标记校验可自动启动的 APP。 */
static uint8_t Boot_RuntimeValidatePersistedApp(Boot_Config_t *cfg_out)
{
    Boot_Config_t cfg; /* 从配置页读出的 APP 元数据。 */
    if (Boot_ConfigLoad(&cfg) == 0U) return 0U;
    if (cfg.app_valid == 0U) return 0U;
    if (Boot_RuntimeValidateImage(cfg.app_size, cfg.app_crc32) == 0U) return 0U;
    if (cfg_out != NULL) *cfg_out = cfg;
    return 1U;
}

/* Trial 运行期间元数据会被故意置为 app_valid=0。
 * 此处忽略临时失效标志，仅按已保存的长度、CRC 和向量表重新校验镜像。
 */
static uint8_t Boot_RuntimeValidateTrialImage(Boot_Config_t *cfg_out)
{
    Boot_Config_t cfg; /* Trial 状态下重新校验的 APP 元数据。 */

    if (Boot_ConfigLoad(&cfg) == 0U) return 0U;
    if (Boot_RuntimeValidateImage(cfg.app_size, cfg.app_crc32) == 0U) return 0U;
    if (cfg_out != NULL) *cfg_out = cfg;
    return 1U;
}

/*
 * 将控制权从 Bootloader 转交给 APP 镜像。
 *
 * 注意：这不等同于直接调用 APP 的复位处理函数。Bootloader 退出时的 PLL、
 * 外设时钟、SysTick 和 NVIC 状态可能与复位默认值不同。STM32G4 HAL 在
 * PLL 仍作为 SYSCLK 且目标 PLL 参数不同时拒绝重新配置；硬件上曾出现
 * Bootloader 为 170 MHz、APP 为 168 MHz 的情况。
 *
 * 因此交接顺序必须固定为：
 *   1) HAL_DeInit()      - 复位外设模块和 MSP 底层状态；
 *   2) HAL_RCC_DeInit()  - 将 SYSCLK 切回 HSI 并关闭 HSE/PLL；
 *   3) 禁用并清除 SysTick 及所有 NVIC 挂起/使能中断；
 *   4) 设置 APP 的 VTOR、CONTROL 和 MSP；
 *   5) 跳转到 APP 的 Reset_Handler。
 *
 * HAL_DeInit/HAL_RCC_DeInit 必须在 __disable_irq() 之前调用，因为它们的
 * 超时路径依赖 HAL tick。RCC 恢复到近似复位状态后，再关闭并清理中断，
 * 最后修改 MSP。
 */
static void Boot_RuntimeJumpToApp(void)
{
    typedef void (*AppEntry_t)(void); /* APP 复位入口函数类型。 */

    uint32_t app_msp = *(const volatile uint32_t *)BOOT_APP_START_ADDR; /* 交接给 APP 的主栈值。 */
    uint32_t app_reset = *(const volatile uint32_t *)(BOOT_APP_START_ADDR + 4UL); /* APP 复位入口向量。 */
    uint32_t i; /* 清理 NVIC 各寄存器组的循环索引。 */
    AppEntry_t entry = (AppEntry_t)(uintptr_t)app_reset; /* 转换后的 APP 入口函数。 */

    /* 在交接给 APP 前恢复近似复位态的时钟/外设状态。Bootloader 与
     * Observer_Motor 使用不同 PLL 配置；当 PLL 仍是活动 SYSCLK 源时，
     * HAL_RCC_OscConfig() 会拒绝重新配置该 PLL。 */
    (void)HAL_DeInit();
    (void)HAL_RCC_DeInit();

    __disable_irq();

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;

    for (i = 0U; i < 8U; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = BOOT_APP_START_ADDR;
    __DSB();
    __ISB();

    __set_CONTROL(0U);
    __set_MSP(app_msp);
    __DSB();
    __ISB();

    __enable_irq();
    entry();

    while (1)
    {
        /* APP 复位处理函数不应返回。 */
    }
}

/* ================= 协议 ================= */
static uint8_t g_node_id = BOOT_DEFAULT_NODE_ID; /* 当前节点号。 */
static Boot_Status_t g_status = BOOT_STATUS_IDLE; /* 当前协议状态。 */
static Boot_Error_t g_last_error = BOOT_ERR_NONE; /* 最近一次错误码。 */
static uint8_t g_progress = 0U; /* 当前写入/恢复进度百分比。 */
static uint8_t g_boot_requested = 0U; /* 是否收到进入 Bootloader 的请求。 */

static Boot_Config_t g_config; /* 当前缓存的持久化配置。 */
static uint8_t g_config_valid = 0U; /* 配置缓存是否通过校验。 */
static uint8_t g_app_valid = 0U; /* APP 镜像是否可启动。 */

static uint8_t g_guard_active = 0U; /* 当前是否存在 Guard 保护。 */
static uint8_t g_guard_node_id = 0U; /* Guard 节点号。 */
static uint8_t g_is_guard = 0U; /* 当前节点是否承担 Guard 角色。 */

static uint8_t g_app_erased = 0U; /* 本次会话是否已完成 APP 擦除。 */
static uint8_t g_write_active = 0U; /* 是否存在进行中的写入会话。 */
static uint8_t g_write_region = BOOT_WRITE_REGION_APP; /* 当前写入区域。 */
static uint32_t g_write_size = 0U; /* 当前镜像声明长度。 */
static uint16_t g_total_packets = 0U; /* 当前镜像逻辑 DATA 包总数。 */
static uint16_t g_received_packets = 0U; /* 已提交到 Flash 的 DATA 包数。 */
static uint8_t g_bitmap[BOOT_BITMAP_SIZE_BYTES]; /* 本地已写入包的接收 Bitmap。 */

typedef enum
{
    BOOT_FLASH_WINDOW_EMPTY = 0U, /* 窗口未分配给任何包范围。 */
    BOOT_FLASH_WINDOW_FILLING,    /* 正在接收属于该窗口的 DATA 包。 */
    BOOT_FLASH_WINDOW_READY,      /* 所需包已齐，等待 Flash 写入。 */
    BOOT_FLASH_WINDOW_WRITING,    /* 正在把窗口内容分批提交到 Flash。 */
    BOOT_FLASH_WINDOW_ACK_PENDING /* 写入完成，等待发送窗口确认。 */
} Boot_FlashWindowState_t;

typedef struct
{
    Boot_FlashWindowState_t state; /* 窗口当前阶段。 */
    uint16_t base_seq; /* 窗口覆盖的首个逻辑包序号。 */
    uint8_t expected_packets; /* 窗口期望接收的包数。 */
    uint8_t received_packets; /* 窗口已缓存的包数。 */
    uint8_t received_mask[BOOT_FLASH_WINDOW_PACKETS / 8U]; /* 窗口内包接收位图。 */
    uint8_t bytes[BOOT_FLASH_WINDOW_BYTES]; /* 窗口内固件数据暂存区。 */
} Boot_FlashWindow_t;

static Boot_FlashWindow_t g_flash_windows[BOOT_FLASH_WINDOW_COUNT]; /* 双窗口缓存。 */

static uint8_t g_read_active = 0U; /* 非阻塞 READ 任务是否有效。 */
static uint32_t g_read_address = 0U; /* 下一段待读取的 Flash 地址。 */
static uint16_t g_read_remaining = 0U; /* 尚未发送的读取字节数。 */

static uint8_t g_missing_report_active = 0U; /* 主机缺包报告任务是否运行。 */
static uint8_t g_missing_count_sent = 0U; /* 是否已发送缺包总数。 */
static uint16_t g_missing_count = 0U; /* 当前缺包总数。 */
static uint16_t g_missing_scan_seq = 0U; /* 扫描本地 Bitmap 的序号。 */
static uint16_t g_missing_item_index = 0U; /* 已发送缺包项数量。 */

static uint8_t g_provider_active = 0U; /* Provider 发送任务是否运行。 */
static uint8_t g_provider_done_pending = 0U; /* Provider 完成通知是否待发送。 */
static uint8_t g_provider_target = 0U; /* Provider 数据接收目标节点。 */
static uint16_t g_provider_next_seq = 0U; /* Provider 下一包序号。 */
static uint16_t g_provider_remaining = 0U; /* Provider 尚未发送的包数。 */
static uint8_t g_provider_peer_mode = 0U; /* 是否由节点间恢复流程触发。 */
static uint16_t g_provider_peer_seq = 0U; /* 节点间 Provider 当前请求序号。 */

static uint8_t g_session_active = 0U; /* 分布式升级会话是否打开。 */
static uint8_t g_session_flags = 0U; /* 当前会话策略位。 */
static uint16_t g_session_id = 0U; /* 当前会话标识。 */
static uint32_t g_session_image_size = 0U; /* 会话声明的镜像长度。 */
static uint32_t g_session_expected_crc32 = 0U; /* 会话声明的镜像 CRC32。 */
static uint8_t g_session_crc_valid = 0U; /* 会话 CRC 元数据是否已接收。 */

static uint8_t g_peer_active_bitmap = 0U; /* 本次恢复会话参与节点位图。 */
static uint8_t g_peer_report_complete_bitmap = 0U; /* 已完成缺包报告节点位图。 */
static uint8_t g_peer_missing_bitmap[BOOT_MAX_NODE_NUM][BOOT_BITMAP_SIZE_BYTES]; /* 各节点缺包位图。 */
static uint16_t g_peer_missing_expected[BOOT_MAX_NODE_NUM]; /* 各节点声明的缺包数。 */
static uint16_t g_peer_missing_received[BOOT_MAX_NODE_NUM]; /* 各节点已接收缺包项数。 */
static uint8_t g_peer_report_tx_active = 0U; /* 本节点缺包报告发送任务状态。 */
static uint8_t g_peer_report_tx_count_sent = 0U; /* 本节点是否已发送缺包总数。 */
static uint16_t g_peer_report_tx_scan_seq = 0U; /* 本节点缺包位图扫描序号。 */
static uint16_t g_peer_report_tx_item_count = 0U; /* 本节点已发送缺包项数。 */
static uint32_t g_peer_report_items_after_ms = 0U; /* 下一缺包项允许发送的时间。 */
static uint8_t g_peer_recovery_started = 0U; /* 节点间恢复是否已启动。 */
static uint8_t g_election_pending = 0U; /* 是否正在等待协调者竞选。 */
static uint32_t g_election_started_ms = 0U; /* 竞选阶段起始时间戳。 */
static uint8_t g_coordinator_id = 0U; /* 当前协调者节点号。 */
static uint8_t g_is_coordinator = 0U; /* 当前节点是否为协调者。 */
static uint8_t g_recovery_members_bitmap = 0U; /* 当前恢复成员位图。 */
static uint8_t g_repair_round = 0U; /* 当前修复轮次。 */
static uint16_t g_coord_scan_seq = 0U; /* 协调者扫描的目标包序号。 */
static uint16_t g_coord_current_seq = 0U; /* 当前协调分配中的包序号。 */
static uint8_t g_coord_provider_id = 0U; /* 当前被选中的 Provider 节点号。 */
static uint8_t g_coord_wait_provider = 0U; /* 是否等待 Provider 完成通知。 */
static uint8_t g_provider_failed_bitmap = 0U; /* Provider 失败节点位图。 */
static uint32_t g_provider_wait_started_ms = 0U; /* Provider 等待起始时间戳。 */
static uint8_t g_coord_sweep_had_missing = 0U; /* 本轮扫描是否发现缺包。 */
static uint8_t g_coord_terminal = 0U; /* 协调恢复是否已进入终态。 */
static uint8_t g_primary_members_bitmap = 0U; /* 主升级成员位图。 */
static Boot_RecoveryPhase_t g_recovery_phase = BOOT_RECOVERY_PHASE_IDLE; /* 当前恢复阶段。 */
static uint32_t g_phase_started_ms = 0U; /* 当前恢复阶段起始时间戳。 */

static uint8_t g_verify_response_bitmap = 0U; /* 已回复校验结果节点位图。 */
static uint8_t g_verify_ok_bitmap = 0U; /* 校验成功节点位图。 */
static uint8_t g_verify_context = 0U; /* 当前校验上下文。 */
static uint32_t g_prepared_app_size = 0U; /* 已准备提交的 APP 长度。 */
static uint32_t g_prepared_app_crc32 = 0U; /* 已准备提交的 APP CRC32。 */
static uint8_t g_local_image_prepared = 0U; /* 本地镜像是否已完成 Prepare。 */

static uint32_t g_rollback_size = 0U; /* 回滚镜像长度。 */
static uint32_t g_rollback_crc32 = 0U; /* 回滚镜像 CRC32。 */
static uint8_t g_rollback_meta_mask = 0U; /* 已接收回滚元数据字段位图。 */
static uint8_t g_guard_meta_tx_stage = 0U; /* Guard 元数据发送阶段。 */
static uint8_t g_rollback_prepared_bitmap = 0U; /* 已完成回滚准备的节点位图。 */

static uint8_t g_commit_expected_bitmap = 0U; /* 需要参与提交的节点位图。 */
static uint8_t g_commit_ack_bitmap = 0U; /* 已确认提交准备的节点位图。 */
static uint8_t g_commit_armed = 0U; /* 本地是否已武装提交。 */
static uint8_t g_commit_tx_remaining = 0U; /* 尚需发送的提交广播次数。 */
static uint8_t g_commit_execute_received = 0U; /* 是否已接收提交执行命令。 */
static uint32_t g_commit_due_ms = 0U; /* 延迟提交允许执行的时间戳。 */

static void Boot_StartRollback(void);
static void Boot_StartCommit(void);
static void Boot_StartVerifyContext(uint8_t context);
static void Boot_CoordinatorProviderDone(uint16_t value);
static void Boot_CoordinatorPhaseFailure(void);

/* 从字节数组读取小端序 32 位整数。 */
static uint32_t Boot_ReadU32LE(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

/* 从字节数组读取小端序 16 位整数。 */
static uint16_t Boot_ReadU16LE(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8U));
}

/* 将 16 位整数按小端序写入字节数组。 */
static void Boot_WriteU16LE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

/* 将 32 位整数按小端序写入字节数组。 */
static void Boot_WriteU32LE(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
    p[2] = (uint8_t)((value >> 16U) & 0xFFU);
    p[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/* 把节点号转换为成员位图中的单比特掩码。 */
static uint8_t Boot_NodeBit(uint8_t node_id)
{
    if ((node_id < 1U) || (node_id > BOOT_MAX_NODE_NUM)) return 0U;
    return (uint8_t)(1U << (node_id - 1U));
}

/* 查询指定节点是否报告某个逻辑包缺失。 */
static uint8_t Boot_PeerMissingGet(uint8_t node_id, uint16_t seq)
{
    uint8_t index;
    if ((node_id < 1U) || (node_id > BOOT_MAX_NODE_NUM) ||
        ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)) return 1U;
    index = (uint8_t)(node_id - 1U);
    return (uint8_t)((g_peer_missing_bitmap[index][seq >> 3U] >> (seq & 7U)) & 0x01U);
}

/* 在指定节点的缺包位图中置位一个逻辑包序号。 */
static void Boot_PeerMissingSet(uint8_t node_id, uint16_t seq)
{
    uint8_t index;
    if ((node_id < 1U) || (node_id > BOOT_MAX_NODE_NUM) ||
        ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)) return;
    index = (uint8_t)(node_id - 1U);
    g_peer_missing_bitmap[index][seq >> 3U] |= (uint8_t)(1U << (seq & 7U));
}

/* 组装并发送一帧节点间协调控制消息。 */
static uint8_t Boot_SendPeerFrame(uint8_t target, uint8_t cmd, uint8_t source,
                                  uint16_t session, uint16_t value)
{
    Boot_ControlFrame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.target = target;
    frame.cmd = cmd;
    frame.seq = source;
    Boot_WriteU16LE(&frame.param[0], session);
    Boot_WriteU16LE(&frame.param[2], value);
    frame.crc = Boot_CRC8((const uint8_t *)&frame, 7U);
    return Boot_SendPeerControl((const uint8_t *)&frame, BOOT_CONTROL_SIZE);
}

static void Boot_BitmapClear(void);
static uint16_t Boot_MissingCount(void);
static void Boot_CancelAsyncTasks(void);

static uint8_t Boot_PrepareAppWriteInternal(uint32_t size)
{
    uint32_t packets;
    if ((size == 0U) || (size > BOOT_APP_MAX_SIZE)) return 0U;
    packets = (size + BOOT_DATA_PAYLOAD_SIZE - 1UL) / BOOT_DATA_PAYLOAD_SIZE;
    if ((packets == 0U) || (packets > BOOT_MAX_PACKET_COUNT)) return 0U;

    Boot_CancelAsyncTasks();
    Boot_StorageConfigStageAbort();
    g_app_erased = 0U;
    g_progress = 0U;
    if (Boot_StorageInvalidateApp(g_node_id) == 0U) return 0U;
    g_app_valid = 0U;
    g_config_valid = Boot_ConfigLoad(&g_config);
    if (Boot_StorageEraseApp() == 0U) return 0U;

    Boot_BitmapClear();
    g_app_erased = 1U;
    g_write_active = 1U;
    g_write_region = BOOT_WRITE_REGION_APP;
    g_write_size = size;
    g_total_packets = (uint16_t)packets;
    g_status = BOOT_STATUS_WRITE;
    g_last_error = BOOT_ERR_NONE;
    g_local_image_prepared = 0U;
    return 1U;
}

static uint8_t Boot_PrepareVerifiedImage(uint32_t expected_crc)
{
    uint32_t actual_crc;
    if ((g_write_active == 0U) || (g_write_region != BOOT_WRITE_REGION_APP) ||
        (Boot_MissingCount() != 0U) || (g_write_size == 0U)) return 0U;
    actual_crc = Boot_CRC32((const uint8_t *)BOOT_APP_START_ADDR, g_write_size);
    if (actual_crc != expected_crc) { g_last_error = BOOT_ERR_CRC_MISMATCH; return 0U; }
    if (Boot_RuntimeValidateImage(g_write_size, actual_crc) == 0U) { g_last_error = BOOT_ERR_APP_INVALID; return 0U; }
    /* 自治模式采用 Prepare/Commit：校验通过的镜像在 COMMIT_EXECUTE 前仍不可启动。 */
    if (Boot_StorageSaveAppMetadata(g_write_size, actual_crc, 0U) == 0U) { g_last_error = BOOT_ERR_CONFIG; return 0U; }
    g_config_valid = Boot_ConfigLoad(&g_config);
    if (g_config_valid == 0U) { g_last_error = BOOT_ERR_CONFIG; return 0U; }
    g_app_valid = 0U;
    g_prepared_app_size = g_write_size;
    g_prepared_app_crc32 = actual_crc;
    g_local_image_prepared = 1U;
    g_status = BOOT_STATUS_READY;
    return 1U;
}

static uint8_t Boot_ArmCommitLocal(void)
{
    if (g_local_image_prepared != 0U)
    {
        if (Boot_RuntimeValidateImage(g_prepared_app_size, g_prepared_app_crc32) == 0U) return 0U;
    }
    else if (Boot_RuntimeValidatePersistedApp(&g_config) != 0U)
    {
        g_prepared_app_size = g_config.app_size;
        g_prepared_app_crc32 = g_config.app_crc32;
    }
    else return 0U;
    g_commit_armed = 1U;
    return 1U;
}

static uint8_t Boot_ExecuteCommitLocal(void)
{
    if (g_commit_armed == 0U) return 0U;
    if (Boot_StorageSaveAppMetadata(g_prepared_app_size, g_prepared_app_crc32, 1U) == 0U) return 0U;
    if (Boot_ConfigLoad(&g_config) == 0U) return 0U;
    g_config_valid = 1U;
    g_app_valid = 1U;
    g_commit_execute_received = 1U;
    g_commit_due_ms = HAL_GetTick() + BOOT_COMMIT_DELAY_MS;
    return 1U;
}

static void Boot_SetError(Boot_Error_t error, uint8_t fatal)
{
    g_last_error = error;
    if (fatal != 0U)
    {
        g_status = BOOT_STATUS_ERROR;
    }
}

static void Boot_SendError(uint8_t cmd, Boot_Error_t error)
{
    uint8_t data[4] = {0};
    data[0] = (uint8_t)error;
    Boot_SetError(error, 1U);
    (void)Boot_SendResponse(cmd, BOOT_STATUS_ERROR, data);
}

/* 清空本地接收 Bitmap，并同步清零已接收包计数。 */
static void Boot_BitmapClear(void)
{
    memset(g_bitmap, 0, sizeof(g_bitmap));
    g_received_packets = 0U;
}

/* 复位所有 SRAM 窗口，丢弃尚未提交的窗口数据。 */
static void Boot_FlashWindowsReset(void)
{
    memset(g_flash_windows, 0, sizeof(g_flash_windows));
}

/* 检查是否存在尚未处理完的 SRAM 窗口。 */
static uint8_t Boot_FlashWindowsBusy(void)
{
    uint8_t i;

    for (i = 0U; i < BOOT_FLASH_WINDOW_COUNT; ++i)
    {
        if (g_flash_windows[i].state != BOOT_FLASH_WINDOW_EMPTY)
        {
            return 1U;
        }
    }
    return 0U;
}

/* 统计可继续接收数据的空闲窗口数量。 */
static uint8_t Boot_FlashWindowFreeCount(void)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < BOOT_FLASH_WINDOW_COUNT; ++i)
    {
        if (g_flash_windows[i].state == BOOT_FLASH_WINDOW_EMPTY)
        {
            count++;
        }
    }
    return count;
}

/* 返回当前 Bitmap 中第一个缺失的数据包序号。 */
static uint16_t Boot_FirstMissingSequence(void)
{
    uint16_t seq;

    for (seq = 0U; seq < g_total_packets; ++seq)
    {
        if (Boot_BitmapGet(seq) == 0U)
        {
            return seq;
        }
    }
    return g_total_packets;
}

/* 将一个逻辑 DATA 包写入对应 SRAM 窗口并更新窗口位图。 */
static uint8_t Boot_FlashWindowStage(uint16_t seq,
                                     const uint8_t *payload,
                                     uint32_t valid_len)
{
    Boot_FlashWindow_t *window = NULL; /* 与当前序号对应的窗口。 */
    uint16_t base_seq; /* 当前窗口起始序号。 */
    uint16_t remaining; /* 镜像尾部剩余包数。 */
    uint8_t slot; /* 包在窗口内的相对槽位。 */
    uint8_t i; /* 搜索窗口槽位的循环索引。 */

    base_seq = (uint16_t)(seq - (seq % BOOT_FLASH_WINDOW_PACKETS));
    slot = (uint8_t)(seq - base_seq);

    for (i = 0U; i < BOOT_FLASH_WINDOW_COUNT; ++i)
    {
        if ((g_flash_windows[i].state != BOOT_FLASH_WINDOW_EMPTY) &&
            (g_flash_windows[i].base_seq == base_seq))
        {
            window = &g_flash_windows[i];
            break;
        }
    }

    if (window == NULL)
    {
        for (i = 0U; i < BOOT_FLASH_WINDOW_COUNT; ++i)
        {
            if (g_flash_windows[i].state == BOOT_FLASH_WINDOW_EMPTY)
            {
                window = &g_flash_windows[i];
                memset(window, 0, sizeof(*window));
                memset(window->bytes, 0xFF, sizeof(window->bytes));
                window->state = BOOT_FLASH_WINDOW_FILLING;
                window->base_seq = base_seq;
                remaining = (uint16_t)(g_total_packets - base_seq);
                window->expected_packets =
                    (remaining > BOOT_FLASH_WINDOW_PACKETS)
                        ? BOOT_FLASH_WINDOW_PACKETS
                        : (uint8_t)remaining;
                break;
            }
        }
    }

    if ((window == NULL) ||
        ((window->state != BOOT_FLASH_WINDOW_FILLING) &&
         (window->state != BOOT_FLASH_WINDOW_READY)))
    {
        g_last_error = BOOT_ERR_BUSY;
        return 0U;
    }

    if ((window->received_mask[slot >> 3U] &
         (uint8_t)(1U << (slot & 7U))) != 0U)
    {
        return 1U;
    }

    memcpy(&window->bytes[(uint32_t)slot * BOOT_DATA_PAYLOAD_SIZE],
           payload,
           valid_len);
    window->received_mask[slot >> 3U] |=
        (uint8_t)(1U << (slot & 7U));
    window->received_packets++;

    if (window->received_packets >= window->expected_packets)
    {
        window->state = BOOT_FLASH_WINDOW_READY;
    }
    return 1U;
}

/* 读取本地接收 Bitmap 中指定包的完成状态。 */
static uint8_t Boot_BitmapGet(uint16_t seq)
{
    if ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)
    {
        return 0U;
    }

    return (uint8_t)((g_bitmap[seq >> 3U] >> (seq & 7U)) & 0x01U);
}

/* 将指定逻辑包标记为已成功写入 Flash。 */
static void Boot_BitmapSet(uint16_t seq)
{
    if ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)
    {
        return;
    }

    g_bitmap[seq >> 3U] |= (uint8_t)(1U << (seq & 7U));
}

/* 统计当前镜像尚未写入 Flash 的逻辑包数量。 */
static uint16_t Boot_MissingCount(void)
{
    if (g_received_packets >= g_total_packets)
    {
        return 0U;
    }

    return (uint16_t)(g_total_packets - g_received_packets);
}

/* 根据接收包数更新对外报告的百分比进度。 */
static void Boot_UpdateProgress(void)
{
    if (g_total_packets == 0U)
    {
        g_progress = 0U;
        return;
    }

    g_progress = (uint8_t)(((uint32_t)g_received_packets * 100UL) /
                           (uint32_t)g_total_packets);
}

/*
 * 单节点Host下载的Flash消费者。一个完整窗口只执行一次Unlock/Lock，
 * 写入与整窗口回读成功后才设置包Bitmap并发送WINDOW_READY。
 * CAN中断在此期间仍可把后续物理帧放入扩大的软件接收队列。
 */
static void Boot_TaskFlashWindows(void)
{
    Boot_FlashWindow_t *window; /* 当前正在提交或清理的 SRAM 窗口。 */
    uint32_t offset;
    uint32_t valid_len;
    uint8_t data[4];
    uint8_t i;
    uint8_t slot;

    for (i = 0U; i < BOOT_FLASH_WINDOW_COUNT; ++i)
    {
        window = &g_flash_windows[i];

        if (window->state == BOOT_FLASH_WINDOW_ACK_PENDING)
        {
            Boot_WriteU16LE(data, Boot_FirstMissingSequence());
            data[2] = BOOT_FLASH_WINDOW_PACKETS;
            data[3] = 1U; /* 至少释放一个窗口，可继续发送下一窗口。 */

            if (Boot_SendResponse(BOOT_CMD_WINDOW_READY,
                                  BOOT_STATUS_WRITE,
                                  data) != 0U)
            {
                memset(window, 0, sizeof(*window));
            }
            return;
        }

        if (window->state != BOOT_FLASH_WINDOW_READY)
        {
            continue;
        }

        window->state = BOOT_FLASH_WINDOW_WRITING;
        offset = (uint32_t)window->base_seq * BOOT_DATA_PAYLOAD_SIZE;
        valid_len = g_write_size - offset;
        if (valid_len > BOOT_FLASH_WINDOW_BYTES)
        {
            valid_len = BOOT_FLASH_WINDOW_BYTES;
        }

        if (Boot_StorageProgramApp(offset, window->bytes, valid_len) == 0U)
        {
            g_last_error = BOOT_ERR_FLASH_WRITE;
            g_status = BOOT_STATUS_ERROR;
            memset(window, 0, sizeof(*window));
            Boot_SendError(BOOT_CMD_WINDOW_READY, BOOT_ERR_FLASH_WRITE);
            return;
        }

        for (slot = 0U; slot < window->expected_packets; ++slot)
        {
            uint16_t seq = (uint16_t)(window->base_seq + slot);
            if (Boot_BitmapGet(seq) == 0U)
            {
                Boot_BitmapSet(seq);
                g_received_packets++;
            }
        }
        Boot_UpdateProgress();
        window->state = BOOT_FLASH_WINDOW_ACK_PENDING;
        return; /* 每次Boot_Task最多提交一个窗口。 */
    }
}

/* 判断当前 APP 写入是否受到 Guard 节点保护。 */
static uint8_t Boot_IsGuardProtected(void)
{
    return ((g_guard_active != 0U) && (g_is_guard != 0U)) ? 1U : 0U;
}

static void Boot_CancelAsyncTasks(void)
{
    g_read_active = 0U;
    g_missing_report_active = 0U;
    g_missing_count_sent = 0U;
    g_provider_active = 0U;
    g_provider_done_pending = 0U;
    g_provider_peer_mode = 0U;
    Boot_FlashWindowsReset();
}

/* 检查 Provider 是否拥有指定序号的数据包，并返回源镜像长度。 */
static uint8_t Boot_ProviderPacketAvailable(uint16_t seq, uint32_t *source_size)
{
    uint32_t persisted_packets;

    if ((g_write_active != 0U) &&
        (g_write_region == BOOT_WRITE_REGION_APP) &&
        (seq < g_total_packets) &&
        (Boot_BitmapGet(seq) != 0U))
    {
        if (source_size != NULL)
        {
            *source_size = g_write_size;
        }
        return 1U;
    }

    if ((g_app_valid != 0U) &&
        (g_config_valid != 0U) &&
        (g_config.app_valid != 0U) &&
        (g_config.app_size > 0U) &&
        (g_config.app_size <= BOOT_APP_MAX_SIZE))
    {
        persisted_packets = (g_config.app_size + BOOT_DATA_PAYLOAD_SIZE - 1UL) /
                            BOOT_DATA_PAYLOAD_SIZE;

        if ((uint32_t)seq < persisted_packets)
        {
            if (source_size != NULL)
            {
                *source_size = g_config.app_size;
            }
            return 1U;
        }
    }

    return 0U;
}

/* 从本地 Flash 构造一个 Provider DATA 回复帧。 */
static uint8_t Boot_BuildProviderFrame(uint16_t seq,
                                          uint8_t target,
                                          uint8_t frame[BOOT_DATA_SIZE])
{
    uint32_t source_size;
    uint32_t offset;
    uint32_t valid_len;

    if ((frame == NULL) || (Boot_ProviderPacketAvailable(seq, &source_size) == 0U))
    {
        return 0U;
    }

    offset = (uint32_t)seq * BOOT_DATA_PAYLOAD_SIZE;
    if (offset >= source_size)
    {
        return 0U;
    }

    valid_len = source_size - offset;
    if (valid_len > BOOT_DATA_PAYLOAD_SIZE)
    {
        valid_len = BOOT_DATA_PAYLOAD_SIZE;
    }

    memset(frame, 0, BOOT_DATA_SIZE);
    memset(&frame[8], 0xFF, BOOT_DATA_PAYLOAD_SIZE);

    frame[0] = target;
    frame[1] = BOOT_DATA_CMD_WRITE;
    Boot_WriteU16LE(&frame[2], seq);
    Boot_WriteU16LE(&frame[4], (g_session_active != 0U) ? g_session_id : 0U);
    frame[6] = 0U;
    frame[7] = 0U;

    memcpy(&frame[8],
           (const void *)(BOOT_APP_START_ADDR + offset),
           valid_len);

    return 1U;
}

/* 启动面向主机的缺包统计和逐项异步报告。 */
static void Boot_StartMissingReport(void)
{
    g_missing_count = Boot_MissingCount();
    g_missing_report_active = 1U;
    g_missing_count_sent = 0U;
    g_missing_scan_seq = 0U;
    g_missing_item_index = 0U;
}

/* 清除节点间恢复事务的全部运行时状态。 */
static void Boot_ResetPeerRecoveryState(void)
{
    memset(g_peer_missing_bitmap, 0, sizeof(g_peer_missing_bitmap));
    memset(g_peer_missing_expected, 0, sizeof(g_peer_missing_expected));
    memset(g_peer_missing_received, 0, sizeof(g_peer_missing_received));
    g_peer_active_bitmap = 0U;
    g_peer_report_complete_bitmap = 0U;
    g_peer_report_tx_active = 0U;
    g_peer_report_tx_count_sent = 0U;
    g_peer_report_tx_scan_seq = 0U;
    g_peer_report_tx_item_count = 0U;
    g_peer_report_items_after_ms = 0U;
    g_peer_recovery_started = 0U;
    g_election_pending = 0U;
    g_election_started_ms = 0U;
    g_coordinator_id = 0U;
    g_is_coordinator = 0U;
    g_recovery_members_bitmap = 0U;
    g_repair_round = 0U;
    g_coord_scan_seq = 0U;
    g_coord_current_seq = 0U;
    g_coord_provider_id = 0U;
    g_coord_wait_provider = 0U;
    g_provider_failed_bitmap = 0U;
    g_provider_wait_started_ms = 0U;
    g_coord_sweep_had_missing = 0U;
    g_coord_terminal = 0U;
    g_primary_members_bitmap = 0U;
    g_recovery_phase = BOOT_RECOVERY_PHASE_IDLE;
    g_phase_started_ms = 0U;
    g_verify_response_bitmap = 0U;
    g_verify_ok_bitmap = 0U;
    g_verify_context = 0U;
    g_prepared_app_size = 0U;
    g_prepared_app_crc32 = 0U;
    g_local_image_prepared = 0U;
    g_rollback_size = 0U;
    g_rollback_crc32 = 0U;
    g_rollback_meta_mask = 0U;
    g_guard_meta_tx_stage = 0U;
    g_rollback_prepared_bitmap = 0U;
    g_commit_expected_bitmap = 0U;
    g_commit_ack_bitmap = 0U;
    g_commit_armed = 0U;
    g_commit_tx_remaining = 0U;
    g_commit_execute_received = 0U;
    g_commit_due_ms = 0U;
}

/* 将本节点缺包 Bitmap 快照到节点间恢复表。 */
static void Boot_CaptureSelfMissing(void)
{
    uint8_t self_index = (uint8_t)(g_node_id - 1U);
    uint16_t seq;
    uint16_t count = 0U;
    memset(g_peer_missing_bitmap[self_index], 0, sizeof(g_peer_missing_bitmap[self_index]));
    for (seq = 0U; seq < g_total_packets; ++seq)
    {
        if (Boot_BitmapGet(seq) == 0U)
        {
            Boot_PeerMissingSet(g_node_id, seq);
            count++;
        }
    }
    g_peer_missing_expected[self_index] = count;
    g_peer_missing_received[self_index] = count;
    g_peer_active_bitmap |= Boot_NodeBit(g_node_id);
    g_peer_report_complete_bitmap |= Boot_NodeBit(g_node_id);
}

/* 启动本节点向协调者发送缺包总数和缺包项。 */
static void Boot_StartPeerMissingReport(void)
{
    Boot_CaptureSelfMissing();
    g_peer_report_tx_active = 1U;
    g_peer_report_tx_count_sent = 0U;
    g_peer_report_tx_scan_seq = 0U;
    g_peer_report_tx_item_count = 0U;
    g_peer_report_items_after_ms = HAL_GetTick() + BOOT_PEER_MISSING_ITEM_DELAY_MS;
}

/* 为下一轮修复清除其它节点的旧缺包报告。 */
static void Boot_ClearRemoteReportsForNextRound(void)
{
    uint8_t self_bit = Boot_NodeBit(g_node_id);
    memset(g_peer_missing_bitmap, 0, sizeof(g_peer_missing_bitmap));
    memset(g_peer_missing_expected, 0, sizeof(g_peer_missing_expected));
    memset(g_peer_missing_received, 0, sizeof(g_peer_missing_received));
    g_peer_report_complete_bitmap = 0U;
    if ((g_recovery_members_bitmap & self_bit) != 0U)
    {
        Boot_CaptureSelfMissing();
    }
}

/* 启动恢复协调者竞选并记录竞选起始时间。 */
static void Boot_StartPeerElection(void)
{
    if ((g_session_active == 0U) ||
        ((g_session_flags & BOOT_SESSION_FLAG_PEER_RECOVERY) == 0U) ||
        (g_write_active == 0U) ||
        (g_write_region != BOOT_WRITE_REGION_APP) ||
        (Boot_IsGuardProtected() != 0U) ||
        (g_peer_recovery_started != 0U)) return;
    g_peer_recovery_started = 1U;
    g_recovery_phase = BOOT_RECOVERY_PHASE_NEW_REPAIR;
    g_phase_started_ms = HAL_GetTick();
    Boot_StartPeerMissingReport();
    g_election_pending = 1U;
    g_election_started_ms = HAL_GetTick();
}

/* 判断所有参与节点是否都已完成缺包报告。 */
static uint8_t Boot_AllPeerReportsComplete(void)
{
    return ((g_peer_report_complete_bitmap & g_recovery_members_bitmap) == g_recovery_members_bitmap) ? 1U : 0U;
}

/* 判断指定数据包是否至少被一个恢复成员报告缺失。 */
static uint8_t Boot_AnyMemberMissing(uint16_t seq)
{
    uint8_t node;
    for (node = 1U; node <= BOOT_MAX_NODE_NUM; ++node)
        if (((g_recovery_members_bitmap & Boot_NodeBit(node)) != 0U) && (Boot_PeerMissingGet(node, seq) != 0U)) return 1U;
    return 0U;
}

/* 为指定数据包选择一个拥有该包的 Provider 节点。 */
static uint8_t Boot_SelectProvider(uint16_t seq)
{
    uint8_t node;
    uint8_t candidates = (g_primary_members_bitmap != 0U) ? g_primary_members_bitmap : g_recovery_members_bitmap;

    if ((g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_REPAIR) &&
        (g_guard_active != 0U) && (g_guard_node_id >= 1U) &&
        (g_guard_node_id <= BOOT_MAX_NODE_NUM))
    {
        if ((g_provider_failed_bitmap & Boot_NodeBit(g_guard_node_id)) != 0U) return 0U;
        return g_guard_node_id;
    }

    for (node = 1U; node <= BOOT_MAX_NODE_NUM; ++node)
    {
        uint8_t bit = Boot_NodeBit(node);
        if ((candidates & bit) == 0U) continue;
        if ((g_provider_failed_bitmap & bit) != 0U) continue;
        if ((g_guard_active != 0U) && (node == g_guard_node_id)) continue;
        if ((g_verify_ok_bitmap & bit) != 0U) return node;
        if (Boot_PeerMissingGet(node, seq) == 0U) return node;
    }
    return 0U;
}

/*
 * SESSION_BEGIN (0x07) 是自治升级的严格事务边界。
 * 它会主动丢弃上一个事务的全部 RAM 状态：异步 READ/Missing/Provider 任务、
 * 配置暂存写入、活动写入会话、包 Bitmap、Guard 角色、
 * Coordinator/Provider/修复状态以及旧 CRC 信息。
 *
 * 请求字段：
 *   seq       = Session 策略标志；
 *   param[0:1] = Session ID（小端序，必须非零）。
 *
 * 新会话之后必须发送 SESSION_CRC32，并可按需发送 SET_GUARD。
 * Session 0 保留给 Legacy 模式，此处拒绝该值。
 */
/* 处理主机发起的升级会话建立请求。 */
static void Boot_HandleSessionBegin(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint16_t session = Boot_ReadU16LE(&frame->param[0]);
    if (session == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_SESSION);
        return;
    }
    Boot_CancelAsyncTasks();
    Boot_StorageConfigStageAbort();
    g_write_active = 0U;
    g_app_erased = 0U;
    g_write_size = 0U;
    g_total_packets = 0U;
    Boot_BitmapClear();
    g_guard_active = 0U;
    g_guard_node_id = 0U;
    g_is_guard = 0U;
    g_status = BOOT_STATUS_IDLE;
    g_last_error = BOOT_ERR_NONE;
    g_session_active = 1U;
    g_session_flags = frame->seq;
    g_session_id = session;
    g_session_image_size = 0U;
    g_session_expected_crc32 = 0U;
    g_session_crc_valid = 0U;
    Boot_ResetPeerRecoveryState();
    Boot_WriteU16LE(&data[0], g_session_id);
    data[2] = g_session_flags;
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

/* 接收并校验当前升级会话声明的镜像 CRC32。 */
static void Boot_HandleSessionCrc32(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4];
    if (g_session_active == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_SESSION);
        return;
    }
    g_session_expected_crc32 = Boot_ReadU32LE(frame->param);
    g_session_crc_valid = 1U;
    Boot_WriteU32LE(data, g_session_expected_crc32);
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

/* 回复 Bootloader 协议版本信息。 */
static void Boot_HandleGetVersion(uint8_t cmd)
{
    uint8_t data[4] = {
        BOOT_VERSION_MAJOR,
        BOOT_VERSION_MINOR,
        BOOT_VERSION_PATCH,
        BOOT_VERSION_BUILD
    };

    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, data);
}

/* 回复设备/节点身份信息。 */
static void Boot_HandleGetDeviceId(uint8_t cmd)
{
    uint8_t data[4];
    Boot_WriteU32LE(data, DBGMCU->IDCODE);
    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, data);
}

/* 回复 Flash 布局、镜像容量和运行状态摘要。 */
static void Boot_HandleGetInfo(uint8_t cmd)
{
    uint8_t data[4] = {0};

    data[0] = g_node_id;
    data[1] = (g_config_valid != 0U) ? g_config.hardware_version : 0U;
    data[2] = g_app_valid;
    data[3] = g_config_valid;

    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, data);
}

/* 处理进入升级模式请求并返回当前节点状态。 */
static void Boot_HandleEnterBoot(uint8_t cmd)
{
    g_boot_requested = 1U;
    if (g_status == BOOT_STATUS_ERROR)
    {
        g_status = BOOT_STATUS_IDLE;
    }
    g_last_error = BOOT_ERR_NONE;
    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, NULL);
}

/* 处理 Guard 节点设置请求。 */
static void Boot_HandleSetGuard(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t guard_id = frame->seq;

    if ((guard_id < 1U) || (guard_id > BOOT_MAX_NODE_NUM))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    g_guard_node_id = guard_id;
    g_guard_active = 1U;
    g_is_guard = (g_node_id == guard_id) ? 1U : 0U;
    data[0] = guard_id;

    if (g_is_guard != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_GUARD, data);
    }
    else
    {
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
    }
}

/* 处理 Guard 保护释放请求。 */
static void Boot_HandleReleaseGuard(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t guard_id = frame->seq;

    if ((g_guard_active == 0U) || (guard_id != g_guard_node_id))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    data[0] = g_guard_node_id;
    g_guard_active = 0U;
    g_guard_node_id = 0U;
    g_is_guard = 0U;

    if (g_status == BOOT_STATUS_GUARD)
    {
        g_status = BOOT_STATUS_IDLE;
    }

    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

/*
 * ERASE (0x10) 在操作 APP Flash 前先使 APP 元数据失效，再擦除所有 APP 页。
 * 这里有意发送两次成功响应：
 *   1) 接受命令后立即发送 BOOT_STATUS_ERASE；
 *   2) 完成全部擦除后才发送 BOOT_STATUS_READY。
 * Host/CANPro 必须等待第二个响应后才能开始 WRITE。
 * 受保护的 Guard 在主升级阶段不会擦除自身 APP。
 */
/* 擦除 APP 或配置写入区域并初始化写入会话。 */
static void Boot_HandleErase(uint8_t cmd)
{
    if (Boot_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        g_last_error = BOOT_ERR_GUARD_PROTECTED;
        (void)Boot_SendResponse(cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    Boot_CancelAsyncTasks();
    Boot_StorageConfigStageAbort();
    g_write_active = 0U;
    g_app_erased = 0U;
    g_progress = 0U;
    g_status = BOOT_STATUS_ERASE;
    g_last_error = BOOT_ERR_NONE;

    //收到立即回复
     (void)Boot_SendResponse(cmd,BOOT_STATUS_ERASE,NULL);
    /* 掉电安全：操作 APP Flash 前先使元数据失效。 */
    if (Boot_StorageInvalidateApp(g_node_id) == 0U)
    {
        Boot_SendError(cmd, BOOT_ERR_CONFIG);
        return;
    }

    g_app_valid = 0U;
    if (Boot_ConfigLoad(&g_config) != 0U)
    {
        g_config_valid = 1U;
    }

    if (Boot_StorageEraseApp() == 0U)
    {
        Boot_SendError(cmd, BOOT_ERR_FLASH_ERASE);
        return;
    }

    Boot_BitmapClear();
    g_app_erased = 1U;
    g_progress = 100U;
    g_status = BOOT_STATUS_READY;
    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, NULL);
}

/*
 * WRITE (0x11) 建立逻辑写入会话，不携带固件数据；它只校验目标区域/大小、
 * 计算期望包数并清除每包 Bitmap。实际固件字节随后通过 64 字节逻辑
 * DATA 消息到达（每包有效载荷为 56 字节）。
 *
 * APP 写入必须先成功完成 ERASE。配置写入先暂存于 RAM，仅由 WRITE_END 提交。
 * Bootloader 区域始终受保护。
 */
/* 处理主机发来的写入控制帧，更新写入目标和包参数。 */
static void Boot_HandleWrite(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t region = frame->seq;
    uint32_t size = Boot_ReadU32LE(frame->param);
    uint32_t packets;

    if ((region == BOOT_WRITE_REGION_APP) && (size > 0U) && (size <= BOOT_APP_MAX_SIZE))
    {
        g_session_image_size = size;
    }

    if (Boot_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        g_last_error = BOOT_ERR_GUARD_PROTECTED;
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if (region == BOOT_WRITE_REGION_BOOTLOADER)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_PROTECTED_REGION);
        return;
    }

    if (region == BOOT_WRITE_REGION_APP)
    {
        if (g_app_erased == 0U)
        {
            Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
            return;
        }

        if ((size == 0U) || (size > BOOT_APP_MAX_SIZE))
        {
            Boot_SendError(frame->cmd, BOOT_ERR_SIZE);
            return;
        }
    }
    else if (region == BOOT_WRITE_REGION_CONFIG)
    {
        if ((size == 0U) || (size > BOOT_CONFIG_PAGE_SIZE))
        {
            Boot_SendError(frame->cmd, BOOT_ERR_SIZE);
            return;
        }

        if (Boot_StorageConfigStageBegin() == 0U)
        {
            Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }
    }
    else
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    packets = (size + BOOT_DATA_PAYLOAD_SIZE - 1UL) / BOOT_DATA_PAYLOAD_SIZE;
    if ((packets == 0U) || (packets > BOOT_MAX_PACKET_COUNT))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_SIZE);
        return;
    }

    Boot_CancelAsyncTasks();
    Boot_BitmapClear();

    g_write_active = 1U;
    g_write_region = region;
    g_write_size = size;
    g_total_packets = (uint16_t)packets;
    g_status = BOOT_STATUS_WRITE;
    g_last_error = BOOT_ERR_NONE;
    g_progress = 0U;

    data[0] = region;
    Boot_WriteU16LE(&data[1], g_total_packets);
    data[3] = ((region == BOOT_WRITE_REGION_APP) &&
               (g_session_active == 0U))
                  ? BOOT_FLASH_WINDOW_PACKETS
                  : 0U;
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_WRITE, data);
}

/*
 * READ (0x12) 启动非阻塞 Flash 读取。frame->seq 是请求字节数（1~255），
 * param[0:3] 是 32 位 Flash 绝对地址。
 *
 * 命令不会立即回复“已接受” ACK。Boot_TaskRead() 随后发送一个或多个 READY
 * 响应，每个响应最多携带 4 个数据字节。这样 CONTROL 帧始终固定为 8 字节，
 * 同时支持任意较小长度的 Flash 读取。
 */
/* 启动非阻塞 Flash READ 任务。 */
static void Boot_HandleRead(const Boot_ControlFrame_t *frame)
{
    uint32_t address;
    uint16_t length = frame->seq;

    if (length == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_LENGTH);
        return;
    }

    if (g_read_active != 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BUSY);
        return;
    }

    address = Boot_ReadU32LE(frame->param);
    if (Boot_StorageIsFlashRangeValid(address, length) == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    g_read_address = address;
    g_read_remaining = length;
    g_read_active = 1U;
}

/*
 * VERIFY (0x13) 是 Legacy 模式的校验/提交路径。
 *
 * 当 COORD_COMMIT 生效时有意禁止该路径，否则 Host VERIFY 会绕过分布式
 * VERIFY + Prepare/Commit，使某个节点在集群其余节点准备好之前就变为可启动。
 *
 * Legacy 成功必须同时满足：APP 写入会话有效、MissingCount==0、CRC32 完整匹配、
 * MSP/Reset_Handler 向量有效且元数据写入成功。只有这样才会将 app_valid 持久化为 1。
 */
/* 校验本地 APP 镜像大小、向量表和 CRC32。 */
static void Boot_HandleVerify(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4];
    uint32_t expected_crc;
    uint32_t actual_crc;

    if ((g_session_active != 0U) &&
        ((g_session_flags & BOOT_SESSION_FLAG_COORD_COMMIT) != 0U))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    if (Boot_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if ((g_write_active == 0U) || (g_write_region != BOOT_WRITE_REGION_APP))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    if (Boot_MissingCount() != 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    expected_crc = Boot_ReadU32LE(frame->param);
    g_status = BOOT_STATUS_VERIFY;

    actual_crc = Boot_CRC32((const uint8_t *)BOOT_APP_START_ADDR,
                                     g_write_size);

    if (actual_crc != expected_crc)
    {
        Boot_WriteU32LE(data, actual_crc);
        Boot_SetError(BOOT_ERR_CRC_MISMATCH, 1U);
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_ERROR, data);
        return;
    }

    if (Boot_RuntimeValidateImage(g_write_size, actual_crc) == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_APP_INVALID);
        return;
    }

    if (Boot_StorageSaveAppMetadata(g_write_size, actual_crc, 1U) == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
        return;
    }

    if (Boot_ConfigLoad(&g_config) == 0U)
    {
        g_config_valid = 0U;
        Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
        return;
    }

    g_config_valid = 1U;
    g_app_valid = 1U;
    g_status = BOOT_STATUS_READY;
    g_last_error = BOOT_ERR_NONE;
    g_progress = 100U;

    Boot_WriteU32LE(data, actual_crc);
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

/*
 * WRITE_END (0x14) 关闭第一阶段固件数据流，并根据本地包 Bitmap 决定下一步协议动作。
 *
 * Legacy/单播：
 *   - Missing > 0：进入 REPAIR，并向 Host 发送 MISSING_COUNT/MISSING_ITEM；
 *   - Missing = 0：进入 VERIFY，等待 Host 发送 Legacy VERIFY。
 *
 * 启用 PEER_RECOVERY 的自治/广播模式：
 *   - 详细缺包信息只通过节点间控制帧发送；
 *   - 协调者竞选只在这里、初次完整广播结束后启动；
 *   - 第一次 DATA 注入期间没有固定主节点。
 *
 * 配置区 WRITE_END 提交 RAM 暂存页，而不是执行 APP 校验。
 */
/* 处理写入结束请求，触发缺包报告或分布式恢复流程。 */
static void Boot_HandleWriteEnd(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint16_t missing;
    uint8_t autonomous;

    if (Boot_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if (g_write_active == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    /* Host必须等待最后一个WINDOW_READY；否则当前窗口尚未落入Flash。 */
    if ((g_session_active == 0U) &&
        (g_write_region == BOOT_WRITE_REGION_APP) &&
        (Boot_FlashWindowsBusy() != 0U))
    {
        Boot_WriteU16LE(data, g_received_packets);
        data[2] = BOOT_FLASH_WINDOW_PACKETS;
        data[3] = 0U;
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_WRITE, data);
        return;
    }

    missing = Boot_MissingCount();
    Boot_WriteU16LE(data, missing);
    autonomous = ((frame->target == BOOT_BROADCAST_ID) &&
                  (g_session_active != 0U) &&
                  ((g_session_flags & BOOT_SESSION_FLAG_PEER_RECOVERY) != 0U)) ? 1U : 0U;

    /* 节点竞选有意延迟到第一次广播结束后。 */
    if (frame->target == BOOT_BROADCAST_ID)
    {
        Boot_StartPeerElection();
    }

    if (missing != 0U)
    {
        g_status = BOOT_STATUS_REPAIR;
        if (autonomous == 0U) Boot_StartMissingReport();
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_REPAIR, data);
        return;
    }

    if (g_write_region == BOOT_WRITE_REGION_CONFIG)
    {
        if (Boot_StorageConfigStageCommit() == 0U)
        {
            Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }

        g_config_valid = Boot_ConfigLoad(&g_config);
        if (g_config_valid != 0U)
        {
            g_app_valid = Boot_RuntimeValidatePersistedApp(&g_config);
        }
        else
        {
            g_app_valid = 0U;
        }
        g_write_active = 0U;
        g_status = BOOT_STATUS_READY;
        g_progress = 100U;
        Boot_StartMissingReport(); /* Sends MISSING_COUNT = 0. */
        (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
        return;
    }

    g_status = BOOT_STATUS_VERIFY;
    g_progress = 100U;
    if (autonomous == 0U) Boot_StartMissingReport(); /* Legacy Host 缺包报告。 */
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_VERIFY, data);
}

/*
 * PROVIDER_GRANT (0x17) 是 Legacy 模式由 Host 指定的修复机制。
 * Host 将该 CONTROL 命令单播给一个 Provider 节点，请求它向目标节点/广播地址
 * 发送指定 Sequence 范围的数据。
 *
 * 广播授予会按设计静默忽略：若多个健康节点接受同一授予，它们会同时发送
 * 相同 DATA，破坏总线确定性仲裁和修复行为。
 *
 * 第一个响应为 WRITE（任务已接受）。只有在所有 DATA 交给传输层，且
 * Boot_FlushTx() 确认经典 CAN 分片/原生 FD 帧均已排空后，才发送最终 READY。
 */
/* 处理主机授予的 Provider 补包任务。 */
static void Boot_HandleProviderGrant(const Boot_ControlFrame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t data_target;
    uint16_t start_seq;
    uint16_t count;
    uint32_t end_seq;

    /* Provider 选择必须是单播。广播授予可能导致多个健康节点同时发送，
     * 这是本协议禁止的行为。 */
    if (frame->target == BOOT_BROADCAST_ID)
    {
        return;
    }

    data_target = frame->seq;
    start_seq = Boot_ReadU16LE(&frame->param[0]);
    count = Boot_ReadU16LE(&frame->param[2]);

    if (!(((data_target >= 1U) && (data_target <= BOOT_MAX_NODE_NUM)) ||
          (data_target == BOOT_BROADCAST_ID)))
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    if (count == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_LENGTH);
        return;
    }

    end_seq = (uint32_t)start_seq + (uint32_t)count;
    if (end_seq > 65536UL)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_SEQUENCE);
        return;
    }

    if (Boot_ProviderPacketAvailable(start_seq, NULL) == 0U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_PROVIDER_SOURCE);
        return;
    }

    g_provider_target = data_target;
    g_provider_next_seq = start_seq;
    g_provider_remaining = count;
    g_provider_active = 1U;
    g_provider_done_pending = 0U;
    g_provider_peer_mode = 0U;

    data[0] = data_target;
    Boot_WriteU16LE(&data[1], start_seq);
    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_WRITE, data);
}

/* 处理普通跳转或一次性 Trial Jump 请求。 */
static void Boot_HandleJumpApp(const Boot_ControlFrame_t *frame)
{
    uint8_t trial_mode;

    if (frame == NULL) return;

    /* JUMP_APP Byte2/seq：0=原普通跳转，1=一次性 Trial Jump。 */
    trial_mode = frame->seq;
    if (trial_mode > 1U)
    {
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    if ((trial_mode != 0U) &&
        (g_session_active != 0U) &&
        ((g_session_flags & BOOT_SESSION_FLAG_COORD_COMMIT) != 0U))
    {
        /* 第一版 Trial 不跨复位保存 Coordinator Session 状态。 */
        Boot_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    if ((g_app_valid == 0U) || (Boot_RuntimeValidatePersistedApp(&g_config) == 0U))
    {
        g_app_valid = 0U;
        Boot_SendError(frame->cmd, BOOT_ERR_APP_INVALID);
        return;
    }

    g_config_valid = 1U;

    if (trial_mode != 0U)
    {
        /* APP 证明能主动返回前，先持久化 app_valid=0，禁止自动再次启动。 */
        if (Boot_StorageSaveAppMetadata(g_config.app_size, g_config.app_crc32, 0U) == 0U)
        {
            Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }

        if (Boot_ConfigLoad(&g_config) == 0U)
        {
            g_config_valid = 0U;
            g_app_valid = 0U;
            Boot_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }

        g_config_valid = 1U;
        g_app_valid = 0U;
        Boot_RuntimeSetTrialPending();
    }

    (void)Boot_SendResponse(frame->cmd, BOOT_STATUS_READY, NULL);

    if (trial_mode != 0U)
    {
        Boot_RuntimeStartTrialWatchdog();
    }

    Boot_FlushTx(20U);
    Boot_RuntimeJumpToApp();
}

/* 刷新发送队列后复位 MCU。 */
static void Boot_HandleReset(uint8_t cmd)
{
    (void)Boot_SendResponse(cmd, BOOT_STATUS_READY, NULL);
    Boot_FlushTx(20U);
    HAL_Delay(1U);
    NVIC_SystemReset();
}

/* 回复当前状态、错误码和升级进度。 */
static void Boot_HandleGetStatus(uint8_t cmd)
{
    uint8_t data[4] = {0};

    data[0] = (uint8_t)g_status;
    data[1] = (uint8_t)g_last_error;
    data[2] = g_progress;
    data[3] = 0U; /* reserved */

    (void)Boot_SendResponse(cmd, (uint8_t)g_status, data);
}

/*
 * WINDOW_READY既是节点的主动窗口提交通知，也是Host超时后的查询命令。
 * 查询响应返回第一个尚未成功写入Flash的Sequence，ACK丢失时无需猜测。
 */
static void Boot_HandleWindowStatus(uint8_t cmd)
{
    uint8_t data[4];

    if ((g_write_active == 0U) ||
        (g_write_region != BOOT_WRITE_REGION_APP) ||
        (g_session_active != 0U))
    {
        Boot_SendError(cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    Boot_WriteU16LE(data, Boot_FirstMissingSequence());
    data[2] = BOOT_FLASH_WINDOW_PACKETS;
    data[3] = Boot_FlashWindowFreeCount();
    (void)Boot_SendResponse(cmd, BOOT_STATUS_WRITE, data);
}

/* 取消当前写入、读取、Provider 和恢复异步任务。 */
static void Boot_HandleAbort(uint8_t cmd)
{
    Boot_CancelAsyncTasks();
    Boot_StorageConfigStageAbort();
    g_write_active = 0U;
    g_status = BOOT_STATUS_ERROR;
    g_last_error = BOOT_ERR_ABORTED;
    (void)Boot_SendResponse(cmd, BOOT_STATUS_ERROR, NULL);
}

/* 初始化协议核心状态、持久化配置和 Trial/Boot 请求判定。 */
static void Boot_CoreInit(uint8_t default_node_id)
{
    uint8_t trial_pending;
    uint8_t app_returned;

    memset(&g_config, 0, sizeof(g_config));
    Boot_BitmapClear();
    Boot_CancelAsyncTasks();

    g_config_valid = Boot_ConfigLoad(&g_config);

    if ((g_config_valid != 0U) &&
        (g_config.node_id >= 1U) &&
        (g_config.node_id <= BOOT_MAX_NODE_NUM))
    {
        g_node_id = g_config.node_id;
    }
    else if ((default_node_id >= 1U) && (default_node_id <= BOOT_MAX_NODE_NUM))
    {
        g_node_id = default_node_id;
    }
    else
    {
        g_node_id = BOOT_DEFAULT_NODE_ID;
    }

    g_app_valid = Boot_RuntimeValidatePersistedApp(&g_config);
    if (g_app_valid != 0U)
    {
        g_config_valid = 1U;
    }

    g_boot_requested = Boot_RuntimeConsumeBootRequest();
    app_returned = g_boot_requested;
    trial_pending = Boot_RuntimeConsumeTrialPending();
    g_guard_active = 0U;
    g_guard_node_id = 0U;
    g_is_guard = 0U;
    g_app_erased = 0U;
    g_write_active = 0U;
    g_write_size = 0U;
    g_total_packets = 0U;
    g_session_active = 0U;
    g_session_flags = 0U;
    g_session_id = 0U;
    Boot_ResetPeerRecoveryState();
    g_progress = 0U;
    g_last_error = BOOT_ERR_NONE;
    g_status = BOOT_STATUS_IDLE;

    if (trial_pending != 0U)
    {
        /* Trial 结果判定完成后，本次启动始终停留在 Bootloader。 */
        g_boot_requested = 1U;

        if (app_returned != 0U)
        {
            /* APP 已处理 ENTER_BOOT；恢复可信状态前再次完整校验 Flash。 */
            if (Boot_RuntimeValidateTrialImage(&g_config) == 0U)
            {
                g_app_valid = 0U;
                g_last_error = BOOT_ERR_APP_INVALID;
                g_status = BOOT_STATUS_ERROR;
            }
            else if ((Boot_StorageSaveAppMetadata(g_config.app_size, g_config.app_crc32, 1U) == 0U) ||
                     (Boot_ConfigLoad(&g_config) == 0U))
            {
                g_config_valid = 0U;
                g_app_valid = 0U;
                g_last_error = BOOT_ERR_CONFIG;
                g_status = BOOT_STATUS_ERROR;
            }
            else
            {
                g_config_valid = 1U;
                g_app_valid = 1U;
                g_last_error = BOOT_ERR_NONE;
                g_status = BOOT_STATUS_IDLE;
            }
        }
        else
        {
            /* 没有 BKP0R 请求：APP 未完成主动返回握手。 */
            g_app_valid = 0U;
            g_last_error = BOOT_ERR_APP_TRIAL_TIMEOUT;
            g_status = BOOT_STATUS_ERROR;
        }
    }
}

/* 判断当前启动是否满足自动跳转到有效 APP 的条件。 */
uint8_t Boot_ShouldJumpApp(void)
{
    return ((g_boot_requested == 0U) && (g_app_valid != 0U)) ? 1U : 0U;
}

/* 执行到 APP 的最终硬件状态交接。 */
void Boot_JumpApp(void)
{
    if (Boot_RuntimeValidatePersistedApp(&g_config) != 0U)
    {
        g_app_valid = 1U;
        g_config_valid = 1U;
        Boot_RuntimeJumpToApp();
    }

    g_app_valid = 0U;
    g_status = BOOT_STATUS_ERROR;
    g_last_error = BOOT_ERR_APP_INVALID;
}

/* 校验并分发一帧主机控制命令。 */
static void Boot_ProcessControl(const uint8_t *data, uint8_t len)
{
    Boot_ControlFrame_t frame;

    if ((data == NULL) || (len != BOOT_CONTROL_SIZE))
    {
        return;
    }

    if (Boot_CRC8(data, 7U) != data[7])
    {
        /* target/cmd 字段尚未通过校验，不能据此生成错误响应。 */
        g_last_error = BOOT_ERR_BAD_CRC;
        return;
    }

    memcpy(&frame, data, sizeof(frame));

    if ((frame.target != g_node_id) && (frame.target != BOOT_BROADCAST_ID))
    {
        return;
    }

    switch ((Boot_Command_t)frame.cmd)
    {
    case BOOT_CMD_GET_VERSION:
        Boot_HandleGetVersion(frame.cmd);
        break;

    case BOOT_CMD_GET_DEVICE_ID:
        Boot_HandleGetDeviceId(frame.cmd);
        break;

    case BOOT_CMD_GET_INFO:
        Boot_HandleGetInfo(frame.cmd);
        break;

    case BOOT_CMD_ENTER_BOOT:
        Boot_HandleEnterBoot(frame.cmd);
        break;

    case BOOT_CMD_SET_GUARD:
        Boot_HandleSetGuard(&frame);
        break;

    case BOOT_CMD_RELEASE_GUARD:
        Boot_HandleReleaseGuard(&frame);
        break;

    case BOOT_CMD_SESSION_BEGIN:
        Boot_HandleSessionBegin(&frame);
        break;

    case BOOT_CMD_SESSION_CRC32:
        Boot_HandleSessionCrc32(&frame);
        break;

    case BOOT_CMD_ERASE:
        Boot_HandleErase(frame.cmd);
        break;

    case BOOT_CMD_WRITE:
        Boot_HandleWrite(&frame);
        break;

    case BOOT_CMD_READ:
        Boot_HandleRead(&frame);
        break;

    case BOOT_CMD_VERIFY:
        Boot_HandleVerify(&frame);
        break;

    case BOOT_CMD_WRITE_END:
        Boot_HandleWriteEnd(&frame);
        break;

    case BOOT_CMD_PROVIDER_GRANT:
        Boot_HandleProviderGrant(&frame);
        break;

    case BOOT_CMD_ABORT:
        Boot_HandleAbort(frame.cmd);
        break;

    case BOOT_CMD_JUMP_APP:
        Boot_HandleJumpApp(&frame);
        break;

    case BOOT_CMD_RESET:
        Boot_HandleReset(frame.cmd);
        break;

    case BOOT_CMD_GET_STATUS:
        Boot_HandleGetStatus(frame.cmd);
        break;

    case BOOT_CMD_WINDOW_READY:
        Boot_HandleWindowStatus(frame.cmd);
        break;

    case BOOT_CMD_MISSING_COUNT:
    case BOOT_CMD_MISSING_ITEM:
    default:
        Boot_SendError(frame.cmd, BOOT_ERR_BAD_STATE);
        break;
    }
}

/* 选择一个已通过完整镜像校验的节点作为回滚/补包 Provider。 */
static uint8_t Boot_SelectVerifiedProvider(void)
{
    uint8_t node;
    for (node = 1U; node <= BOOT_MAX_NODE_NUM; ++node)
    {
        uint8_t bit = Boot_NodeBit(node);
        if (((g_primary_members_bitmap & bit) != 0U) && ((g_verify_ok_bitmap & bit) != 0U)) return node;
    }
    return 0U;
}

/* 按恢复上下文校验本地镜像并记录结果。 */
static uint8_t Boot_VerifyLocalForContext(uint8_t context)
{
    if (context == (uint8_t)BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY)
    {
        return Boot_PrepareVerifiedImage(g_rollback_crc32);
    }
    if ((context == (uint8_t)BOOT_RECOVERY_PHASE_NEW_VERIFY) ||
        (context == (uint8_t)BOOT_RECOVERY_PHASE_GUARD_UPDATE))
    {
        if (g_session_crc_valid == 0U) return 0U;
        return Boot_PrepareVerifiedImage(g_session_expected_crc32);
    }
    return 0U;
}

/* 将恢复流程置为失败终态并通知相关节点。 */
static void Boot_FinalRecoveryFailure(void)
{
    g_coord_terminal = 1U;
    g_recovery_phase = BOOT_RECOVERY_PHASE_FAILED;
    g_status = BOOT_STATUS_ERROR;
    g_last_error = BOOT_ERR_RECOVERY_FAILED;
    (void)Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_RECOVERY_FAILED,
                             g_node_id, g_session_id, g_repair_round);
}

/* 启动回滚元数据同步及回滚镜像修复流程。 */
static void Boot_StartRollback(void)
{
    if (((g_session_flags & BOOT_SESSION_FLAG_GUARD_ROLLBACK) == 0U) ||
        (g_guard_active == 0U) || (g_guard_node_id < 1U) ||
        (g_guard_node_id > BOOT_MAX_NODE_NUM))
    {
        Boot_FinalRecoveryFailure();
        return;
    }
    g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_META;
    g_phase_started_ms = HAL_GetTick();
    g_rollback_meta_mask = 0U;
    g_rollback_size = 0U;
    g_rollback_crc32 = 0U;
    g_rollback_prepared_bitmap = 0U;
    g_verify_response_bitmap = 0U;
    g_verify_ok_bitmap = 0U;
    g_coord_wait_provider = 0U;
    g_repair_round = 0U;
    if (Boot_SendPeerFrame(g_guard_node_id, BOOT_CMD_ROLLBACK_REQUEST,
                           g_node_id, g_session_id, 0U) == 0U)
    {
        g_phase_started_ms = HAL_GetTick();
    }
}

/* 处理协调者阶段超时或成员失败。 */
static void Boot_CoordinatorPhaseFailure(void)
{
    if ((g_recovery_phase == BOOT_RECOVERY_PHASE_NEW_REPAIR) ||
        (g_recovery_phase == BOOT_RECOVERY_PHASE_NEW_VERIFY))
    {
        Boot_StartRollback();
    }
    else
    {
        Boot_FinalRecoveryFailure();
    }
}

/* 启动分布式镜像提交准备阶段。 */
static void Boot_StartCommit(void)
{
    uint8_t self_bit = Boot_NodeBit(g_node_id);
    g_recovery_phase = BOOT_RECOVERY_PHASE_COMMIT;
    g_phase_started_ms = HAL_GetTick();
    g_commit_expected_bitmap = g_primary_members_bitmap;
    if ((g_guard_active != 0U) && (g_guard_node_id >= 1U) && (g_guard_node_id <= BOOT_MAX_NODE_NUM))
        g_commit_expected_bitmap |= Boot_NodeBit(g_guard_node_id);
    g_commit_ack_bitmap = 0U;
    g_commit_tx_remaining = 0U;
    g_commit_execute_received = 0U;
    if ((g_commit_expected_bitmap & self_bit) != 0U)
    {
        if (Boot_ArmCommitLocal() == 0U) { Boot_FinalRecoveryFailure(); return; }
        g_commit_ack_bitmap |= self_bit;
    }
    (void)Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_COMMIT_PREPARE,
                             g_node_id, g_session_id, g_commit_expected_bitmap);
}

/* 启动 Guard 节点配置和镜像元数据更新阶段。 */
static void Boot_StartGuardUpdate(void)
{
    if ((g_guard_active == 0U) || (g_guard_node_id == 0U))
    {
        Boot_StartCommit();
        return;
    }
    g_recovery_phase = BOOT_RECOVERY_PHASE_GUARD_UPDATE;
    g_phase_started_ms = HAL_GetTick();
    g_repair_round = 0U;
    g_coord_wait_provider = 0U;
    g_recovery_members_bitmap = Boot_NodeBit(g_guard_node_id);
    (void)Boot_SendPeerFrame(g_guard_node_id, BOOT_CMD_GUARD_UPDATE_BEGIN,
                             g_node_id, g_session_id, 0U);
}

/* 向恢复成员广播指定上下文的镜像校验请求。 */
static void Boot_StartVerifyContext(uint8_t context)
{
    uint8_t target = BOOT_BROADCAST_ID;
    uint8_t self_bit = Boot_NodeBit(g_node_id);
    g_verify_context = context;
    g_verify_response_bitmap = 0U;
    g_verify_ok_bitmap &= (uint8_t)~g_recovery_members_bitmap;
    g_phase_started_ms = HAL_GetTick();
    if (context == (uint8_t)BOOT_RECOVERY_PHASE_NEW_VERIFY)
        g_recovery_phase = BOOT_RECOVERY_PHASE_NEW_VERIFY;
    else if (context == (uint8_t)BOOT_RECOVERY_PHASE_GUARD_UPDATE)
        target = g_guard_node_id;
    else if (context == (uint8_t)BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY)
        g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY;

    if ((g_recovery_members_bitmap & self_bit) != 0U)
    {
        g_verify_response_bitmap |= self_bit;
        if (Boot_VerifyLocalForContext(context) != 0U) g_verify_ok_bitmap |= self_bit;
        else { Boot_CoordinatorPhaseFailure(); return; }
    }
    (void)Boot_SendPeerFrame(target, BOOT_CMD_VERIFY_REQUEST, g_node_id, g_session_id, context);
}

/*
 * 消费一帧 8 字节节点间 Peer Control 控制帧。
 *
 * 逻辑布局：
 *   Byte0    目标节点或 0xFF
 *   Byte1    节点间命令
 *   Byte2    源节点
 *   Byte3-4  Session ID（小端序）
 *   Byte5-6  16 位命令值（小端序）
 *   Byte7    CRC8
 *
 * CAN 适配层已经检查 CAN ID 0x600+N 与 Byte2=N 一致。
 * 核心层随后拒绝错误目标、错误 Session、非法源节点和错误 CRC。
 * 大多数 Peer 命令有意不发送通用 ACK，协议完成由配对命令（VERIFY_RESULT、
 * COMMIT_ACK 等）、DATA 后的 PROVIDER_DONE 或超时/下一恢复阶段表示。
 */
/* 校验并分发节点间协调控制帧。 */
static void Boot_ProcessPeerControl(const uint8_t *data, uint8_t len)
{
    Boot_ControlFrame_t frame;
    uint16_t session;
    uint16_t value;
    uint8_t source;
    uint8_t source_index;
    uint8_t source_bit;

    if ((data == NULL) || (len != BOOT_CONTROL_SIZE)) return;
    if (Boot_CRC8(data, 7U) != data[7]) return;
    memcpy(&frame, data, sizeof(frame));
    if ((frame.target != g_node_id) && (frame.target != BOOT_BROADCAST_ID)) return;

    session = Boot_ReadU16LE(&frame.param[0]);
    value = Boot_ReadU16LE(&frame.param[2]);
    source = frame.seq;
    if ((g_session_active == 0U) || (session != g_session_id)) return;
    if ((source < 1U) || (source > BOOT_MAX_NODE_NUM)) return;

    source_index = (uint8_t)(source - 1U);
    source_bit = Boot_NodeBit(source);

    switch ((Boot_Command_t)frame.cmd)
    {    case BOOT_CMD_MISSING_COUNT:
        g_peer_active_bitmap |= source_bit;
        if ((g_is_coordinator != 0U) && (g_coord_terminal == 0U) &&
            (g_recovery_phase == BOOT_RECOVERY_PHASE_NEW_REPAIR))
        {
            g_recovery_members_bitmap |= source_bit;
            g_primary_members_bitmap |= source_bit;
        }
        memset(g_peer_missing_bitmap[source_index], 0, sizeof(g_peer_missing_bitmap[source_index]));
        g_peer_missing_expected[source_index] = value;
        g_peer_missing_received[source_index] = 0U;
        g_peer_report_complete_bitmap &= (uint8_t)~source_bit;
        if (value == 0U) g_peer_report_complete_bitmap |= source_bit;
        break;

    case BOOT_CMD_MISSING_ITEM:
        if (value >= g_total_packets) break;
        if (Boot_PeerMissingGet(source, value) == 0U)
        {
            Boot_PeerMissingSet(source, value);
            g_peer_missing_received[source_index]++;
        }
        if ((g_peer_missing_expected[source_index] != 0U) &&
            (g_peer_missing_received[source_index] >= g_peer_missing_expected[source_index]))
            g_peer_report_complete_bitmap |= source_bit;
        break;

    case BOOT_CMD_COORDINATOR_CLAIM:
        if ((g_election_pending != 0U) && (g_node_id < source)) break;
        if ((g_coordinator_id == 0U) || (source < g_coordinator_id))
        {
            g_coordinator_id = source;
            g_is_coordinator = (source == g_node_id) ? 1U : 0U;
            g_primary_members_bitmap = (uint8_t)(value & 0x00FFU);
            g_recovery_members_bitmap = g_primary_members_bitmap;
            g_election_pending = 0U;
            if (g_is_coordinator != 0U) g_coord_scan_seq = 0U;
        }
        break;

    case BOOT_CMD_PROVIDER_ASSIGN:
        if ((frame.target != g_node_id) || (source != g_coordinator_id) ||
            (g_provider_active != 0U) || (Boot_ProviderPacketAvailable(value, NULL) == 0U)) break;
        g_provider_target = BOOT_BROADCAST_ID;
        g_provider_next_seq = value;
        g_provider_remaining = 1U;
        g_provider_peer_seq = value;
        g_provider_peer_mode = 1U;
        g_provider_active = 1U;
        g_provider_done_pending = 0U;
        break;

    case BOOT_CMD_FULL_STREAM:
        if ((frame.target != g_node_id) || (source != g_coordinator_id) || (g_provider_active != 0U)) break;
        {
            uint32_t source_size = 0U;
            uint32_t packets;
            uint8_t data_target = (uint8_t)(value & 0x00FFU);
            if (!(((data_target >= 1U) && (data_target <= BOOT_MAX_NODE_NUM)) ||
                  (data_target == BOOT_BROADCAST_ID))) break;
            if (Boot_ProviderPacketAvailable(0U, &source_size) == 0U) break;
            packets = (source_size + BOOT_DATA_PAYLOAD_SIZE - 1UL) / BOOT_DATA_PAYLOAD_SIZE;
            if ((packets == 0U) || (packets > 65535UL)) break;
            g_provider_target = data_target;
            g_provider_next_seq = 0U;
            g_provider_remaining = (uint16_t)packets;
            g_provider_peer_seq = 0xFFFFU;
            g_provider_peer_mode = 1U;
            g_provider_active = 1U;
            g_provider_done_pending = 0U;
        }
        break;

    case BOOT_CMD_PROVIDER_DONE:
        if ((g_is_coordinator != 0U) && (frame.target == g_node_id) &&
            (source == g_coord_provider_id) && (value == g_coord_current_seq))
        {
            Boot_CoordinatorProviderDone(value);
        }
        break;

    case BOOT_CMD_REPAIR_ROUND_END:
        if (source != g_coordinator_id) break;
        g_repair_round = (uint8_t)(value & 0x00FFU);
        if (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_UPDATE)
            g_recovery_phase = BOOT_RECOVERY_PHASE_GUARD_REPAIR;
        else if (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_PREP)
            g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_REPAIR;
        g_status = BOOT_STATUS_REPAIR;
        if ((g_is_coordinator == 0U) &&
            ((g_recovery_members_bitmap & Boot_NodeBit(g_node_id)) != 0U))
        {
            Boot_StartPeerMissingReport();
        }
        break;

    case BOOT_CMD_VERIFY_REQUEST:
        if (source != g_coordinator_id) break;
        {
            uint8_t context = (uint8_t)(value & 0x00FFU);
            uint8_t ok = 0U;
            uint8_t self_bit = Boot_NodeBit(g_node_id);
            if ((g_recovery_members_bitmap & self_bit) == 0U) break;
            g_verify_context = context;
            if (context == (uint8_t)BOOT_RECOVERY_PHASE_NEW_VERIFY)
                g_recovery_phase = BOOT_RECOVERY_PHASE_NEW_VERIFY;
            else if (context == (uint8_t)BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY)
                g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY;
            ok = Boot_VerifyLocalForContext(context);
            if (ok == 0U) g_status = BOOT_STATUS_ERROR;
            (void)Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_VERIFY_RESULT,
                                     g_node_id, g_session_id,
                                     (uint16_t)(((uint16_t)context << 8U) | (ok ? 1U : 0U)));
        }
        break;

    case BOOT_CMD_VERIFY_RESULT:        if ((g_is_coordinator != 0U) && (frame.target == g_node_id))
        {
            uint8_t context = (uint8_t)((value >> 8U) & 0xFFU);
            uint8_t ok = (uint8_t)(value & 0xFFU);
            if ((context != g_verify_context) ||
                ((g_recovery_members_bitmap & source_bit) == 0U)) break;
            g_verify_response_bitmap |= source_bit;
            if (ok != 0U) g_verify_ok_bitmap |= source_bit;
            else Boot_CoordinatorPhaseFailure();
        }
        break;

    case BOOT_CMD_GUARD_UPDATE_BEGIN:
        if ((source != g_coordinator_id) || (g_node_id != g_guard_node_id) ||
            (g_is_guard == 0U) || (g_session_crc_valid == 0U) ||
            (g_session_image_size == 0U)) break;
        g_recovery_phase = BOOT_RECOVERY_PHASE_GUARD_UPDATE;
        g_recovery_members_bitmap = Boot_NodeBit(g_node_id);
        g_is_guard = 0U;
        value = Boot_PrepareAppWriteInternal(g_session_image_size) ? 1U : 0U;
        (void)Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_GUARD_UPDATE_READY,
                                 g_node_id, g_session_id, value);
        break;

    case BOOT_CMD_GUARD_UPDATE_READY:
        if ((g_is_coordinator != 0U) && (frame.target == g_node_id) &&
            (source == g_guard_node_id) &&
            (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_UPDATE))
        {
            uint8_t provider;
            if (value == 0U) { Boot_CoordinatorPhaseFailure(); break; }
            provider = Boot_SelectVerifiedProvider();
            if (provider == 0U) { Boot_CoordinatorPhaseFailure(); break; }
            g_coord_provider_id = provider;
            g_coord_current_seq = 0xFFFFU;
            g_coord_wait_provider = 1U;
            g_provider_wait_started_ms = HAL_GetTick();
            g_phase_started_ms = HAL_GetTick();
            if (provider == g_node_id)
            {
                uint32_t source_size = 0U;
                if (Boot_ProviderPacketAvailable(0U, &source_size) == 0U)
                {
                    Boot_CoordinatorPhaseFailure();
                    break;
                }
                g_provider_target = g_guard_node_id;
                g_provider_next_seq = 0U;
                g_provider_remaining = (uint16_t)((source_size + BOOT_DATA_PAYLOAD_SIZE - 1UL) /
                                                   BOOT_DATA_PAYLOAD_SIZE);
                g_provider_peer_seq = 0xFFFFU;
                g_provider_peer_mode = 1U;
                g_provider_active = 1U;
                g_provider_done_pending = 0U;
            }
            else if (Boot_SendPeerFrame(provider, BOOT_CMD_FULL_STREAM, g_node_id,
                                        g_session_id, g_guard_node_id) == 0U)
            {
                g_coord_wait_provider = 0U;
            }
        }
        break;

    case BOOT_CMD_ROLLBACK_REQUEST:
        if ((g_node_id != g_guard_node_id) || (g_is_guard == 0U) ||
            (Boot_RuntimeValidatePersistedApp(&g_config) == 0U)) break;
        g_rollback_size = g_config.app_size;
        g_rollback_crc32 = g_config.app_crc32;
        g_guard_meta_tx_stage = 1U;
        break;

    case BOOT_CMD_ROLLBACK_SIZE_LO:        if (source == g_guard_node_id)
        {
            g_rollback_size = (g_rollback_size & 0xFFFF0000UL) | (uint32_t)value;
            g_rollback_meta_mask |= 0x01U;
        }
        break;

    case BOOT_CMD_ROLLBACK_SIZE_HI:
        if (source == g_guard_node_id)
        {
            g_rollback_size = (g_rollback_size & 0x0000FFFFUL) | ((uint32_t)value << 16U);
            g_rollback_meta_mask |= 0x02U;
        }
        break;

    case BOOT_CMD_ROLLBACK_CRC_LO:
        if (source == g_guard_node_id)
        {
            g_rollback_crc32 = (g_rollback_crc32 & 0xFFFF0000UL) | (uint32_t)value;
            g_rollback_meta_mask |= 0x04U;
        }
        break;

    case BOOT_CMD_ROLLBACK_CRC_HI:
        if (source == g_guard_node_id)
        {
            g_rollback_crc32 = (g_rollback_crc32 & 0x0000FFFFUL) | ((uint32_t)value << 16U);
            g_rollback_meta_mask |= 0x08U;
        }
        break;

    case BOOT_CMD_ROLLBACK_BEGIN:
        if ((source != g_coordinator_id) ||
            ((g_primary_members_bitmap & Boot_NodeBit(g_node_id)) == 0U) ||
            (g_rollback_meta_mask != 0x0FU)) break;
        g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_PREP;
        value = Boot_PrepareAppWriteInternal(g_rollback_size) ? 1U : 0U;
        (void)Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_ROLLBACK_PREPARED,
                                 g_node_id, g_session_id, value);
        break;

    case BOOT_CMD_ROLLBACK_PREPARED:        if ((g_is_coordinator != 0U) && (frame.target == g_node_id) &&
            ((g_primary_members_bitmap & source_bit) != 0U) &&
            (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_PREP))
        {
            if (value == 0U) { Boot_FinalRecoveryFailure(); break; }
            g_rollback_prepared_bitmap |= source_bit;
        }
        break;

    case BOOT_CMD_COMMIT_PREPARE:
        if (source != g_coordinator_id) break;
        g_recovery_phase = BOOT_RECOVERY_PHASE_COMMIT;
        g_commit_expected_bitmap = (uint8_t)(value & 0x00FFU);
        if ((g_commit_expected_bitmap & Boot_NodeBit(g_node_id)) == 0U) break;
        value = Boot_ArmCommitLocal() ? 1U : 0U;
        (void)Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_COMMIT_ACK,
                                 g_node_id, g_session_id, value);
        break;

    case BOOT_CMD_COMMIT_ACK:
        if ((g_is_coordinator != 0U) && (frame.target == g_node_id) &&
            (g_recovery_phase == BOOT_RECOVERY_PHASE_COMMIT) &&
            ((g_commit_expected_bitmap & source_bit) != 0U))
        {
            if (value == 0U) { Boot_FinalRecoveryFailure(); break; }
            g_commit_ack_bitmap |= source_bit;
        }
        break;

    case BOOT_CMD_COMMIT_EXECUTE:
        if ((source != g_coordinator_id) ||
            ((uint8_t)(value & 0x00FFU) != g_commit_expected_bitmap) ||
            ((g_commit_expected_bitmap & Boot_NodeBit(g_node_id)) == 0U)) break;
        if (g_commit_execute_received == 0U)
        {
            if (Boot_ExecuteCommitLocal() == 0U)
            {
                g_status = BOOT_STATUS_ERROR;
                g_last_error = BOOT_ERR_COMMIT;
            }
        }
        break;

    case BOOT_CMD_RECOVERY_READY:        if (source != g_coordinator_id) break;
        g_repair_round = (uint8_t)(value & 0x00FFU);
        if (Boot_MissingCount() == 0U) g_status = BOOT_STATUS_VERIFY;
        g_coord_terminal = 1U;
        break;

    case BOOT_CMD_RECOVERY_FAILED:
        if (source != g_coordinator_id) break;
        g_repair_round = (uint8_t)(value & 0x00FFU);
        g_recovery_phase = BOOT_RECOVERY_PHASE_FAILED;
        g_status = BOOT_STATUS_ERROR;
        g_last_error = BOOT_ERR_RECOVERY_FAILED;
        g_coord_terminal = 1U;
        break;

    default:
        break;
    }
}

/*
 * 消费一个完整的 64 字节逻辑固件 DATA 包。
 *
 * 此处 DATA 与传输无关：原生 CAN FD 以一个 64 字节帧到达；经典 CAN 已由
 * 适配层根据 0x100~0x107 ID 重组。因此核心在两种传输下看到完全相同的
 * 64 字节对象。
 *
 * 关键可靠性规则：只有 Flash 写入并立即回读校验成功后，才置位包 Bitmap。
 * 缺失/失败包保持 bit=0，稍后再修复；初始数据流不会因单个错误 Seq 停止。
 * 已置位的重复包会被忽略，避免不必要的 Flash 编程。
 */
/* 校验并处理一帧固件 DATA，包括窗口缓存和 Flash 提交。 */
static void Boot_ProcessData(const uint8_t *data, uint8_t len)
{
    uint8_t target;
    uint16_t seq;
    uint32_t offset;
    uint32_t valid_len;
    uint8_t ok;

    if ((data == NULL) || (len != BOOT_DATA_SIZE))
    {
        return;
    }

    target = data[0];
    if ((target != g_node_id) && (target != BOOT_BROADCAST_ID))
    {
        return;
    }

    if (data[1] != BOOT_DATA_CMD_WRITE)
    {
        return;
    }

    if (Boot_ReadU16LE(&data[4]) != ((g_session_active != 0U) ? g_session_id : 0U))
    {
        g_last_error = BOOT_ERR_SESSION;
        return;
    }
    if ((data[6] != 0U) || (data[7] != 0U))
    {
        g_last_error = BOOT_ERR_BAD_STATE;
        return;
    }

    if (Boot_IsGuardProtected() != 0U)
    {
        /* Guard 只在阶段 1 监听；它绝不会修改自身 APP/配置。 */
        return;
    }

    if (g_write_active == 0U)
    {
        g_last_error = BOOT_ERR_BAD_STATE;
        return;
    }

    seq = Boot_ReadU16LE(&data[2]);
    if (seq >= g_total_packets)
    {
        g_last_error = BOOT_ERR_SEQUENCE;
        return;
    }

    if (Boot_BitmapGet(seq) != 0U)
    {
        /* 重复包：本地已经回读校验通过，不再重复编程。 */
        return;
    }

    offset = (uint32_t)seq * BOOT_DATA_PAYLOAD_SIZE;
    if (offset >= g_write_size)
    {
        g_last_error = BOOT_ERR_SEQUENCE;
        return;
    }

    valid_len = g_write_size - offset;
    if (valid_len > BOOT_DATA_PAYLOAD_SIZE)
    {
        valid_len = BOOT_DATA_PAYLOAD_SIZE;
    }

    if (g_write_region == BOOT_WRITE_REGION_APP)
    {
        /*
         * Legacy单节点首次下载先进入双SRAM窗口，不再每56字节立即写Flash。
         * Repair和自治会话保留原来的随机Sequence单包写入路径。
         */
        if ((g_session_active == 0U) &&
            (g_status == BOOT_STATUS_WRITE))
        {
            (void)Boot_FlashWindowStage(seq, &data[8], valid_len);
            return;
        }

        ok = Boot_StorageProgramApp(offset, &data[8], valid_len);
    }
    else if (g_write_region == BOOT_WRITE_REGION_CONFIG)
    {
        ok = Boot_StorageConfigStageWrite(offset, &data[8], valid_len);
    }
    else
    {
        g_last_error = BOOT_ERR_PROTECTED_REGION;
        return;
    }

    if (ok == 0U)
    {
        /* 包处理失败不会中止传输。保持 Bitmap bit=0，继续接收后续包；
         * 主节点会在 WRITE_END 后修复该包。 */
        g_last_error = BOOT_ERR_FLASH_WRITE;
        return;
    }

    Boot_BitmapSet(seq);
    g_received_packets++;
    Boot_UpdateProgress();

    if (g_status == BOOT_STATUS_REPAIR)
    {
        /* 保持 REPAIR，直到主节点在本轮结束后再次发送 WRITE_END。 */
    }
    else
    {
        g_status = BOOT_STATUS_WRITE;
    }
}

/*
 * 异步 READ 生产任务。每次 Boot_Task() 最多发送一个 RESPONSE，携带最多 4
 * 个 Flash 字节。这样不会独占 TX 队列，并允许 CAN RX/恢复任务在读取块之间运行。
 *
 * 每个响应不编码偏移量；Host 根据原始 READ 请求中的地址/长度，从到达顺序
 * 开始重构数据。
 */
/* 每次主循环发送一段 Flash READ 结果，避免阻塞协议处理。 */
static void Boot_TaskRead(void)
{
    uint8_t data[4] = {0};
    uint16_t chunk;

    if ((g_read_active == 0U))
    {
        return;
    }

    chunk = (g_read_remaining > 4U) ? 4U : g_read_remaining;

    if (Boot_StorageReadFlash(g_read_address, data, chunk) == 0U)
    {
        g_read_active = 0U;
        Boot_SendError(BOOT_CMD_READ, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    if (Boot_SendResponse(BOOT_CMD_READ, BOOT_STATUS_READY, data) != 0U)
    {
        g_read_address += chunk;
        g_read_remaining = (uint16_t)(g_read_remaining - chunk);
        if (g_read_remaining == 0U)
        {
            g_read_active = 0U;
        }
    }
}

/*
 * 面向 Legacy Host 的缺包报告。
 * 首先准确发送一帧包含 {missing,total} 的 MISSING_COUNT；随后每个缺失
 * Sequence 发送一帧 MISSING_ITEM。每次 Boot_Task() 只尝试发送一帧，因此
 * 忙碌的传输层不会阻塞主循环。
 *
 * 自治广播模式不使用这组详细的 0x50x 流，而是使用 0x60x 上的
 * Boot_TaskPeerMissingReport()，避免重复占用总线流量。
 */
/* 分时发送主机缺包数量和缺包序号。 */
static void Boot_TaskMissingReport(void)
{
    uint8_t data[4] = {0};

    if ((g_missing_report_active == 0U))
    {
        return;
    }

    if (g_missing_count_sent == 0U)
    {
        Boot_WriteU16LE(&data[0], g_missing_count);
        Boot_WriteU16LE(&data[2], g_total_packets);

        if (Boot_SendResponse(BOOT_CMD_MISSING_COUNT,
                                 (g_missing_count == 0U) ? BOOT_STATUS_READY : BOOT_STATUS_REPAIR,
                                 data) != 0U)
        {
            g_missing_count_sent = 1U;
            if (g_missing_count == 0U)
            {
                g_missing_report_active = 0U;
            }
        }
        return;
    }

    while (g_missing_scan_seq < g_total_packets)
    {
        uint16_t seq = g_missing_scan_seq++;

        if (Boot_BitmapGet(seq) == 0U)
        {
            memset(data, 0, sizeof(data));
            Boot_WriteU16LE(&data[0], seq);
            Boot_WriteU16LE(&data[2], g_missing_item_index);

            if (Boot_SendResponse(BOOT_CMD_MISSING_ITEM,
                                     BOOT_STATUS_REPAIR,
                                     data) != 0U)
            {
                g_missing_item_index++;
                if (g_missing_item_index >= g_missing_count)
                {
                    g_missing_report_active = 0U;
                }
            }
            else
            {
                /* 下次 Task() 调用时重试此 Sequence。 */
                g_missing_scan_seq--;
            }
            return;
        }
    }

    g_missing_report_active = 0U;
}

/*
 * Legacy PROVIDER_GRANT 与自治修复共用的 Provider DATA 生产任务。
 * 每次任务迭代暂存一个逻辑 DATA 包。最后一个包被传输层接受后不会立即
 * 宣布完成：accepted 只表示已入队/暂存，尤其在经典 CAN 模式下，一个逻辑
 * 64 字节 DATA 仍要排空 8 个物理 8 字节分片。
 *
 * 因此 g_provider_done_pending 会强制在发送 READY 或 PROVIDER_DONE 前调用
 * Boot_FlushTx()。没有这个屏障，Coordinator 可能在最后的 0x100~0x107
 * 分片到达总线前就开始新一轮缺包扫描。
 */
/* 分时执行 Provider 补包发送任务。 */
static void Boot_TaskProvider(void)
{
    uint8_t fd_frame[BOOT_DATA_SIZE];
    uint8_t data[4] = {0};

    if (g_provider_done_pending != 0U)
    {
        /* Boot_SendData() 表示已接受/暂存，不一定已经物理发送。
         * DONE 前必须排空原生 FD TX FIFO 或经典 CAN 0x100~0x107 分片，
         * 否则 Coordinator 可能过早开始下一轮缺包扫描。 */
        Boot_FlushTx(20U);

        if (g_provider_peer_mode != 0U)
        {
            if (g_coordinator_id == g_node_id)
            {
                Boot_CoordinatorProviderDone(g_provider_peer_seq);
                g_provider_done_pending = 0U;
                g_provider_peer_mode = 0U;
            }
            else if (Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_PROVIDER_DONE, g_node_id, g_session_id, g_provider_peer_seq) != 0U)
            {
                g_provider_done_pending = 0U;
                g_provider_peer_mode = 0U;
            }
        }
        else
        {
            data[0] = g_provider_target;
            if (Boot_SendResponse(BOOT_CMD_PROVIDER_GRANT, BOOT_STATUS_READY, data) != 0U) g_provider_done_pending = 0U;
        }
        return;
    }

    if (g_provider_active == 0U) return;
    if (Boot_BuildProviderFrame(g_provider_next_seq, g_provider_target, fd_frame) == 0U)
    {
        g_provider_active = 0U;
        Boot_SetError(BOOT_ERR_PROVIDER_SOURCE, 0U);
        if (g_provider_peer_mode != 0U)
        {
            if (g_coordinator_id == g_node_id)
            {
                g_coord_wait_provider = 0U;
                Boot_CoordinatorPhaseFailure();
            }
            else
            {
                (void)Boot_SendPeerFrame(g_coordinator_id, BOOT_CMD_PROVIDER_DONE, g_node_id, g_session_id, g_provider_peer_seq);
            }
            g_provider_peer_mode = 0U;
        }
        else
        {
            data[0] = (uint8_t)BOOT_ERR_PROVIDER_SOURCE;
            (void)Boot_SendResponse(BOOT_CMD_PROVIDER_GRANT, BOOT_STATUS_ERROR, data);
        }
        return;
    }

    if (Boot_SendData(fd_frame, BOOT_DATA_SIZE) == 0U) return;
    g_provider_next_seq++;
    g_provider_remaining--;
    if (g_provider_remaining == 0U)
    {
        g_provider_active = 0U;
        g_provider_done_pending = 1U;
    }
}

/* 分时发送节点间缺包报告。 */
static void Boot_TaskPeerMissingReport(void)
{
    uint16_t missing;
    if (g_peer_report_tx_active == 0U) return;
    missing = Boot_MissingCount();
    if (g_peer_report_tx_count_sent == 0U)
    {
        if (Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_MISSING_COUNT, g_node_id, g_session_id, missing) != 0U)
        {
            g_peer_report_tx_count_sent = 1U;
            if (missing == 0U) g_peer_report_tx_active = 0U;
        }
        return;
    }
    if ((int32_t)(HAL_GetTick() - g_peer_report_items_after_ms) < 0) return;

    while (g_peer_report_tx_scan_seq < g_total_packets)
    {
        uint16_t seq = g_peer_report_tx_scan_seq++;
        if (Boot_BitmapGet(seq) == 0U)
        {
            if (Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_MISSING_ITEM, g_node_id, g_session_id, seq) != 0U)
            {
                g_peer_report_tx_item_count++;
                if (g_peer_report_tx_item_count >= missing) g_peer_report_tx_active = 0U;
            }
            else g_peer_report_tx_scan_seq--;
            return;
        }
    }
    g_peer_report_tx_active = 0U;
}

/* 执行确定性协调者竞选时隙。 */
static void Boot_TaskElection(void)
{
    uint32_t wait_ms;
    uint16_t claim_info;

    if (g_election_pending == 0U) return;
    if (g_coordinator_id != 0U)
    {
        g_election_pending = 0U;
        return;
    }

    /* 确定性竞选时隙：Node1 首先竞选，然后是 Node2 等。
     * 如果较小 ID 节点在发现阶段报告在线、但在竞选前掉线，
     * 后续仍在线的节点会在自己的时隙自动竞选。 */
    wait_ms = BOOT_COORD_ELECTION_DELAY_MS +
              ((uint32_t)(g_node_id - 1U) * BOOT_COORD_CLAIM_SLOT_MS);
    if ((HAL_GetTick() - g_election_started_ms) < wait_ms) return;

    claim_info = (uint16_t)g_peer_active_bitmap |
                 ((uint16_t)g_guard_node_id << 8U);
    if (Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_COORDINATOR_CLAIM,
                           g_node_id, g_session_id, claim_info) != 0U)
    {
        g_coordinator_id = g_node_id;
        g_is_coordinator = 1U;
        g_primary_members_bitmap = g_peer_active_bitmap;
        g_recovery_members_bitmap = g_primary_members_bitmap;
        g_coord_scan_seq = 0U;
        g_phase_started_ms = HAL_GetTick();
        g_election_pending = 0U;
    }
}

/* 请求目标节点重新发送其缺包报告。 */
static void Boot_RequestRepairReports(uint8_t target)
{
    Boot_ClearRemoteReportsForNextRound();
    g_provider_failed_bitmap = 0U;
    g_phase_started_ms = HAL_GetTick();
    (void)Boot_SendPeerFrame(target, BOOT_CMD_REPAIR_ROUND_END,
                             g_node_id, g_session_id, g_repair_round);
}

/* 处理 Provider 完成通知并推进协调者扫描状态。 */
static void Boot_CoordinatorProviderDone(uint16_t value)
{
    g_coord_wait_provider = 0U;
    g_coord_provider_id = 0U;
    g_provider_wait_started_ms = 0U;
    if (value != 0xFFFFU)
    {
        g_coord_scan_seq = (uint16_t)(value + 1U);
        return;
    }

    g_coord_scan_seq = 0U;
    g_coord_sweep_had_missing = 0U;
    g_repair_round = 0U;

    if (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_UPDATE)
    {
        g_recovery_phase = BOOT_RECOVERY_PHASE_GUARD_REPAIR;
        g_recovery_members_bitmap = Boot_NodeBit(g_guard_node_id);
        Boot_RequestRepairReports(g_guard_node_id);
    }
    else if (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_PREP)
    {
        g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_REPAIR;
        g_recovery_members_bitmap = g_primary_members_bitmap;
        Boot_RequestRepairReports(BOOT_BROADCAST_ID);
    }
    else
    {
        Boot_CoordinatorPhaseFailure();
    }
}

/* 驱动协调者对缺包、Provider 和修复轮次的状态机。 */
static void Boot_TaskCoordinator(void)
{
    uint8_t provider;
    uint8_t report_target;

    if ((g_is_coordinator == 0U) || (g_coord_terminal != 0U)) return;
    if (!((g_recovery_phase == BOOT_RECOVERY_PHASE_NEW_REPAIR) ||
          (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_REPAIR) ||
          (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_REPAIR))) return;
    if (Boot_AllPeerReportsComplete() == 0U)
    {
        if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
            Boot_CoordinatorPhaseFailure();
        return;
    }
    if (g_coord_wait_provider != 0U)
    {
        if ((HAL_GetTick() - g_provider_wait_started_ms) >= BOOT_PROVIDER_TIMEOUT_MS)
        {
            if ((g_coord_provider_id >= 1U) && (g_coord_provider_id <= BOOT_MAX_NODE_NUM))
                g_provider_failed_bitmap |= Boot_NodeBit(g_coord_provider_id);
            if (g_coord_provider_id == g_node_id)
            {
                g_provider_active = 0U;
                g_provider_done_pending = 0U;
                g_provider_peer_mode = 0U;
            }
            g_coord_wait_provider = 0U;
            g_coord_provider_id = 0U;
        }
        return;
    }

    while (g_coord_scan_seq < g_total_packets)
    {
        uint16_t seq = g_coord_scan_seq;
        if (Boot_AnyMemberMissing(seq) == 0U)
        {
            g_coord_scan_seq++;
            continue;
        }

        if (g_repair_round >= BOOT_MAX_REPAIR_ROUNDS)
        {
            Boot_CoordinatorPhaseFailure();
            return;
        }

        provider = Boot_SelectProvider(seq);
        if (provider == 0U)
        {
            g_repair_round++;
            if (g_repair_round >= BOOT_MAX_REPAIR_ROUNDS)
            {
                Boot_CoordinatorPhaseFailure();
                return;
            }
            g_coord_scan_seq = 0U;
            g_coord_sweep_had_missing = 0U;
            report_target = (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_REPAIR) ?
                            g_guard_node_id : BOOT_BROADCAST_ID;
            Boot_RequestRepairReports(report_target);
            return;
        }

        g_coord_current_seq = seq;
        g_coord_provider_id = provider;
        g_coord_wait_provider = 1U;
        g_provider_wait_started_ms = HAL_GetTick();
        g_coord_sweep_had_missing = 1U;

        if (provider == g_node_id)
        {
            g_provider_target = BOOT_BROADCAST_ID;
            g_provider_next_seq = seq;
            g_provider_remaining = 1U;
            g_provider_peer_seq = seq;
            g_provider_peer_mode = 1U;
            g_provider_active = 1U;
            g_provider_done_pending = 0U;
            return;
        }

        if (Boot_SendPeerFrame(provider, BOOT_CMD_PROVIDER_ASSIGN,
                               g_node_id, g_session_id, seq) == 0U)
        {
            g_coord_wait_provider = 0U;
        }
        return;
    }

    if (g_coord_sweep_had_missing == 0U)
    {
        if (g_recovery_phase == BOOT_RECOVERY_PHASE_NEW_REPAIR)
        {
            if ((g_session_flags & BOOT_SESSION_FLAG_COORD_COMMIT) != 0U)
            {
                if (g_session_crc_valid == 0U) { Boot_CoordinatorPhaseFailure(); return; }
                g_recovery_members_bitmap = g_primary_members_bitmap;
                Boot_StartVerifyContext((uint8_t)BOOT_RECOVERY_PHASE_NEW_VERIFY);
            }
            else
            {
                g_coord_terminal = 1U;
                g_status = BOOT_STATUS_VERIFY;
                (void)Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_RECOVERY_READY,
                                         g_node_id, g_session_id, g_repair_round);
            }
        }
        else if (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_REPAIR)
        {
            Boot_StartVerifyContext((uint8_t)BOOT_RECOVERY_PHASE_GUARD_UPDATE);
        }
        else
        {
            Boot_StartVerifyContext((uint8_t)BOOT_RECOVERY_PHASE_ROLLBACK_VERIFY);
        }
        return;
    }

    g_repair_round++;
    g_coord_scan_seq = 0U;
    g_coord_sweep_had_missing = 0U;
    report_target = (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_REPAIR) ?
                    g_guard_node_id : BOOT_BROADCAST_ID;
    Boot_RequestRepairReports(report_target);
}

/* 推进分布式镜像校验阶段并检测成员超时。 */
static void Boot_TaskVerifyPhase(void)
{
    uint8_t expected;
    if ((g_is_coordinator == 0U) || (g_verify_context == 0U) ||
        (g_coord_terminal != 0U)) return;

    expected = g_recovery_members_bitmap;
    if ((g_verify_response_bitmap & expected) == expected)
    {
        uint8_t context = g_verify_context;
        g_verify_context = 0U;
        if ((g_verify_ok_bitmap & expected) != expected)
        {
            Boot_CoordinatorPhaseFailure();
            return;
        }

        if (context == (uint8_t)BOOT_RECOVERY_PHASE_NEW_VERIFY)
        {
            Boot_StartGuardUpdate();
        }
        else
        {
            Boot_StartCommit();
        }
        return;
    }

    if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
    {
        Boot_CoordinatorPhaseFailure();
    }
}

/* 推进 Guard 元数据广播与确认阶段。 */
static void Boot_TaskGuardMetadata(void)
{
    uint8_t cmd = 0U;
    uint16_t value = 0U;

    if (g_guard_meta_tx_stage == 0U) return;
    switch (g_guard_meta_tx_stage)
    {
    case 1U: cmd = BOOT_CMD_ROLLBACK_SIZE_LO; value = (uint16_t)(g_rollback_size & 0xFFFFU); break;
    case 2U: cmd = BOOT_CMD_ROLLBACK_SIZE_HI; value = (uint16_t)(g_rollback_size >> 16U); break;
    case 3U: cmd = BOOT_CMD_ROLLBACK_CRC_LO; value = (uint16_t)(g_rollback_crc32 & 0xFFFFU); break;
    case 4U: cmd = BOOT_CMD_ROLLBACK_CRC_HI; value = (uint16_t)(g_rollback_crc32 >> 16U); break;
    default: g_guard_meta_tx_stage = 0U; return;
    }

    if (Boot_SendPeerFrame(BOOT_BROADCAST_ID, cmd, g_node_id, g_session_id, value) != 0U)
    {
        g_guard_meta_tx_stage++;
        if (g_guard_meta_tx_stage > 4U) g_guard_meta_tx_stage = 0U;
    }
}

/* 推进回滚元数据、准备和修复状态机。 */
static void Boot_TaskRollbackState(void)
{
    uint8_t self_bit;
    if (g_is_coordinator == 0U) return;

    if (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_META)
    {
        if (g_rollback_meta_mask == 0x0FU)
        {
            if ((g_rollback_size == 0U) || (g_rollback_size > BOOT_APP_MAX_SIZE))
            {
                Boot_FinalRecoveryFailure();
                return;
            }
            g_recovery_phase = BOOT_RECOVERY_PHASE_ROLLBACK_PREP;
            g_phase_started_ms = HAL_GetTick();
            g_recovery_members_bitmap = g_primary_members_bitmap;
            g_rollback_prepared_bitmap = 0U;
            self_bit = Boot_NodeBit(g_node_id);
            if ((g_primary_members_bitmap & self_bit) != 0U)
            {
                if (Boot_PrepareAppWriteInternal(g_rollback_size) == 0U)
                {
                    Boot_FinalRecoveryFailure();
                    return;
                }
                g_rollback_prepared_bitmap |= self_bit;
            }
            (void)Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_ROLLBACK_BEGIN,
                                     g_node_id, g_session_id, 0U);
        }
        else if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
        {
            Boot_FinalRecoveryFailure();
        }
        return;
    }
    if (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_PREP)
    {
        if ((g_rollback_prepared_bitmap & g_primary_members_bitmap) == g_primary_members_bitmap)
        {
            if (g_coord_wait_provider == 0U)
            {
                g_coord_provider_id = g_guard_node_id;
                g_coord_current_seq = 0xFFFFU;
                if (Boot_SendPeerFrame(g_guard_node_id, BOOT_CMD_FULL_STREAM,
                                       g_node_id, g_session_id, BOOT_BROADCAST_ID) != 0U)
                {
                    g_coord_wait_provider = 1U;
                    g_provider_wait_started_ms = HAL_GetTick();
                    g_phase_started_ms = HAL_GetTick();
                }
            }
        }
        else if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
        {
            Boot_FinalRecoveryFailure();
        }
    }
}

/* 监视 Provider 阶段，超时后触发失败处理。 */
static void Boot_TaskProviderPhaseWatchdog(void)
{
    if (g_is_coordinator == 0U) return;
    if (g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_UPDATE)
    {
        if ((g_coord_wait_provider == 0U) && (g_coord_provider_id == 0U))
        {
            if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
                Boot_CoordinatorPhaseFailure();
            return;
        }
        if ((g_coord_wait_provider == 0U) && (g_coord_provider_id != g_node_id))
        {
            if (Boot_SendPeerFrame(g_coord_provider_id, BOOT_CMD_FULL_STREAM, g_node_id,
                                   g_session_id, g_guard_node_id) != 0U)
            {
                g_coord_wait_provider = 1U;
                g_provider_wait_started_ms = HAL_GetTick();
            }
            else if ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS)
                Boot_CoordinatorPhaseFailure();
            return;
        }
    }
    if (g_coord_wait_provider == 0U) return;
    if (!((g_recovery_phase == BOOT_RECOVERY_PHASE_GUARD_UPDATE) ||
          (g_recovery_phase == BOOT_RECOVERY_PHASE_ROLLBACK_PREP))) return;
    if ((HAL_GetTick() - g_provider_wait_started_ms) < BOOT_PROVIDER_TIMEOUT_MS) return;
    if (g_coord_provider_id == g_node_id)
    {
        g_provider_active = 0U;
        g_provider_done_pending = 0U;
        g_provider_peer_mode = 0U;
    }
    g_coord_wait_provider = 0U;
    g_coord_provider_id = 0U;
    Boot_CoordinatorPhaseFailure();
}

/* 推进分布式提交广播和延迟执行。 */
static void Boot_TaskCommit(void)
{
    if (g_recovery_phase != BOOT_RECOVERY_PHASE_COMMIT) return;

    if ((g_is_coordinator != 0U) && (g_commit_tx_remaining == 0U) &&
        (g_commit_execute_received == 0U) &&
        ((g_commit_ack_bitmap & g_commit_expected_bitmap) == g_commit_expected_bitmap))
    {
        g_commit_tx_remaining = BOOT_COMMIT_REPEAT_COUNT;
    }

    if ((g_is_coordinator != 0U) && (g_commit_tx_remaining != 0U))
    {
        if (Boot_SendPeerFrame(BOOT_BROADCAST_ID, BOOT_CMD_COMMIT_EXECUTE,
                               g_node_id, g_session_id, g_commit_expected_bitmap) != 0U)
        {
            g_commit_tx_remaining--;
            if (g_commit_execute_received == 0U && Boot_ExecuteCommitLocal() == 0U)
            {
                g_status = BOOT_STATUS_ERROR;
                g_last_error = BOOT_ERR_COMMIT;
                return;
            }
        }
    }

    if ((g_is_coordinator != 0U) && (g_commit_execute_received == 0U) &&
        ((HAL_GetTick() - g_phase_started_ms) >= BOOT_PEER_PHASE_TIMEOUT_MS))
    {
        Boot_FinalRecoveryFailure();
        return;
    }

    if ((g_commit_execute_received != 0U) &&
        ((g_is_coordinator == 0U) || (g_commit_tx_remaining == 0U)) &&
        ((int32_t)(HAL_GetTick() - g_commit_due_ms) >= 0))
    {
        Boot_FlushTx(20U);
        Boot_RuntimeJumpToApp();
    }
}

/* 统一调度所有非阻塞 Flash、报告和恢复任务。 */
static void Boot_AsyncTask(void)
{
    Boot_TaskFlashWindows();
    Boot_TaskMissingReport();
    Boot_TaskPeerMissingReport();
    Boot_TaskGuardMetadata();
    Boot_TaskElection();
    Boot_TaskCoordinator();
    Boot_TaskVerifyPhase();
    Boot_TaskRollbackState();
    Boot_TaskProviderPhaseWatchdog();
    Boot_TaskCommit();
    Boot_TaskRead();
    Boot_TaskProvider();
}

/* 组装并发送标准 8 字节主机响应帧。 */
static uint8_t Boot_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data)
{
    Boot_ControlResponse_t response;

    memset(&response, 0, sizeof(response));
    response.node = g_node_id;
    response.cmd = cmd;
    response.status = status;

    if (data != NULL)
    {
        memcpy(response.data, data, sizeof(response.data));
    }

    response.crc = Boot_CRC8((const uint8_t *)&response, 7U);

    return Boot_SendControl((const uint8_t *)&response, BOOT_CONTROL_SIZE);
}

/* 返回当前节点号。 */
uint8_t Boot_GetNodeId(void)
{
    return g_node_id;
}

/* 返回当前 Bootloader 状态。 */
uint8_t Boot_GetStatus(void)
{
    return (uint8_t)g_status;
}

/* 返回最近一次错误码。 */
uint8_t Boot_GetLastError(void)
{
    return (uint8_t)g_last_error;
}

/* 返回当前升级进度百分比。 */
uint8_t Boot_GetProgress(void)
{
    return g_progress;
}


/* ========================================================================
 * 对外提供的、与传输无关的入口函数。
 * ======================================================================== */
/* 绑定传输回调并初始化 Bootloader 核心。 */
void Boot_Init(uint8_t default_node_id,
               Boot_SendCallback_t send_cb,
               Boot_FlushCallback_t flush_cb,
               void *transport_user)
{
    g_send_cb = send_cb;
    g_flush_cb = flush_cb;
    g_transport_user = transport_user;
    g_rx_write = 0U;
    g_rx_read = 0U;
    g_rx_overflow = 0U;
    Boot_CoreInit(default_node_id);
}

/* 在轻量路径中复制一条完整消息到接收环形队列。 */
uint8_t Boot_Input(const Boot_Message_t *message)
{
    uint16_t next;

    if (message == NULL)
    {
        return 0U;
    }

    if ((message->type != (uint8_t)BOOT_MESSAGE_CONTROL) &&
        (message->type != (uint8_t)BOOT_MESSAGE_DATA) &&
        (message->type != (uint8_t)BOOT_MESSAGE_PEER_CONTROL))
    {
        return 0U;
    }

    if ((message->len == 0U) || (message->len > BOOT_DATA_SIZE))
    {
        return 0U;
    }

    next = (uint16_t)((g_rx_write + 1U) % BOOT_RX_QUEUE_SIZE);
    if (next == g_rx_read)
    {
        g_rx_overflow++;
        g_last_error = BOOT_ERR_RX_OVERFLOW;
        return 0U;
    }

    g_rx_queue[g_rx_write] = *message;
    g_rx_write = next;
    return 1U;
}

/* 从接收环形队列取出一条待处理消息。 */
static uint8_t Boot_PopInput(Boot_Message_t *message)
{
    if ((message == NULL) || (g_rx_read == g_rx_write))
    {
        return 0U;
    }

    *message = g_rx_queue[g_rx_read];
    g_rx_read = (uint16_t)((g_rx_read + 1U) % BOOT_RX_QUEUE_SIZE);
    return 1U;
}

/* 主循环任务：处理接收队列并调度所有异步工作。 */
void Boot_Task(void)
{
    Boot_Message_t message;
    uint8_t budget = 16U;

    /* 耗时工作统一在这里执行，绝不在通信中断中执行。 */
    while ((budget-- > 0U) && (Boot_PopInput(&message) != 0U))
    {
        if ((message.type == (uint8_t)BOOT_MESSAGE_CONTROL) &&
            (message.len == BOOT_CONTROL_SIZE))
        {
            Boot_ProcessControl(message.data, (uint8_t)message.len);
        }
        else if ((message.type == (uint8_t)BOOT_MESSAGE_DATA) &&
                 (message.len == BOOT_DATA_SIZE))
        {
            Boot_ProcessData(message.data, (uint8_t)message.len);
        }
        else if ((message.type == (uint8_t)BOOT_MESSAGE_PEER_CONTROL) &&
                 (message.len == BOOT_CONTROL_SIZE))
        {
            Boot_ProcessPeerControl(message.data, (uint8_t)message.len);
        }
    }

    Boot_AsyncTask();
}

/* 返回接收队列溢出累计次数。 */
uint32_t Boot_GetRxOverflowCount(void)
{
    return g_rx_overflow;
}

/* 返回当前升级会话号。 */
uint16_t Boot_GetSessionId(void)
{
    return g_session_id;
}

/* 返回当前恢复协调者节点号。 */
uint8_t Boot_GetCoordinatorId(void)
{
    return g_coordinator_id;
}

/* 返回当前节点是否为恢复协调者。 */
uint8_t Boot_IsCoordinator(void)
{
    return g_is_coordinator;
}

/* 返回当前自动修复轮次。 */
uint8_t Boot_GetRepairRound(void)
{
    return g_repair_round;
}

/* 返回当前分布式恢复阶段。 */
uint8_t Boot_GetRecoveryPhase(void)
{
    return (uint8_t)g_recovery_phase;
}
