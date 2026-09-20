/*
 * STM32G4 FDCAN 传输适配层：把与硬件有关的 CAN 帧映射为与传输无关的
 * Boot_Message_t。协议核心只调用本文件导出的回调，不直接依赖 HAL FDCAN。
 */
#include "bootloader.h"
#include "fdcan.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include "boot_port_can_stm32g4.h"


/* 主机下发控制帧使用的标准 CAN ID。 */
#define BOOT_CAN_CONTROL_RX_ID     0x000U
/* 固件数据帧的基础 ID；经典 CAN 模式下会扩展为 0x100~0x107。 */
#define BOOT_CAN_DATA_ID           0x100U
/* 节点给主机回复时，在基础 ID 上叠加节点号。 */
#define BOOT_CAN_RESPONSE_BASE_ID  0x500U
/* 节点间协调控制帧的基础 ID。 */
#define BOOT_CAN_PEER_BASE_ID      0x600U
/* FDCAN 硬件发送 FIFO 满载时需要保留的队列深度。 */
#define BOOT_CAN_TX_FIFO_DEPTH     3U

/** 将逻辑字节数转换为 STM32 HAL 使用的 DLC 编码。 */
static uint32_t CanBytesToDlc(uint16_t len)
{
    /* HAL 定义的 DLC 编码不是简单的字节数，因此逐项映射。 */
    switch (len)
    {
    case 0U:  return FDCAN_DLC_BYTES_0;
    case 1U:  return FDCAN_DLC_BYTES_1;
    case 2U:  return FDCAN_DLC_BYTES_2;
    case 3U:  return FDCAN_DLC_BYTES_3;
    case 4U:  return FDCAN_DLC_BYTES_4;
    case 5U:  return FDCAN_DLC_BYTES_5;
    case 6U:  return FDCAN_DLC_BYTES_6;
    case 7U:  return FDCAN_DLC_BYTES_7;
    case 8U:  return FDCAN_DLC_BYTES_8;
    case 12U: return FDCAN_DLC_BYTES_12;
    case 16U: return FDCAN_DLC_BYTES_16;
    case 20U: return FDCAN_DLC_BYTES_20;
    case 24U: return FDCAN_DLC_BYTES_24;
    case 32U: return FDCAN_DLC_BYTES_32;
    case 48U: return FDCAN_DLC_BYTES_48;
    case 64U: return FDCAN_DLC_BYTES_64;
    default: return FDCAN_DLC_BYTES_0;
    }
}


/* 经典 CAN 接收分片的临时重组缓冲区，一次容纳一个 64 字节逻辑包。 */
static uint8_t g_classic_data_buffer[64];

/* 经典 CAN 接收状态：1 表示正在等待后续分片。 */
static uint8_t g_classic_data_active = 0U;
/* 下一个允许接收的经典 CAN 分片序号，范围为 1~7。 */
static uint8_t g_classic_expected_fragment = 0U;

/* Classic-only TX fallback for Provider DATA. One logical 64-byte message is
 * staged here and drained by BootPort_CAN_Task() as IDs 0x100..0x107. */
/* 经典 CAN 发送分片的临时缓冲区，保存待拆分的完整 DATA 消息。 */
static uint8_t g_classic_tx_buffer[BOOT_DATA_SIZE];
/* 经典 CAN 发送是否有待发送的逻辑 DATA 包。 */
static uint8_t g_classic_tx_active = 0U;
/* 当前待发送的经典 CAN 分片序号，范围为 0~7。 */
static uint8_t g_classic_tx_fragment = 0U;




/**
 * @brief 将 Bootloader 的逻辑消息编码并放入 FDCAN 发送队列。
 *
 * 控制和节点间消息使用 8 字节经典 CAN；固件 DATA 消息优先使用 64
 * 字节 CAN FD，若外设处于 Classic-only 模式则暂存，交由
 * BootPort_CAN_Task() 逐帧发送。
 */
uint8_t BootPort_CAN_Send(const Boot_Message_t *message, void *user)
{
    /* 由调用方传入的 FDCAN 外设句柄，不在本适配层复制或持有。 */
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)user;
    /* 每次发送使用独立的 HAL 帧头，避免修改输入逻辑消息。 */
    FDCAN_TxHeaderTypeDef tx = {0};
    /* 当前逻辑消息对应的 HAL DLC 编码。 */
    uint32_t dlc;

    if ((message == NULL) || (hfdcan == NULL)) return 0U;
    dlc = CanBytesToDlc(message->len);
    if ((message->len != 0U) && (dlc == FDCAN_DLC_BYTES_0)) return 0U;

    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = dlc;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    if (message->type == (uint8_t)BOOT_MESSAGE_CONTROL)
    {
        if (message->len != BOOT_CONTROL_SIZE) return 0U;
        /* Response payload Byte0 is Node ID. */
        tx.Identifier = BOOT_CAN_RESPONSE_BASE_ID + message->data[0];
        tx.BitRateSwitch = FDCAN_BRS_OFF;
        tx.FDFormat = FDCAN_CLASSIC_CAN;
    }
    else if (message->type == (uint8_t)BOOT_MESSAGE_DATA)
    {
        if (message->len != BOOT_DATA_SIZE) return 0U;

        if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
        {
            if (g_classic_tx_active != 0U) return 0U;
            memcpy(g_classic_tx_buffer, message->data, BOOT_DATA_SIZE);
            g_classic_tx_fragment = 0U;
            g_classic_tx_active = 1U;
            return 1U;
        }

        tx.Identifier = BOOT_CAN_DATA_ID;
        tx.BitRateSwitch = FDCAN_BRS_ON;
        tx.FDFormat = FDCAN_FD_CAN;
    }
    else if (message->type == (uint8_t)BOOT_MESSAGE_PEER_CONTROL)
    {
        if (message->len != BOOT_CONTROL_SIZE) return 0U;
        tx.Identifier = BOOT_CAN_PEER_BASE_ID + Boot_GetNodeId();
        tx.BitRateSwitch = FDCAN_BRS_OFF;
        tx.FDFormat = FDCAN_CLASSIC_CAN;
    }
    else
    {
        return 0U;
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx,
                                           (uint8_t *)message->data) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief 发送一个待发的经典 CAN DATA 分片。
 * @param user FDCAN 句柄指针。
 *
 * 该函数是非阻塞的，每次最多向硬件 FIFO 放入一个 8 字节分片，避免在
 * 主循环中长时间占用 CPU。
 */
void BootPort_CAN_Task(void *user)
{
    /* 当前服务的 FDCAN 外设句柄。 */
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)user;
    /* 经典 CAN 单个 8 字节分片的硬件帧头。 */
    FDCAN_TxHeaderTypeDef tx = {0};

    if ((hfdcan == NULL) || (g_classic_tx_active == 0U)) return;
    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0U) return;

    tx.Identifier = BOOT_CAN_DATA_ID + g_classic_tx_fragment;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            hfdcan, &tx, &g_classic_tx_buffer[(uint32_t)g_classic_tx_fragment * 8U]) == HAL_OK)
    {
        g_classic_tx_fragment++;
        if (g_classic_tx_fragment >= 8U)
        {
            g_classic_tx_fragment = 0U;
            g_classic_tx_active = 0U;
        }
    }
}

/**
 * @brief 在跳转或复位前等待底层发送工作完成。
 * @param user       FDCAN 句柄指针。
 * @param timeout_ms 最大等待时间，防止异常总线状态导致永久阻塞。
 */
void BootPort_CAN_Flush(void *user, uint32_t timeout_ms)
{
    /* 当前服务的 FDCAN 外设句柄。 */
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)user;
    /* 记录等待起点，用于限制总等待时长。 */
    uint32_t start = HAL_GetTick();
    if (hfdcan == NULL) return;

    while ((HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) < BOOT_CAN_TX_FIFO_DEPTH) ||
           (g_classic_tx_active != 0U))
    {
        BootPort_CAN_Task(hfdcan);
        if ((HAL_GetTick() - start) >= timeout_ms) break;
    }
}





/**
 * @brief 按严格序号重组一个经典 CAN 固件 DATA 逻辑包。
 * @param can_id 当前分片的标准 CAN ID（0x100~0x107）。
 * @param data   当前分片的 8 字节数据。
 *
 * 任意分片丢失或乱序都会丢弃当前重组结果，等待下一个分片 0 开始新的
 * 逻辑包；上层通过序号 Bitmap 发现丢包并安排补发。
 */
static void BootPort_CAN_ProcessClassicDataFragment(
    uint32_t can_id,
    const uint8_t data[8])
{
    /* 重组完成后投递给协议核心的逻辑消息对象。 */
    Boot_Message_t message;

    /* 当前分片在 0x100~0x107 序列中的序号。 */
    uint8_t fragment;

    if ((can_id < 0x100U) || (can_id > 0x107U))
    {
        return;
    }

    fragment = (uint8_t)(can_id - 0x100U);

    /*
     * Fragment 0 represents the start of one new
     * 64-byte logical DATA packet.
     */
    if (fragment == 0U)
    {
        g_classic_data_active = 1U;
        g_classic_expected_fragment = 1U;

        memcpy(
            &g_classic_data_buffer[0],
            data,
            8U);

        return;
    }

    /*
     * Require strict order:
     *
     * 0x100
     * 0x101
     * ...
     * 0x107
     *
     * If one Classic CAN frame is lost, discard
     * this complete logical packet.
     *
     * The upper Sequence Bitmap will later
     * identify the firmware packet as missing.
     */
    if ((g_classic_data_active == 0U) ||
        (fragment != g_classic_expected_fragment))
    {
        g_classic_data_active = 0U;
        g_classic_expected_fragment = 0U;

        return;
    }

    memcpy(
        &g_classic_data_buffer[(uint32_t)fragment * 8U],
        data,
        8U);

    if (fragment < 7U)
    {
        g_classic_expected_fragment++;

        return;
    }

    /* 8 Classic CAN frames -> one logical DATA message */
    message.type = BOOT_MESSAGE_DATA;
    message.len  = 64U;

    memcpy(
        message.data,
        g_classic_data_buffer,
        64U);

    g_classic_data_active = 0U;
    g_classic_expected_fragment = 0U;

    Boot_Input(&message);
}








/**
 * @brief 将 FIFO0 中的硬件帧转换为逻辑消息并投递给 Boot_Input()。
 *
 * 回调运行在中断上下文，只做取帧、校验 ID/格式和入队，不执行 Flash
 * 擦写、CRC 或其它耗时协议处理；这些工作统一由 Boot_Task() 完成。
 */
void BootPort_CAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    /* HAL 填充的 FDCAN 接收帧头，包含 ID、格式和数据长度。 */
    FDCAN_RxHeaderTypeDef rx_header;
    /* HAL 接收缓冲区，CAN FD 最大承载 64 字节。 */
    uint8_t data[64];
    /* 经过硬件帧校验后送入协议队列的统一消息对象。 */
    Boot_Message_t message;
 

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(
               hfdcan,
               FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &rx_header,
                data) != HAL_OK)
        {
            return;
        }

        /* =========================
         * CONTROL
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.Identifier == 0x000U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8))
        {
            message.type = BOOT_MESSAGE_CONTROL;
            message.len  = 8U;

            memcpy(message.data, data, 8U);

            Boot_Input(&message);

            continue;
        }

        /* =========================
         * PEER CONTROL: Node -> Nodes, ID 0x601..0x608.
         * Byte2 always carries the source Node ID and is checked here.
         * ========================= */
        if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.Identifier >= (BOOT_CAN_PEER_BASE_ID + 1U)) &&
            (rx_header.Identifier <= (BOOT_CAN_PEER_BASE_ID + BOOT_MAX_NODE_NUM)) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8))
        {
            /* ID 中的偏移量就是发送端声明的源节点号。 */
            uint8_t source = (uint8_t)(rx_header.Identifier - BOOT_CAN_PEER_BASE_ID);
            if (data[2] == source)
            {
                message.type = BOOT_MESSAGE_PEER_CONTROL;
                message.len = 8U;
                memcpy(message.data, data, 8U);
                Boot_Input(&message);
            }
            continue;
        }

        /* =========================
         * DATA - native CAN FD
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_FD_CAN) &&
            (rx_header.Identifier == 0x100U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_64))
        {
            message.type = BOOT_MESSAGE_DATA;
            message.len  = 64U;

            memcpy(message.data, data, 64U);

            Boot_Input(&message);

            continue;
        }

        /* =========================
         * DATA - Classic CAN fallback
         * ========================= */

        if ((rx_header.FDFormat == FDCAN_CLASSIC_CAN) &&
            (rx_header.Identifier >= 0x100U) &&
            (rx_header.Identifier <= 0x107U) &&
            (rx_header.DataLength == FDCAN_DLC_BYTES_8))
        {
            BootPort_CAN_ProcessClassicDataFragment(
                rx_header.Identifier,
                data);

            continue;
        }
    }
}
/**
 * @brief 安装 Bootloader 使用的 FDCAN 标准帧过滤规则。
 *
 * 过滤器将控制帧、CAN FD DATA 帧、节点间控制帧以及经典 CAN 分片导入
 * FIFO0，实际帧格式和长度仍在接收回调中再次校验。
 */
void BootPort_CAN_Filter_Init(void) {
  /* 复用同一个过滤器对象，依次安装不同 ID 区间的规则。 */
  FDCAN_FilterTypeDef filter = {0};

  /* 0x000：控制面 */
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x7FF;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Error_Handler();
  }

  /* 0x100：数据面 */
  filter.FilterIndex = 1;
  filter.FilterID1 = 0x100;
  filter.FilterID2 = 0x7F8;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Error_Handler();
  }

  /* 0x601~0x608 peer-control. Mask admits 0x600~0x60F; adapter checks exact range. */
  filter.FilterIndex = 2;
  filter.FilterID1 = 0x600;
  filter.FilterID2 = 0x7F0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE) != HAL_OK)
{
    Error_Handler();
}
}
