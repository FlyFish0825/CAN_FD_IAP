#include "boot_can_protocol.h"
#include "boot_can_config.h"
#include "boot_can_port.h"
#include "boot_crc.h"
#include "boot_runtime.h"
#include "boot_storage.h"
#include "stm32g4xx_hal.h"

#include <string.h>

static uint8_t g_node_id = BOOT_DEFAULT_NODE_ID;
static Boot_Status_t g_status = BOOT_STATUS_IDLE;
static Boot_Error_t g_last_error = BOOT_ERR_NONE;
static uint8_t g_progress = 0U;
static uint8_t g_boot_requested = 0U;

static Boot_PersistConfig_t g_config;
static uint8_t g_config_valid = 0U;
static uint8_t g_app_valid = 0U;

static uint8_t g_guard_active = 0U;
static uint8_t g_guard_node_id = 0U;
static uint8_t g_is_guard = 0U;

static uint8_t g_app_erased = 0U;
static uint8_t g_write_active = 0U;
static uint8_t g_write_region = BOOT_WRITE_REGION_APP;
static uint32_t g_write_size = 0U;
static uint16_t g_total_packets = 0U;
static uint16_t g_received_packets = 0U;
static uint8_t g_bitmap[BOOT_BITMAP_SIZE_BYTES];

static uint8_t g_read_active = 0U;
static uint32_t g_read_address = 0U;
static uint16_t g_read_remaining = 0U;

static uint8_t g_missing_report_active = 0U;
static uint8_t g_missing_count_sent = 0U;
static uint16_t g_missing_count = 0U;
static uint16_t g_missing_scan_seq = 0U;
static uint16_t g_missing_item_index = 0U;

static uint8_t g_provider_active = 0U;
static uint8_t g_provider_done_pending = 0U;
static uint8_t g_provider_target = 0U;
static uint16_t g_provider_next_seq = 0U;
static uint16_t g_provider_remaining = 0U;

static uint32_t BootCAN_ReadU32LE(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) |
           ((uint32_t)p[3] << 24U);
}

static uint16_t BootCAN_ReadU16LE(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8U));
}

static void BootCAN_WriteU16LE(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void BootCAN_WriteU32LE(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8U) & 0xFFU);
    p[2] = (uint8_t)((value >> 16U) & 0xFFU);
    p[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void BootCAN_SetError(Boot_Error_t error, uint8_t fatal)
{
    g_last_error = error;
    if (fatal != 0U)
    {
        g_status = BOOT_STATUS_ERROR;
    }
}

static void BootCAN_SendError(uint8_t cmd, Boot_Error_t error)
{
    uint8_t data[4] = {0};
    data[0] = (uint8_t)error;
    BootCAN_SetError(error, 1U);
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_ERROR, data);
}

static void BootCAN_BitmapClear(void)
{
    memset(g_bitmap, 0, sizeof(g_bitmap));
    g_received_packets = 0U;
}

static uint8_t BootCAN_BitmapGet(uint16_t seq)
{
    if ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)
    {
        return 0U;
    }

    return (uint8_t)((g_bitmap[seq >> 3U] >> (seq & 7U)) & 0x01U);
}

static void BootCAN_BitmapSet(uint16_t seq)
{
    if ((uint32_t)seq >= BOOT_MAX_PACKET_COUNT)
    {
        return;
    }

    g_bitmap[seq >> 3U] |= (uint8_t)(1U << (seq & 7U));
}

static uint16_t BootCAN_MissingCount(void)
{
    if (g_received_packets >= g_total_packets)
    {
        return 0U;
    }

    return (uint16_t)(g_total_packets - g_received_packets);
}

static void BootCAN_UpdateProgress(void)
{
    if (g_total_packets == 0U)
    {
        g_progress = 0U;
        return;
    }

    g_progress = (uint8_t)(((uint32_t)g_received_packets * 100UL) /
                           (uint32_t)g_total_packets);
}

static uint8_t BootCAN_IsGuardProtected(void)
{
    return ((g_guard_active != 0U) && (g_is_guard != 0U)) ? 1U : 0U;
}

static void BootCAN_CancelAsyncTasks(void)
{
    g_read_active = 0U;
    g_missing_report_active = 0U;
    g_missing_count_sent = 0U;
    g_provider_active = 0U;
    g_provider_done_pending = 0U;
}

static uint8_t BootCAN_ProviderPacketAvailable(uint16_t seq, uint32_t *source_size)
{
    uint32_t persisted_packets;

    if ((g_write_active != 0U) &&
        (g_write_region == BOOT_WRITE_REGION_APP) &&
        (seq < g_total_packets) &&
        (BootCAN_BitmapGet(seq) != 0U))
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
        persisted_packets = (g_config.app_size + BOOT_CAN_FD_PAYLOAD_SIZE - 1UL) /
                            BOOT_CAN_FD_PAYLOAD_SIZE;

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

static uint8_t BootCAN_BuildProviderFrame(uint16_t seq,
                                          uint8_t target,
                                          uint8_t frame[BOOT_CAN_FD_DLC])
{
    uint32_t source_size;
    uint32_t offset;
    uint32_t valid_len;

    if ((frame == NULL) || (BootCAN_ProviderPacketAvailable(seq, &source_size) == 0U))
    {
        return 0U;
    }

    offset = (uint32_t)seq * BOOT_CAN_FD_PAYLOAD_SIZE;
    if (offset >= source_size)
    {
        return 0U;
    }

    valid_len = source_size - offset;
    if (valid_len > BOOT_CAN_FD_PAYLOAD_SIZE)
    {
        valid_len = BOOT_CAN_FD_PAYLOAD_SIZE;
    }

    memset(frame, 0, BOOT_CAN_FD_DLC);
    memset(&frame[8], 0xFF, BOOT_CAN_FD_PAYLOAD_SIZE);

    frame[0] = target;
    frame[1] = BOOT_FD_CMD_WRITE_DATA;
    BootCAN_WriteU16LE(&frame[2], seq);
    /* Byte4..7 reserved for future protocol extensions. */

    memcpy(&frame[8],
           (const void *)(BOOT_APP_START_ADDR + offset),
           valid_len);

    return 1U;
}

static void BootCAN_StartMissingReport(void)
{
    g_missing_count = BootCAN_MissingCount();
    g_missing_report_active = 1U;
    g_missing_count_sent = 0U;
    g_missing_scan_seq = 0U;
    g_missing_item_index = 0U;
}

static void BootCAN_HandleGetVersion(uint8_t cmd)
{
    uint8_t data[4] = {
        BOOT_VERSION_MAJOR,
        BOOT_VERSION_MINOR,
        BOOT_VERSION_PATCH,
        BOOT_VERSION_BUILD
    };

    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, data);
}

static void BootCAN_HandleGetDeviceId(uint8_t cmd)
{
    uint8_t data[4];
    BootCAN_WriteU32LE(data, DBGMCU->IDCODE);
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, data);
}

static void BootCAN_HandleGetInfo(uint8_t cmd)
{
    uint8_t data[4] = {0};

    data[0] = g_node_id;
    data[1] = (g_config_valid != 0U) ? g_config.hardware_version : 0U;
    data[2] = g_app_valid;
    data[3] = g_config_valid;

    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, data);
}

static void BootCAN_HandleEnterBoot(uint8_t cmd)
{
    g_boot_requested = 1U;
    if (g_status == BOOT_STATUS_ERROR)
    {
        g_status = BOOT_STATUS_IDLE;
    }
    g_last_error = BOOT_ERR_NONE;
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, NULL);
}

static void BootCAN_HandleSetGuard(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t guard_id = frame->seq;

    if ((guard_id < 1U) || (guard_id > BOOT_MAX_NODE_NUM))
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    g_guard_node_id = guard_id;
    g_guard_active = 1U;
    g_is_guard = (g_node_id == guard_id) ? 1U : 0U;
    data[0] = guard_id;

    if (g_is_guard != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_GUARD, data);
    }
    else
    {
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
    }
}

static void BootCAN_HandleReleaseGuard(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t guard_id = frame->seq;

    if ((g_guard_active == 0U) || (guard_id != g_guard_node_id))
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
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

    (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

static void BootCAN_HandleErase(uint8_t cmd)
{
    if (BootCAN_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        g_last_error = BOOT_ERR_GUARD_PROTECTED;
        (void)BootCAN_SendResponse(cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    BootCAN_CancelAsyncTasks();
    BootStorage_ConfigStageAbort();
    g_write_active = 0U;
    g_app_erased = 0U;
    g_progress = 0U;
    g_status = BOOT_STATUS_ERASE;
    g_last_error = BOOT_ERR_NONE;

    /* Power-loss safety: invalidate metadata before touching APP Flash. */
    if (BootStorage_InvalidateApp(g_node_id) == 0U)
    {
        BootCAN_SendError(cmd, BOOT_ERR_CONFIG);
        return;
    }

    g_app_valid = 0U;
    if (BootStorage_LoadConfig(&g_config) != 0U)
    {
        g_config_valid = 1U;
    }

    if (BootStorage_EraseApp() == 0U)
    {
        BootCAN_SendError(cmd, BOOT_ERR_FLASH_ERASE);
        return;
    }

    BootCAN_BitmapClear();
    g_app_erased = 1U;
    g_progress = 100U;
    g_status = BOOT_STATUS_READY;
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, NULL);
}

static void BootCAN_HandleWrite(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t region = frame->seq;
    uint32_t size = BootCAN_ReadU32LE(frame->param);
    uint32_t packets;

    if (BootCAN_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        g_last_error = BOOT_ERR_GUARD_PROTECTED;
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if (region == BOOT_WRITE_REGION_BOOTLOADER)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_PROTECTED_REGION);
        return;
    }

    if (region == BOOT_WRITE_REGION_APP)
    {
        if (g_app_erased == 0U)
        {
            BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
            return;
        }

        if ((size == 0U) || (size > BOOT_APP_MAX_SIZE))
        {
            BootCAN_SendError(frame->cmd, BOOT_ERR_SIZE);
            return;
        }
    }
    else if (region == BOOT_WRITE_REGION_CONFIG)
    {
        if ((size == 0U) || (size > BOOT_CONFIG_PAGE_SIZE))
        {
            BootCAN_SendError(frame->cmd, BOOT_ERR_SIZE);
            return;
        }

        if (BootStorage_ConfigStageBegin() == 0U)
        {
            BootCAN_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }
    }
    else
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    packets = (size + BOOT_CAN_FD_PAYLOAD_SIZE - 1UL) / BOOT_CAN_FD_PAYLOAD_SIZE;
    if ((packets == 0U) || (packets > BOOT_MAX_PACKET_COUNT))
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_SIZE);
        return;
    }

    BootCAN_CancelAsyncTasks();
    BootCAN_BitmapClear();

    g_write_active = 1U;
    g_write_region = region;
    g_write_size = size;
    g_total_packets = (uint16_t)packets;
    g_status = BOOT_STATUS_WRITE;
    g_last_error = BOOT_ERR_NONE;
    g_progress = 0U;

    data[0] = region;
    BootCAN_WriteU16LE(&data[1], g_total_packets);
    (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_WRITE, data);
}

static void BootCAN_HandleRead(const Boot_CAN_Frame_t *frame)
{
    uint32_t address;
    uint16_t length = frame->seq;

    if (length == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_LENGTH);
        return;
    }

    if (g_read_active != 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BUSY);
        return;
    }

    address = BootCAN_ReadU32LE(frame->param);
    if (BootStorage_IsFlashRangeValid(address, length) == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    g_read_address = address;
    g_read_remaining = length;
    g_read_active = 1U;
}

static void BootCAN_HandleVerify(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4];
    uint32_t expected_crc;
    uint32_t actual_crc;

    if (BootCAN_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if ((g_write_active == 0U) || (g_write_region != BOOT_WRITE_REGION_APP))
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    if (BootCAN_MissingCount() != 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    expected_crc = BootCAN_ReadU32LE(frame->param);
    g_status = BOOT_STATUS_VERIFY;

    actual_crc = BootCRC32_Calculate((const uint8_t *)BOOT_APP_START_ADDR,
                                     g_write_size);

    if (actual_crc != expected_crc)
    {
        BootCAN_WriteU32LE(data, actual_crc);
        BootCAN_SetError(BOOT_ERR_CRC_MISMATCH, 1U);
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_ERROR, data);
        return;
    }

    if (BootStorage_SaveAppMetadata(g_write_size, actual_crc, 1U) == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_CONFIG);
        return;
    }

    if (BootStorage_LoadConfig(&g_config) == 0U)
    {
        g_config_valid = 0U;
        BootCAN_SendError(frame->cmd, BOOT_ERR_CONFIG);
        return;
    }

    g_config_valid = 1U;
    g_app_valid = 1U;
    g_status = BOOT_STATUS_READY;
    g_last_error = BOOT_ERR_NONE;
    g_progress = 100U;

    BootCAN_WriteU32LE(data, actual_crc);
    (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
}

static void BootCAN_HandleWriteEnd(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4] = {0};
    uint16_t missing;

    if (BootCAN_IsGuardProtected() != 0U)
    {
        g_status = BOOT_STATUS_GUARD;
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_GUARD, NULL);
        return;
    }

    if (g_write_active == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_STATE);
        return;
    }

    missing = BootCAN_MissingCount();
    BootCAN_WriteU16LE(data, missing);

    if (missing != 0U)
    {
        g_status = BOOT_STATUS_REPAIR;
        BootCAN_StartMissingReport();
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_REPAIR, data);
        return;
    }

    if (g_write_region == BOOT_WRITE_REGION_CONFIG)
    {
        if (BootStorage_ConfigStageCommit() == 0U)
        {
            BootCAN_SendError(frame->cmd, BOOT_ERR_CONFIG);
            return;
        }

        g_config_valid = BootStorage_LoadConfig(&g_config);
        if (g_config_valid != 0U)
        {
            g_app_valid = BootRuntime_ValidatePersistedApp(&g_config);
        }
        else
        {
            g_app_valid = 0U;
        }
        g_write_active = 0U;
        g_status = BOOT_STATUS_READY;
        g_progress = 100U;
        BootCAN_StartMissingReport(); /* Sends MISSING_COUNT = 0. */
        (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_READY, data);
        return;
    }

    g_status = BOOT_STATUS_VERIFY;
    g_progress = 100U;
    BootCAN_StartMissingReport(); /* Sends MISSING_COUNT = 0. */
    (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_VERIFY, data);
}

static void BootCAN_HandleProviderGrant(const Boot_CAN_Frame_t *frame)
{
    uint8_t data[4] = {0};
    uint8_t data_target;
    uint16_t start_seq;
    uint16_t count;
    uint32_t end_seq;

    /* Provider selection MUST be unicast. A broadcast grant could make several
     * healthy nodes transmit simultaneously, which this protocol forbids.
     */
    if (frame->target == BOOT_BROADCAST_ID)
    {
        return;
    }

    data_target = frame->seq;
    start_seq = BootCAN_ReadU16LE(&frame->param[0]);
    count = BootCAN_ReadU16LE(&frame->param[2]);

    if (!(((data_target >= 1U) && (data_target <= BOOT_MAX_NODE_NUM)) ||
          (data_target == BOOT_BROADCAST_ID)))
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    if (count == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_BAD_LENGTH);
        return;
    }

    end_seq = (uint32_t)start_seq + (uint32_t)count;
    if (end_seq > 65536UL)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_SEQUENCE);
        return;
    }

    if (BootCAN_ProviderPacketAvailable(start_seq, NULL) == 0U)
    {
        BootCAN_SendError(frame->cmd, BOOT_ERR_PROVIDER_SOURCE);
        return;
    }

    g_provider_target = data_target;
    g_provider_next_seq = start_seq;
    g_provider_remaining = count;
    g_provider_active = 1U;
    g_provider_done_pending = 0U;

    data[0] = data_target;
    BootCAN_WriteU16LE(&data[1], start_seq);
    (void)BootCAN_SendResponse(frame->cmd, BOOT_STATUS_WRITE, data);
}

static void BootCAN_HandleJumpApp(uint8_t cmd)
{
    if ((g_app_valid == 0U) || (BootRuntime_ValidatePersistedApp(&g_config) == 0U))
    {
        g_app_valid = 0U;
        BootCAN_SendError(cmd, BOOT_ERR_APP_INVALID);
        return;
    }

    g_config_valid = 1U;
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, NULL);
    (void)BootCAN_HW_WaitTxIdle(20U);
    BootRuntime_JumpToApp();
}

static void BootCAN_HandleReset(uint8_t cmd)
{
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_READY, NULL);
    (void)BootCAN_HW_WaitTxIdle(20U);
    HAL_Delay(1U);
    NVIC_SystemReset();
}

static void BootCAN_HandleGetStatus(uint8_t cmd)
{
    uint8_t data[4] = {0};

    data[0] = (uint8_t)g_status;
    data[1] = (uint8_t)g_last_error;
    data[2] = g_progress;
    data[3] = 0U; /* reserved */

    (void)BootCAN_SendResponse(cmd, (uint8_t)g_status, data);
}

static void BootCAN_HandleAbort(uint8_t cmd)
{
    BootCAN_CancelAsyncTasks();
    BootStorage_ConfigStageAbort();
    g_write_active = 0U;
    g_status = BOOT_STATUS_ERROR;
    g_last_error = BOOT_ERR_ABORTED;
    (void)BootCAN_SendResponse(cmd, BOOT_STATUS_ERROR, NULL);
}

void BootCAN_Init(uint8_t default_node_id)
{
    memset(&g_config, 0, sizeof(g_config));
    BootCAN_BitmapClear();
    BootCAN_CancelAsyncTasks();

    g_config_valid = BootStorage_LoadConfig(&g_config);

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

    g_app_valid = BootRuntime_ValidatePersistedApp(&g_config);
    if (g_app_valid != 0U)
    {
        g_config_valid = 1U;
    }

    g_boot_requested = BootRuntime_ConsumeBootRequest();
    g_guard_active = 0U;
    g_guard_node_id = 0U;
    g_is_guard = 0U;
    g_app_erased = 0U;
    g_write_active = 0U;
    g_write_size = 0U;
    g_total_packets = 0U;
    g_progress = 0U;
    g_last_error = BOOT_ERR_NONE;
    g_status = BOOT_STATUS_IDLE;
}

uint8_t BootCAN_ShouldJumpApp(void)
{
    return ((g_boot_requested == 0U) && (g_app_valid != 0U)) ? 1U : 0U;
}

void BootCAN_JumpApp(void)
{
    if (BootRuntime_ValidatePersistedApp(&g_config) != 0U)
    {
        g_app_valid = 1U;
        g_config_valid = 1U;
        BootRuntime_JumpToApp();
    }

    g_app_valid = 0U;
    g_status = BOOT_STATUS_ERROR;
    g_last_error = BOOT_ERR_APP_INVALID;
}

void BootCAN_Process(const uint8_t *data, uint8_t len)
{
    BootCAN_ProcessControl(data, len);
}

void BootCAN_ProcessControl(const uint8_t *data, uint8_t len)
{
    Boot_CAN_Frame_t frame;

    if ((data == NULL) || (len != BOOT_CAN_CTRL_DLC))
    {
        return;
    }

    if (BootCRC8_Calculate(data, 7U) != data[7])
    {
        /* Do not trust target/cmd fields enough to generate an error response. */
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
        BootCAN_HandleGetVersion(frame.cmd);
        break;

    case BOOT_CMD_GET_DEVICE_ID:
        BootCAN_HandleGetDeviceId(frame.cmd);
        break;

    case BOOT_CMD_GET_INFO:
        BootCAN_HandleGetInfo(frame.cmd);
        break;

    case BOOT_CMD_ENTER_BOOT:
        BootCAN_HandleEnterBoot(frame.cmd);
        break;

    case BOOT_CMD_SET_GUARD:
        BootCAN_HandleSetGuard(&frame);
        break;

    case BOOT_CMD_RELEASE_GUARD:
        BootCAN_HandleReleaseGuard(&frame);
        break;

    case BOOT_CMD_ERASE:
        BootCAN_HandleErase(frame.cmd);
        break;

    case BOOT_CMD_WRITE:
        BootCAN_HandleWrite(&frame);
        break;

    case BOOT_CMD_READ:
        BootCAN_HandleRead(&frame);
        break;

    case BOOT_CMD_VERIFY:
        BootCAN_HandleVerify(&frame);
        break;

    case BOOT_CMD_WRITE_END:
        BootCAN_HandleWriteEnd(&frame);
        break;

    case BOOT_CMD_PROVIDER_GRANT:
        BootCAN_HandleProviderGrant(&frame);
        break;

    case BOOT_CMD_ABORT:
        BootCAN_HandleAbort(frame.cmd);
        break;

    case BOOT_CMD_JUMP_APP:
        BootCAN_HandleJumpApp(frame.cmd);
        break;

    case BOOT_CMD_RESET:
        BootCAN_HandleReset(frame.cmd);
        break;

    case BOOT_CMD_GET_STATUS:
        BootCAN_HandleGetStatus(frame.cmd);
        break;

    case BOOT_CMD_MISSING_COUNT:
    case BOOT_CMD_MISSING_ITEM:
    default:
        BootCAN_SendError(frame.cmd, BOOT_ERR_BAD_STATE);
        break;
    }
}

void BootCAN_ProcessFDData(const uint8_t *data, uint8_t len)
{
    uint8_t target;
    uint16_t seq;
    uint32_t offset;
    uint32_t valid_len;
    uint8_t ok;

    if ((data == NULL) || (len != BOOT_CAN_FD_DLC))
    {
        return;
    }

    target = data[0];
    if ((target != g_node_id) && (target != BOOT_BROADCAST_ID))
    {
        return;
    }

    if (data[1] != BOOT_FD_CMD_WRITE_DATA)
    {
        return;
    }

    if (BootCAN_IsGuardProtected() != 0U)
    {
        /* Guard only listens during phase 1; it never touches its APP/config. */
        return;
    }

    if (g_write_active == 0U)
    {
        g_last_error = BOOT_ERR_BAD_STATE;
        return;
    }

    seq = BootCAN_ReadU16LE(&data[2]);
    if (seq >= g_total_packets)
    {
        g_last_error = BOOT_ERR_SEQUENCE;
        return;
    }

    if (BootCAN_BitmapGet(seq) != 0U)
    {
        /* Duplicate packet: already read-back verified locally, do not program again. */
        return;
    }

    offset = (uint32_t)seq * BOOT_CAN_FD_PAYLOAD_SIZE;
    if (offset >= g_write_size)
    {
        g_last_error = BOOT_ERR_SEQUENCE;
        return;
    }

    valid_len = g_write_size - offset;
    if (valid_len > BOOT_CAN_FD_PAYLOAD_SIZE)
    {
        valid_len = BOOT_CAN_FD_PAYLOAD_SIZE;
    }

    if (g_write_region == BOOT_WRITE_REGION_APP)
    {
        ok = BootStorage_ProgramApp(offset, &data[8], valid_len);
    }
    else if (g_write_region == BOOT_WRITE_REGION_CONFIG)
    {
        ok = BootStorage_ConfigStageWrite(offset, &data[8], valid_len);
    }
    else
    {
        g_last_error = BOOT_ERR_PROTECTED_REGION;
        return;
    }

    if (ok == 0U)
    {
        /* Packet failure does NOT abort the transfer. Keep bitmap bit at 0 and
         * continue receiving later packets; the master repairs it after WRITE_END.
         */
        g_last_error = BOOT_ERR_FLASH_WRITE;
        return;
    }

    BootCAN_BitmapSet(seq);
    g_received_packets++;
    BootCAN_UpdateProgress();

    if (g_status == BOOT_STATUS_REPAIR)
    {
        /* Stay in REPAIR until master sends WRITE_END again after the round. */
    }
    else
    {
        g_status = BOOT_STATUS_WRITE;
    }
}

static void BootCAN_TaskRead(void)
{
    uint8_t data[4] = {0};
    uint16_t chunk;

    if ((g_read_active == 0U) || (BootCAN_HW_TxFreeLevel() == 0U))
    {
        return;
    }

    chunk = (g_read_remaining > 4U) ? 4U : g_read_remaining;

    if (BootStorage_ReadFlash(g_read_address, data, chunk) == 0U)
    {
        g_read_active = 0U;
        BootCAN_SendError(BOOT_CMD_READ, BOOT_ERR_BAD_ADDRESS);
        return;
    }

    if (BootCAN_SendResponse(BOOT_CMD_READ, BOOT_STATUS_READY, data) != 0U)
    {
        g_read_address += chunk;
        g_read_remaining = (uint16_t)(g_read_remaining - chunk);
        if (g_read_remaining == 0U)
        {
            g_read_active = 0U;
        }
    }
}

static void BootCAN_TaskMissingReport(void)
{
    uint8_t data[4] = {0};

    if ((g_missing_report_active == 0U) || (BootCAN_HW_TxFreeLevel() == 0U))
    {
        return;
    }

    if (g_missing_count_sent == 0U)
    {
        BootCAN_WriteU16LE(&data[0], g_missing_count);
        BootCAN_WriteU16LE(&data[2], g_total_packets);

        if (BootCAN_SendResponse(BOOT_CMD_MISSING_COUNT,
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

        if (BootCAN_BitmapGet(seq) == 0U)
        {
            memset(data, 0, sizeof(data));
            BootCAN_WriteU16LE(&data[0], seq);
            BootCAN_WriteU16LE(&data[2], g_missing_item_index);

            if (BootCAN_SendResponse(BOOT_CMD_MISSING_ITEM,
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
                /* Retry this sequence next Task() call. */
                g_missing_scan_seq--;
            }
            return;
        }
    }

    g_missing_report_active = 0U;
}

static void BootCAN_TaskProvider(void)
{
    uint8_t fd_frame[BOOT_CAN_FD_DLC];
    uint8_t data[4] = {0};

    if (g_provider_done_pending != 0U)
    {
        if (BootCAN_HW_TxFreeLevel() == 0U)
        {
            return;
        }

        data[0] = g_provider_target;
        if (BootCAN_SendResponse(BOOT_CMD_PROVIDER_GRANT,
                                 BOOT_STATUS_READY,
                                 data) != 0U)
        {
            g_provider_done_pending = 0U;
        }
        return;
    }

    if ((g_provider_active == 0U) || (BootCAN_HW_TxFreeLevel() == 0U))
    {
        return;
    }

    if (BootCAN_BuildProviderFrame(g_provider_next_seq,
                                   g_provider_target,
                                   fd_frame) == 0U)
    {
        g_provider_active = 0U;
        BootCAN_SetError(BOOT_ERR_PROVIDER_SOURCE, 0U);
        data[0] = (uint8_t)BOOT_ERR_PROVIDER_SOURCE;
        (void)BootCAN_SendResponse(BOOT_CMD_PROVIDER_GRANT,
                                   BOOT_STATUS_ERROR,
                                   data);
        return;
    }

    if (BootCAN_HW_SendFD(BOOT_CAN_FD_DATA_ID,
                          fd_frame,
                          BOOT_CAN_FD_DLC) == 0U)
    {
        return;
    }

    g_provider_next_seq++;
    g_provider_remaining--;

    if (g_provider_remaining == 0U)
    {
        g_provider_active = 0U;
        g_provider_done_pending = 1U;
    }
}

void BootCAN_Task(void)
{
    /* Control-plane reports first, then READ streaming, then FD provider data. */
    BootCAN_TaskMissingReport();
    BootCAN_TaskRead();
    BootCAN_TaskProvider();
}

uint8_t BootCAN_SendResponse(uint8_t cmd, uint8_t status, const uint8_t *data)
{
    Boot_CAN_Response_t response;

    memset(&response, 0, sizeof(response));
    response.node = g_node_id;
    response.cmd = cmd;
    response.status = status;

    if (data != NULL)
    {
        memcpy(response.data, data, sizeof(response.data));
    }

    response.crc = BootCRC8_Calculate((const uint8_t *)&response, 7U);

    return BootCAN_HW_SendClassic(BOOT_CAN_RESP_BASE_ID + g_node_id,
                                  (const uint8_t *)&response,
                                  BOOT_CAN_CTRL_DLC);
}

uint8_t BootCAN_GetNodeId(void)
{
    return g_node_id;
}

uint8_t BootCAN_GetStatus(void)
{
    return (uint8_t)g_status;
}

uint8_t BootCAN_GetLastError(void)
{
    return (uint8_t)g_last_error;
}

uint8_t BootCAN_GetProgress(void)
{
    return g_progress;
}
