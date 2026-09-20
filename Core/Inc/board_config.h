#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H /* 防止板卡时钟配置头文件重复包含。 */

/*
 * 板卡外部晶振频率：当前硬件为 24 MHz。
 * 更换为 16 MHz 晶振板时，只需把下面一行改成 16000000UL，
 * 不需要修改 CMake。24/16 MHz 两种配置都会生成 168 MHz 系统时钟。
 */
#define BOARD_HSE_HZ 24000000UL

#if (BOARD_HSE_HZ != 24000000UL) && (BOARD_HSE_HZ != 16000000UL)
#error "BOARD_HSE_HZ must be 24000000UL or 16000000UL"
#endif

#ifndef HSE_VALUE
#define HSE_VALUE BOARD_HSE_HZ
#endif

#endif /* BOARD_CONFIG_H */
