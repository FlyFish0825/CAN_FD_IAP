#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H /* 防止板卡时钟配置头文件重复包含。 */

/*
 * 本文件是项目维护的板级配置入口，不属于 CubeMX 自动生成区。
 * 只有与具体硬件绑定的时钟常量放在这里；协议和 Bootloader 地址布局
 * 由 bootloader.h 维护，避免修改晶振时意外改变升级协议。
 */

/*
 * 板卡外部晶振频率：本工程只使用 16 MHz 晶振，16 MHz 是唯一受支持的配置。
 *
 * 该常量是整机时钟的唯一来源：stm32g4xx_hal_conf.h 在定义 HSE_VALUE 之前
 * 会先包含本文件，因此 HAL、SysTick、system_stm32g4xx.c 的 SystemCoreClock
 * 以及 FDCAN 位定时全部按这里的频率换算。
 *
 * 时钟链：16 MHz / PLLM(2) = 8 MHz VCO 输入，× PLLN(42) = 336 MHz VCO，
 *         / PLLR(2) = 168 MHz SYSCLK，再经 AHB/APB 不分频得到 PCLK1。
 * FDCAN 内核时钟取 PCLK1 = 168 MHz，据此得到仲裁段 1 Mbit/s、数据段 8 Mbit/s。
 *
 * 如果换成非 16 MHz 晶振，必须同时修改本值和 main.c 中的 PLLN，
 * 否则下面的 #if 会直接编译失败，避免生成时钟错误的固件。
 */
#define BOARD_HSE_HZ 16000000UL /* 外部晶振频率，单位 Hz。 */

#if (BOARD_HSE_HZ != 16000000UL)
#error "This project only supports a 16 MHz HSE crystal; BOARD_HSE_HZ must be 16000000UL"
#endif

#ifndef HSE_VALUE
#define HSE_VALUE BOARD_HSE_HZ /* CMSIS/HAL 使用的外部时钟频率。 */
#endif

#endif /* BOARD_CONFIG_H */
