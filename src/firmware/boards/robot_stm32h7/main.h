/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

    /* USER CODE BEGIN EFP */

    /**
     * Initializes the drivetrain
     */
    void initIoDrivetrain(void);

    /**
     * Initializes the networking
     */
    void initIoNetworking(void);

    /**
     * Initializes the power monitor
     */
    void initIoPowerMonitor(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAULT_PWR_BRD_Pin GPIO_PIN_3
#define FAULT_PWR_BRD_GPIO_Port GPIOE
#define DONE_PWR_BRD_Pin GPIO_PIN_4
#define DONE_PWR_BRD_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define IMU_SDA_Pin GPIO_PIN_0
#define IMU_SDA_GPIO_Port GPIOF
#define IMU_SCL_Pin GPIO_PIN_1
#define IMU_SCL_GPIO_Port GPIOF
#define ENCODER_FRONT_LEFT_SIN_Pin GPIO_PIN_3
#define ENCODER_FRONT_LEFT_SIN_GPIO_Port GPIOF
#define ENCODER_FRONT_LEFT_COS_Pin GPIO_PIN_4
#define ENCODER_FRONT_LEFT_COS_GPIO_Port GPIOF
#define ENCODER_BACK_LEFT_COS_Pin GPIO_PIN_5
#define ENCODER_BACK_LEFT_COS_GPIO_Port GPIOF
#define ENCODER_BACK_LEFT_SIN_Pin GPIO_PIN_6
#define ENCODER_BACK_LEFT_SIN_GPIO_Port GPIOF
#define ENCODER_BACK_RIGHT_COS_Pin GPIO_PIN_7
#define ENCODER_BACK_RIGHT_COS_GPIO_Port GPIOF
#define ENCODER_BACK_RIGHT_SIN_Pin GPIO_PIN_8
#define ENCODER_BACK_RIGHT_SIN_GPIO_Port GPIOF
#define HV_SENSE_PWR_BRD_Pin GPIO_PIN_9
#define HV_SENSE_PWR_BRD_GPIO_Port GPIOF
#define WHEEL_FRONT_RIGHT_FF2_Pin GPIO_PIN_10
#define WHEEL_FRONT_RIGHT_FF2_GPIO_Port GPIOF
#define WHEEL_FRONT_RIGHT_CSOUT_Pin GPIO_PIN_0
#define WHEEL_FRONT_RIGHT_CSOUT_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define ENCODER_FRONT_RIGHT_SIN_Pin GPIO_PIN_2
#define ENCODER_FRONT_RIGHT_SIN_GPIO_Port GPIOC
#define ENCODER_FRONT_RIGHT_COS_Pin GPIO_PIN_3
#define ENCODER_FRONT_RIGHT_COS_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define THERM_NSS_Pin GPIO_PIN_4
#define THERM_NSS_GPIO_Port GPIOA
#define THERM_SCK_Pin GPIO_PIN_5
#define THERM_SCK_GPIO_Port GPIOA
#define THERM_MISO_Pin GPIO_PIN_6
#define THERM_MISO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define CH_SEL_1_Pin GPIO_PIN_0
#define CH_SEL_1_GPIO_Port GPIOG
#define CH_SEL_2_Pin GPIO_PIN_1
#define CH_SEL_2_GPIO_Port GPIOG
#define CH_SEL_3_Pin GPIO_PIN_7
#define CH_SEL_3_GPIO_Port GPIOE
#define WHEEL_BACK_RIGHT_FF1_Pin GPIO_PIN_15
#define WHEEL_BACK_RIGHT_FF1_GPIO_Port GPIOE
#define MOTOR_E_MODE_Pin GPIO_PIN_10
#define MOTOR_E_MODE_GPIO_Port GPIOB
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOB
#define RMII_TXD0_Pin GPIO_PIN_12
#define RMII_TXD0_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define MOTOR_C_DIR_Pin GPIO_PIN_14
#define MOTOR_C_DIR_GPIO_Port GPIOB
#define MOTOR_C_MODE_Pin GPIO_PIN_15
#define MOTOR_C_MODE_GPIO_Port GPIOB
#define T_VCP_TX_Pin GPIO_PIN_8
#define T_VCP_TX_GPIO_Port GPIOD
#define T_VCP_RX_Pin GPIO_PIN_9
#define T_VCP_RX_GPIO_Port GPIOD
#define WHEEL_BACK_RIGHT_FF2_Pin GPIO_PIN_10
#define WHEEL_BACK_RIGHT_FF2_GPIO_Port GPIOD
#define MOTOR_C_RESET_Pin GPIO_PIN_11
#define MOTOR_C_RESET_GPIO_Port GPIOD
#define MOTOR_B_MODE_Pin GPIO_PIN_12
#define MOTOR_B_MODE_GPIO_Port GPIOD
#define MOTOR_B_DIR_Pin GPIO_PIN_13
#define MOTOR_B_DIR_GPIO_Port GPIOD
#define DRIBBLER_FF1_Pin GPIO_PIN_14
#define DRIBBLER_FF1_GPIO_Port GPIOD
#define DRIBBLER_FF2_Pin GPIO_PIN_15
#define DRIBBLER_FF2_GPIO_Port GPIOD
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define MOTOR_A_MODE_Pin GPIO_PIN_15
#define MOTOR_A_MODE_GPIO_Port GPIOA
#define SD_DAT2_Pin GPIO_PIN_10
#define SD_DAT2_GPIO_Port GPIOC
#define SD_DAT3_Pin GPIO_PIN_11
#define SD_DAT3_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_12
#define SD_CLK_GPIO_Port GPIOC
#define WIFI_RX_Pin GPIO_PIN_0
#define WIFI_RX_GPIO_Port GPIOD
#define WIFI_TX_Pin GPIO_PIN_1
#define WIFI_TX_GPIO_Port GPIOD
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define ID_SEL_3_Pin GPIO_PIN_3
#define ID_SEL_3_GPIO_Port GPIOD
#define ID_SEL_2_Pin GPIO_PIN_4
#define ID_SEL_2_GPIO_Port GPIOD
#define ID_SEL_4_Pin GPIO_PIN_5
#define ID_SEL_4_GPIO_Port GPIOD
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define PMS_SCL_PWR_BRD_Pin GPIO_PIN_6
#define PMS_SCL_PWR_BRD_GPIO_Port GPIOB
#define PMS_SDA_PWR_BRD_Pin GPIO_PIN_7
#define PMS_SDA_PWR_BRD_GPIO_Port GPIOB
#define CHARGE_PWR_BRD_Pin GPIO_PIN_1
#define CHARGE_PWR_BRD_GPIO_Port GPIOE
    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
