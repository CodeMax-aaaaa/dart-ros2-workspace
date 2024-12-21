/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <sys/types.h>

#include "chalie_leds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t DateAndTime[7] = {};
uint8_t Periph; // 当前进行I2C通信的外设的逻辑地址
uint8_t offset_rx; // 从机被写寄存器当前偏移地�???
uint8_t offset_tx; // 从机被读寄存器当前偏移地�???
uint8_t Error[6] = {0x11, 0x45, 0x14};
static uint8_t first_byte_state = 1; // 是否收到�???1个字�???,也就逻辑地址：已收到�???0：没有收到为1�???
static uint8_t is_transmitting = 0;
uint16_t bq40z50_address = 0x0b;
static uint8_t Rxdata[16] = {0};
HAL_StatusTypeDef condition;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void sys_enter_standby_mode(void);

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

void ManufacturerBlockAccess_write(uint16_t write_command);

void ManufacturerBlockAccess_read(uint16_t read_command);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_I2C1_SMBUS_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    chalie_leds_init();
    HAL_SMBUS_EnableListen_IT(&hsmbus1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    int code = 0;
    uint8_t soc = 0;
    uint8_t address = 0x17;
    uint8_t soc_address = 0x0d;
    uint8_t read = 0;
    uint8_t current_address = 0x0a;
    int16_t current = 0;
    uint8_t last_timer_it_state = 0;
    uint16_t DSG_Toggle = 0x0020;
    uint16_t CHG_Toggle = 0x001f;
    uint16_t PCHG_Toggle = 0x001e;
    uint16_t Status = 0x0057;
    uint16_t FET_Control = 0x0022;

    uint8_t button_state_machine = 0;
    enum {
        BUTTON_STATE_IDLE = 0,
        BUTTON_STATE_SHORT_PRESS = 1,
        BUTTON_STATE_L
    };
    // 启动DSG，CHG，PCHG
    // ManufacturerBlockAccess_write(FET_Control);
    // ManufacturerBlockAccess_write(DSG_Toggle);
    // ManufacturerBlockAccess_write(CHG_Toggle);
    ManufacturerBlockAccess_write(PCHG_Toggle);
    // ManufacturerBlockAccess_read(Status);
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // SMBUS 读取0x0D寄存�??????? State of Charge，格式WORD
        if (HAL_GetTick() % 1000 > 500 && !read) {
            HAL_I2C_Master_Transmit(&hsmbus1, address, (uint8_t *) &soc_address, 1, 100);
            HAL_I2C_Master_Receive(&hsmbus1, address, (uint8_t *) &soc, 1, 100);
            HAL_I2C_Master_Transmit(&hsmbus1, address, (uint8_t *) &current_address, 1, 100);
            HAL_I2C_Master_Receive(&hsmbus1, address, (uint8_t *) &current, 2, 100);

            read = 1;
            // 将SOC0�???????100转换�???????0-7的范�???????
            code = soc / 12.5 + 1;
        } else if (HAL_GetTick() % 1000 < 500) {
            read = 0;
        }

        // 如果PA0被按下，LED显示SOC
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
                for (int i = 1; i < code; i += 2) {
                    chalie_led_code(i);
                    HAL_Delay(100);
                }
                chalie_led_code(code);
            }

            HAL_Delay(3000);
            HAL_TIM_Base_Stop_IT(&htim2);
            last_timer_it_state = 0;
            chalie_led_timer_update(1);
            chalie_leds_set(1, GPIO_PIN_RESET);
        } else if (current > 50 || current < -100) {
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
                for (int i = 1; i < code; i += 2) {
                    chalie_led_code(i);
                    HAL_Delay(100);
                }
                chalie_led_code(code);
            }
            chalie_led_code(code);
            HAL_Delay(10);
        } else {
            if (last_timer_it_state) {
                HAL_TIM_Base_Stop_IT(&htim2);
                last_timer_it_state = 0;
                chalie_led_timer_update(1);
                chalie_leds_set(1, GPIO_PIN_RESET);
            }

            HAL_Delay(10);
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void RTC_LoadFromBKUP(void) {
    // �??????查是否有时间保存
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)) {
        RTC_TimeTypeDef sTime;
        sTime.Hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
        sTime.Minutes = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
        sTime.Seconds = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

        // 禁用 RTC 写保�??????
        __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

        // 设置 RTC 时间
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
        } else {
        }

        // 启用 RTC 写保�??????
        __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
    } else {
    }
}

void sys_enter_standby_mode(void) {
    __HAL_RCC_PWR_CLK_ENABLE(); // 使能PWR时钟
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标记
    HAL_PWR_EnterSTANDBYMode(); //进入待机模式
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2)
        chalie_led_timer_update(0);
}

void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    // 完成�??????次�?�信，清除状�??????
    first_byte_state = 1;
    Periph = 0;
    offset_rx = 0;
    offset_tx = 0;
    is_transmitting = 0;
    HAL_SMBUS_EnableListen_IT(&hsmbus1); // slave is ready again
}

// I2C设备地址回调函数（地�???匹配上以后会进入该函数）
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // 主机发�?�，从机接收
        if (first_byte_state) {
            // 准备接收�???1个字节数�???
            HAL_SMBUS_Slave_Receive_IT(&hsmbus1, &Periph, 1, SMBUS_NEXT_FRAME); // 每次第个数据均为外设逻辑地址
        }
    } else {
        // 主机接收，从机发�???
        // 匹配外设逻辑地址
        is_transmitting = 1;
        switch (Periph) {
            // 如果外设逻辑地址指向RTC
            case 1: {
                // 打开I2C中断发�??,将DateAndTime[]中的数据依次发�??
                RTC_DateTypeDef sDate;
                RTC_TimeTypeDef sTime;

                HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

                DateAndTime[0] = sDate.Year;
                DateAndTime[1] = sDate.Month;
                DateAndTime[2] = sDate.Date;
                DateAndTime[3] = sDate.WeekDay;
                DateAndTime[4] = sTime.Hours;
                DateAndTime[5] = sTime.Minutes;
                DateAndTime[6] = sTime.Seconds;

                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &DateAndTime[offset_tx], 1, SMBUS_NEXT_FRAME);
                // 打开中断并把DateAndTime[]里面对应的数据发送给主机
                break;
            }
            default:
                // 全都不匹配，发�?�错误码
                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
            // 打开中断并把Error[]里面对应的数据发送给主机
        }
    }
}

// I2C数据接收回调函数（在I2C完成�???后一次次接收时会关闭中断并调用该函数，因此在处理完成后需要手动重新打�???中断接收
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    if (first_byte_state) {
        // 收到的第1个字节数据（外设地址�???
        first_byte_state = 0;
    } else {
        // 收到的第N个字节数�???
        offset_rx++;
    }

    if (is_transmitting == 0) {
        // 匹配外设逻辑地址
        switch (Periph) {
            // 如果外设逻辑地址指向RTC
            case 1:
                // 打开I2C中断接收,下一个收到的数据将存放到DateAndTime[offset_rx]
                if (offset_rx <= 6) {
                    HAL_SMBUS_Slave_Receive_IT(&hsmbus1, &DateAndTime[offset_rx], 1, SMBUS_NEXT_FRAME);
                    // 接收数据存到DateAndTime[]里面对应的位�??????
                } else if (offset_rx >= 7) {
                    RTC_TimeTypeDef sTime;
                    sTime.Hours = DateAndTime[4];
                    sTime.Minutes = DateAndTime[5];
                    sTime.Seconds = DateAndTime[6];

                    RTC_DateTypeDef sDate;
                    sDate.Year = DateAndTime[0];
                    sDate.Month = DateAndTime[1];
                    sDate.Date = DateAndTime[2];
                    sDate.WeekDay = DateAndTime[3];


                    __disable_irq();
                    // 更新RTC时钟的设�???
                    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
                        Error_Handler();
                    }
                    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
                        Error_Handler();
                    }

                    __enable_irq();
                }
                break;
            default:
                // 全都不匹配，发�?�错误码
                HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
        }
    }
}

// I2C数据发�?�回调函数（在I2C完成�???后一次发送后会关闭中断并调用该函数，因此在处理完成后�???要手动重新打�???中断发�??
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    offset_tx++; // 每发送一个数据，偏移+1
    // 匹配外设逻辑地址
    switch (Periph) {
        // 如果外设逻辑地址指向RTC
        case 1:
            // 打开I2C中断发�??,将DateAndTime[]中的数据依次发送
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &DateAndTime[offset_tx], 1, SMBUS_NEXT_FRAME);

        // 打开中断并把DateAndTime[]里面对应的数据发送给主机
            break;
        default:
            // 全都不匹配，发�?�错误码
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
        // 打开中断并把Error[]里面对应的数据发送给主机
    }
}

void ManufacturerBlockAccess_write(uint16_t write_command) {
    uint8_t Txdata[4] = {};
    Txdata[0] = 0x44;
    Txdata[1] = 0x02;
    Txdata[2] = write_command & 0xff;
    Txdata[3] = (write_command >> 8) & 0xff;

    // 确保总线空闲
    if (hsmbus1.State == HAL_SMBUS_STATE_READY) {
        condition = HAL_SMBUS_Master_Transmit_IT(&hsmbus1, bq40z50_address, Txdata, 4, SMBUS_NEXT_FRAME);
    }
}


void ManufacturerBlockAccess_read(uint16_t read_command) {
    uint8_t Txdata[4] = {};
    Txdata[0] = 0x44;
    Txdata[1] = 2;
    Txdata[2] = read_command & 0xff;
    Txdata[3] = (read_command >> 8) & 0xff;
    HAL_SMBUS_Master_Transmit_IT(&hsmbus1, bq40z50_address,(uint8_t*) &Txdata, 4, SMBUS_NEXT_FRAME);

    HAL_SMBUS_Master_Transmit_IT(&hsmbus1, bq40z50_address, Txdata, 1, SMBUS_NEXT_FRAME);
    HAL_SMBUS_Master_Receive_IT(&hsmbus1, bq40z50_address, (uint8_t*) &Rxdata, sizeof(Rxdata), SMBUS_NEXT_FRAME);
}

void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    hsmbus->State = HAL_SMBUS_STATE_READY;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
