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
static uint8_t offset_rx; // 从机被写寄存器当前偏移地�???????
static uint8_t offset_tx; // 从机被读寄存器当前偏移地�???????
uint8_t Error[6] = {0x11, 0x45, 0x14};
static uint8_t first_byte_state = 1; // 是否收到�???????1个字�???????,也就逻辑地址：已收到�???????0：没有收到为1�???????
static uint8_t is_transmitting = 0;
uint16_t bq40z50_address = 0x0b;
static uint8_t Rxdata[16] = {0};
uint8_t soc = 0;
uint8_t rx_buffer[5] = {0}; // 存储读取结果
static HAL_StatusTypeDef smbus_status = HAL_OK;
uint8_t Command = 0x44;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void sys_enter_standby_mode(void);

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);

HAL_StatusTypeDef HAL_SMBUS_Master_Transmit(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size, uint32_t XferOptions, uint32_t Timeout);

HAL_StatusTypeDef HAL_SMBUS_Master_Receive(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                           uint16_t Size, uint32_t XferOptions, uint32_t Timeout);

void ManufacturerBlockAccess_write(uint16_t Data);

void ManufacturerBlockAccess_read(uint16_t Data, uint8_t *Rxdata, uint8_t size);

void wait_for_smbus(void);

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

    // HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    chalie_leds_init();
    // HAL_I2C_EnableListen_IT(&hi2c1);
    // HAL_SMBUS_EnableListen_IT(&hsmbus1);
    // �???查是否由RTC唤醒
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WU) != RESET) {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标志
        HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); // 停止唤醒计时�???
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    int code = 0;

    uint8_t address = 0x17;
    uint8_t soc_address = 0x0d;
    uint8_t read = 0;
    uint8_t current_address = 0x0a;
    int16_t current = 0;
    uint8_t voltage_address = 0x09;
    uint16_t voltage = 0;
    uint8_t last_timer_it_state = 0;
    uint16_t FET_Control = 0x0022;
    uint16_t DSG_Toggle = 0x0020;
    uint16_t CHG_Toggle = 0x001f;
    uint16_t PCHG_Toggle = 0x001e;
    uint16_t MAC_Status = 0x0057;

    static uint8_t Manufacturing_Status[5] = {0};
    static uint8_t Manufacturing_Status_BIT[16] = {0};


    typedef enum {
        BUTTON_STATE_IDLE = 0, // 按钮空闲
        BUTTON_STATE_SHORT_PRESS, // 短按
        BUTTON_STATE_LONG_PRESS // 长按
    } ButtonState;
    ButtonState button_state = BUTTON_STATE_IDLE;

    // 获取Manufacturing Status
    ManufacturerBlockAccess_write(MAC_Status);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Manufacturing_Status, 5, SMBUS_LAST_FRAME_NO_PEC, 100);
    uint16_t value = (Manufacturing_Status[4] << 8) | Manufacturing_Status[3];

    // 解包到Manufacturing_Status_BIT数组
    for (int i = 0; i < 16; i++) {
        uint8_t mask = (1 << (15 - i)); // 从第16位到第1位的掩码
        if (value & mask) {
            Manufacturing_Status_BIT[i] = 1;
        } else {
            Manufacturing_Status_BIT[i] = 0;
        }
    }

    // 尝试打开PCHG、CHG、DSG FET
    if (Manufacturing_Status_BIT[11] == 1) {
        // 检查FET_EN
        ManufacturerBlockAccess_write(FET_Control);
        HAL_Delay(100); // 等待FET_EN响应
    }
    if (Manufacturing_Status_BIT[13] == 0) {
        // 检查DSG
        ManufacturerBlockAccess_write(DSG_Toggle);
    }
    if (Manufacturing_Status_BIT[14] == 0) {
        // 检查CHG
        ManufacturerBlockAccess_write(CHG_Toggle);
    }
    if (Manufacturing_Status_BIT[15] == 0) {
        // 检查PCHG
        ManufacturerBlockAccess_write(PCHG_Toggle);
    }

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */


        // SMBUS 读取0x0D寄存�??????????? State of Charge，格式WORD
        if (HAL_GetTick() % 1000 > 500 && !read) {
            // 实时更新Manufacturing Status
            ManufacturerBlockAccess_write(MAC_Status);
            HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
            HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Manufacturing_Status, 5, SMBUS_LAST_FRAME_NO_PEC,
                                     100);
            uint16_t value = (Manufacturing_Status[4] << 8) | Manufacturing_Status[3];

            // 解包到Manufacturing_Status_BIT数组
            for (int i = 0; i < 16; i++) {
                uint8_t mask = (1 << (15 - i)); // 从第16位到第1位的掩码
                if (value & mask) {
                    Manufacturing_Status_BIT[i] = 1;
                } else {
                    Manufacturing_Status_BIT[i] = 0;
                }
            }

            //实时更新soc、current和voltage
            HAL_SMBUS_Master_Transmit(&hsmbus1, address, (uint8_t *) &soc_address, 1, SMBUS_FIRST_FRAME, 100);
            HAL_SMBUS_Master_Receive(&hsmbus1, address, (uint8_t *) &soc, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
            HAL_SMBUS_Master_Transmit(&hsmbus1, address, (uint8_t *) &current_address, 1, SMBUS_FIRST_FRAME, 100);
            HAL_SMBUS_Master_Receive(&hsmbus1, address, (uint8_t *) &current, 2, SMBUS_LAST_FRAME_NO_PEC, 100);
            HAL_SMBUS_Master_Transmit(&hsmbus1, address, (uint8_t *) &voltage_address, 1, SMBUS_FIRST_FRAME, 100);
            HAL_SMBUS_Master_Receive(&hsmbus1, address, (uint8_t *) &voltage, 2, SMBUS_LAST_FRAME_NO_PEC, 100);
            read = 1;
            // 将SOC0�???????????100转换�???????????0-7的范�???????????
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

        // 系统FET电源状态机
        GPIO_PinState button_state_read = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        uint32_t button_press_time = 0; // 按钮按下的时间戳
        switch (button_state) {
            case BUTTON_STATE_IDLE:
                if (button_state_read == GPIO_PIN_RESET) {
                    // 按钮被按�???
                    button_press_time = HAL_GetTick(); // 记录按下的时间戳
                    button_state = BUTTON_STATE_SHORT_PRESS;
                }
                break;

            case BUTTON_STATE_SHORT_PRESS:
                if (button_state_read == GPIO_PIN_SET) {
                    // 按钮释放
                    uint32_t press_duration = HAL_GetTick() - button_press_time;
                    if (press_duration < 1000) {
                        // 短按逻辑
                        chalie_leds_set(1, GPIO_PIN_RESET);
                        for (int i = 1; i < code; i += 2) {
                            chalie_led_code(i);
                            HAL_Delay(100);
                        }
                        chalie_led_code(code);
                    }
                    button_state = BUTTON_STATE_IDLE;
                } else if ((HAL_GetTick() - button_press_time) >= 500) {
                    button_state = BUTTON_STATE_LONG_PRESS; // 切换到长按状�???
                }
                break;

            case BUTTON_STATE_LONG_PRESS:
                if (button_state_read == GPIO_PIN_SET) {
                    // 按钮释放
                    sys_enter_standby_mode(); // 进入待机模式
                    button_state = BUTTON_STATE_IDLE;
                }
                break;

            default:
                button_state = BUTTON_STATE_IDLE;
                break;
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
    // �??????????查是否有时间保存
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)) {
        RTC_TimeTypeDef sTime;
        sTime.Hours = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
        sTime.Minutes = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
        sTime.Seconds = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

        // 禁用 RTC 写保�??????????
        __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

        // 设置 RTC 时间
        if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK) {
        } else {
        }

        // 启用 RTC 写保�??????????
        __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
    } else {
    }
}

void sys_enter_standby_mode(void) {
    __HAL_RCC_PWR_CLK_ENABLE(); // 使能 PWR 时钟
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标志

    // 配置 RTC 唤醒定时器，实现 10 秒钟唤醒
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 9, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK) {
        Error_Handler();
    }

    // 进入待机模式
    HAL_PWR_EnterSTANDBYMode();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2)
        chalie_led_timer_update(0);
}

void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus) {
    // 检查发生的错误类型

    // 其他错误处理代码
}


void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    // 完成�??????????次�?�信，清除状�??????????
    first_byte_state = 1;
    Periph = 0;
    offset_rx = 0;
    offset_tx = 0;
    is_transmitting = 0;
    HAL_SMBUS_EnableListen_IT(&hsmbus1); // slave is ready again
}

// I2C设备地址回调函数（地�???????匹配上以后会进入该函数）
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // 主机发�?�，从机接收
        if (first_byte_state) {
            // 准备接收�???????1个字节数�???????
            HAL_SMBUS_Slave_Receive_IT(&hsmbus1, &Periph, 1, SMBUS_NEXT_FRAME); // 每次第个数据均为外设逻辑地址
        }
    } else {
        // 主机接收，从机发�???????
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

// I2C数据接收回调函数（在I2C完成�???????后一次次接收时会关闭中断并调用该函数，因此在处理完成后需要手动重新打�???????中断接收
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    if (first_byte_state) {
        // 收到的第1个字节数据（外设地址�???????
        first_byte_state = 0;
    } else {
        // 收到的第N个字节数�???????
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
                    // 接收数据存到DateAndTime[]里面对应的位�??????????
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
                    // 更新RTC时钟的设�???????
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

// I2C数据发�?�回调函数（在I2C完成�???????后一次发送后会关闭中断并调用该函数，因此在处理完成后�???????要手动重新打�???????中断发�??
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    offset_tx++; // 每发送一个数据，偏移+1
    // 匹配外设逻辑地址
    switch (Periph) {
        // 如果外设逻辑地址指向RTC
        case 1:
            // 打开I2C中断发�??,将DateAndTime[]中的数据依次发�??
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &DateAndTime[offset_tx], 1, SMBUS_NEXT_FRAME);

        // 打开中断并把DateAndTime[]里面对应的数据发送给主机
            break;
        default:
            // 全都不匹配，发�?�错误码
            HAL_SMBUS_Slave_Transmit_IT(&hsmbus1, &Error[offset_tx], 1, SMBUS_NEXT_FRAME);
        // 打开中断并把Error[]里面对应的数据发送给主机
    }
}

void wait_for_smbus(void) {
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > 100) {
            smbus_status = HAL_TIMEOUT;
            break;
        }
    }
}

void ManufacturerBlockAccess_write(uint16_t Data) {
    uint8_t Txdata[4] = {0};
    Txdata[0] = 0x44;
    Txdata[1] = 0x02;
    Txdata[2] = Data & 0xff;
    Txdata[3] = (Data >> 8) & 0xff;
    HAL_SMBUS_Master_Transmit_IT(&hsmbus1, bq40z50_address << 1, Txdata, 4, SMBUS_LAST_FRAME_NO_PEC);
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > 100) {
            Error_Handler(); // 超时处理
            smbus_status = HAL_TIMEOUT;
        }
    }
}

// void ManufacturerBlockAccess_read(uint16_t Data, uint8_t *Rxdata, uint8_t size) {
//     ManufacturerBlockAccess_write(Data);
//     HAL_SMBUS_Master_Transmit_IT(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_FIRST_FRAME);
//     wait_for_smbus();
//     HAL_SMBUS_Master_Receive_IT(&hsmbus1, bq40z50_address << 1, Rxdata, size, SMBUS_LAST_FRAME_NO_PEC);
//     wait_for_smbus();
// }

void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus) {
    hsmbus->State = HAL_SMBUS_STATE_READY;
}

HAL_StatusTypeDef HAL_SMBUS_Master_Transmit(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                            uint16_t Size, uint32_t XferOptions, uint32_t Timeout) {
    HAL_SMBUS_Master_Transmit_IT(hsmbus, DevAddress, pData, Size, XferOptions);
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > Timeout) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SMBUS_Master_Receive(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData,
                                           uint16_t Size, uint32_t XferOptions, uint32_t Timeout) {
    HAL_SMBUS_Master_Receive_IT(hsmbus, DevAddress, pData, Size, XferOptions);
    uint32_t smbus_start_time = HAL_GetTick();
    while (HAL_SMBUS_GetState(&hsmbus1) != HAL_SMBUS_STATE_READY) {
        if (HAL_GetTick() - smbus_start_time > Timeout) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
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
