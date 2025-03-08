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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <time.h>
#include <sys/types.h>

#include "chalie_leds.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum BMS_State {
    BMS_STATE_INITIALIZING = 0,
    BMS_STATE_IDLE = 1,
    BMS_STATE_SLEEP = 2,
    BMS_STATE_KEY_FIRST_PRESS = 3,
    BMS_STATE_KEY_SHORT_PRESSED_ONCE = 4,
    BMS_STATE_KEY_SHORT_LONG_PRESS = 5,
    BMS_STATE_KEY_LONG_PRESS = 6
};

enum FET_State {
    FET_STATE_OFF = 0,
    FET_STATE_ON
};

enum Light_State {
    LIGHT_STATE_OFF = 0,
    LIGHT_STATE_BRIEF_DISPLAY,
    LIGHT_STATE_SHORT_LONG_PRESS,
    LIGHT_STATE_STEADY_DISPLAY,
    LIGHT_STATE_ERROR_DISPLAY
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOC_THRESHOLD_LOW    5    // 5% SOC critical low
#define VOLTAGE_THRESHOLD    8000 // 8V in mV for power on
#define KEY_LONG_PRESS_TIME  1000 // 1s for long press
#define NO_LOAD_TIMEOUT      10000 // 10s timeout for 0 load
#define I2C_TIMEOUT          100  // I2C timeout in ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t display_active = 0; // 是否正在显示电量
uint8_t last_timer_it_state = 0;
uint8_t is_pressing = 0;

enum BMS_State current_bms_state = BMS_STATE_INITIALIZING;
enum FET_State current_fet_state = FET_STATE_OFF;
enum Light_State current_light_state = LIGHT_STATE_OFF;
uint8_t pres_state = 0; // 0: inactive, 1: active
uint8_t zero_load_state = 0; // 0: normal 1: zero load
uint32_t zero_load_time = 0;
uint8_t first_time_zero_load = 0;
uint32_t key_press_start_time = 0;

uint8_t bq40z50_address = 0x0b;
uint8_t read = 0;
int16_t current = 0;
uint16_t voltage = 0;
uint8_t soc = 1;
int code = 0;

static uint8_t Manufacturing_Status_HEX[5] = {0};
static uint8_t Manufacturing_Status_BIN[16] = {0};
static uint8_t Safety_Alert_HEX[7] = {0};
static uint8_t Safety_Alert_BIN[32] = {0};
static uint8_t Operation_Status_HEX[7] = {0};
static uint8_t Operation_Status_BIN[32] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void BMS_StateMachine(void);

void UpdateBatteryStatus(void);

void UpdateLightState(void);

void UpdateFetState(uint8_t fet_enable, uint8_t dsg_enable, uint8_t chg_enable, uint8_t pchg_enable);

void CheckPRES(void);

void CheckZeroLoad(void);

void CheckFet(void);

void Enter_Standby_Mode(void);

// void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
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
    MX_TIM21_Init();
    /* USER CODE BEGIN 2 */

    // HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    chalie_leds_init();
    // HAL_SMBUS_EnableListen_IT(&hsmbus1);
    // // 检查是否由RTC唤醒
    // if (__HAL_PWR_GET_FLAG(PWR_FLAG_WU) != RESET) {
    //     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); // 清除唤醒标志
    //     // HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); // 停止唤醒计时器
    // }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        BMS_StateMachine();
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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void BMS_StateMachine(void) {
    // 每5000ms更新一次电池状态
    if (HAL_GetTick() % 5000 <= 1000 && !read) {
        UpdateBatteryStatus();
        read = 1;
    } else if (HAL_GetTick() % 5000 >= 1000) {
        read = 0;
    }

    // 检查PRES状态、FET状态和按键状态
    CheckPRES();
    uint8_t key_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1 : 0;

    switch (current_bms_state) {
        case BMS_STATE_INITIALIZING:
            // 初始化I2C和RTC，尝试打开FET
            UpdateFetState(0, 1, 1, 1); // 尝试开启CHG和DSG FET
            current_bms_state = BMS_STATE_IDLE;
            break;

        case BMS_STATE_IDLE:
            // 系统处于空闲状态，等待外部触发
            if (key_pressed) {
                current_bms_state = BMS_STATE_KEY_FIRST_PRESS;
                key_press_start_time = HAL_GetTick();
            }
            if (pres_state && current_fet_state == FET_STATE_OFF || (
                    current_fet_state == FET_STATE_ON && zero_load_state)) {
                current_bms_state = BMS_STATE_SLEEP;
            }
        // 可添加其他触发条件
            break;

        case BMS_STATE_KEY_FIRST_PRESS:
            if (!key_pressed) {
                current_bms_state = BMS_STATE_KEY_SHORT_PRESSED_ONCE;
                current_light_state = LIGHT_STATE_BRIEF_DISPLAY;
                key_press_start_time = HAL_GetTick();
            }
            break;

        case BMS_STATE_KEY_SHORT_PRESSED_ONCE:
            current_light_state = LIGHT_STATE_BRIEF_DISPLAY;
            if (key_pressed && (HAL_GetTick() - key_press_start_time < 2000)) {
                key_press_start_time = HAL_GetTick();
                current_bms_state = BMS_STATE_KEY_SHORT_LONG_PRESS;
                current_light_state = LIGHT_STATE_SHORT_LONG_PRESS;
            } else if (!key_pressed && HAL_GetTick() - key_press_start_time >= 2000) {
                current_bms_state = BMS_STATE_IDLE;
                //
                current_light_state = LIGHT_STATE_OFF;
            }
            break;

        case BMS_STATE_KEY_SHORT_LONG_PRESS:
            current_light_state = LIGHT_STATE_SHORT_LONG_PRESS;
            if (!key_pressed) {
                current_bms_state = BMS_STATE_KEY_SHORT_PRESSED_ONCE;
                current_light_state = LIGHT_STATE_BRIEF_DISPLAY;
                key_press_start_time = HAL_GetTick();
            } else if (HAL_GetTick() - key_press_start_time >= 1000) {
                current_bms_state = BMS_STATE_KEY_LONG_PRESS;
                current_light_state = LIGHT_STATE_STEADY_DISPLAY;
                current_fet_state = FET_STATE_ON;
            }
            break;

        case BMS_STATE_KEY_LONG_PRESS:
            if (!key_pressed) {
                current_bms_state = BMS_STATE_IDLE;
            }
            break;

        case BMS_STATE_SLEEP:
            current_light_state = LIGHT_STATE_OFF;
            if (pres_state == 1 && current_fet_state == FET_STATE_ON || key_pressed) {
                key_press_start_time = HAL_GetTick();
            }
        // 进入standby模式
            Enter_Standby_Mode();
            break;

        default:
            current_bms_state = BMS_STATE_INITIALIZING;
            break;
    }

    UpdateFetState(2, current_fet_state == FET_STATE_ON ? 1 : 0, current_fet_state == FET_STATE_ON ? 1 : 0, 2);
    UpdateLightState();
}

/* 更新电池状态 -----------------------------------------------------------*/
void UpdateBatteryStatus(void) {
    uint8_t Command = 0x44;
    uint16_t Safety_Alert = 0x0050;
    uint16_t Operation_Status = 0x0054;
    uint16_t Manufacturing_Status = 0x0057;
    uint8_t soc_address = 0x0e;
    uint8_t current_address = 0x0a;
    uint8_t voltage_address = 0x09;

    // 更新soc、current和voltage的数值
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, (uint8_t *) &soc_address, 1, SMBUS_FIRST_FRAME, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t *) &soc, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, (uint8_t *) &current_address, 1, SMBUS_FIRST_FRAME, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t *) &current, 2, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, (uint8_t *) &voltage_address, 1, SMBUS_FIRST_FRAME, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, (uint8_t *) &voltage, 2, SMBUS_LAST_FRAME_NO_PEC, 100);
    code = (soc - 1) / 12.5; // 将SOC从0~100转换为0-7的范围

    // 更新Manufacturing Status
    ManufacturerBlockAccess_write(Manufacturing_Status);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Manufacturing_Status_HEX, 5,
                             SMBUS_LAST_FRAME_NO_PEC,
                             100);
    uint16_t value_1 = Manufacturing_Status_HEX[4] << 8 | Manufacturing_Status_HEX[3];
    // 解包到Manufacturing_Status_BIT数组
    for (int i = 0; i < 16; i++) {
        uint16_t mask = 1 << 15 - i; // 从第16位到第1位的掩码
        if (value_1 & mask) {
            Manufacturing_Status_BIN[i] = 1;
        } else {
            Manufacturing_Status_BIN[i] = 0;
        }
    }

    // 实时更新Safety Alert(C+D)
    ManufacturerBlockAccess_write(Safety_Alert);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Safety_Alert_HEX, 7, SMBUS_LAST_FRAME_NO_PEC,
                             100);
    uint32_t value_2 = Safety_Alert_HEX[6] << 24 | Safety_Alert_HEX[5] << 16 | Safety_Alert_HEX[4] << 8 |
                       Safety_Alert_HEX[3];
    // 解包到Safety_Alert_BIN数组
    for (int i = 0; i < 32; i++) {
        uint32_t mask = 1 << 31 - i; // 从第32位到第1位的掩码
        if (value_2 & mask) {
            Safety_Alert_BIN[i] = 1;
        } else {
            Safety_Alert_BIN[i] = 0;
        }
    }

    // 实时更新Operation Status
    ManufacturerBlockAccess_write(Operation_Status);
    HAL_SMBUS_Master_Transmit(&hsmbus1, bq40z50_address << 1, &Command, 1, SMBUS_LAST_FRAME_NO_PEC, 100);
    HAL_SMBUS_Master_Receive(&hsmbus1, bq40z50_address << 1, Operation_Status_HEX, 6, SMBUS_LAST_FRAME_NO_PEC,
                             100);
    uint32_t value_3 = Operation_Status_HEX[6] << 24 | Operation_Status_HEX[5] << 16 | Operation_Status_HEX[4]
                       << 8 | Operation_Status_HEX[3];

    // 解包到Operation_Status_BIN数组
    for (int i = 0; i < 32; i++) {
        uint32_t mask = 1 << 31 - i; // 从第24位到第1位的掩码
        if (value_3 & mask) {
            Operation_Status_BIN[i] = 1;
        } else {
            Operation_Status_BIN[i] = 0;
        }
    }

    static uint8_t is_initial = 0;
    if (!is_initial) {
        is_initial = 1;
        UpdateFetState(0, 1, 1, 1);
    }
    CheckFet();
}

/* 控制FET ---------------------------------------------------------------*/
void UpdateFetState(uint8_t fet_enable, uint8_t dsg_enable, uint8_t chg_enable, uint8_t pchg_enable) {
    uint16_t FET_Control = 0x0022;
    uint16_t DSG_FET_Toggle = 0x0020;
    uint16_t CHG_FET_Toggle = 0x001f;
    uint16_t PCHG_FET_Toggle = 0x001e;

    if ((Manufacturing_Status_BIN[11] == 0 && fet_enable == 1) || (
            Manufacturing_Status_BIN[11] == 1 && fet_enable == 0)) {
        ManufacturerBlockAccess_write(FET_Control);
    }

    if ((Manufacturing_Status_BIN[13] == 0 && dsg_enable == 1) || (
            Manufacturing_Status_BIN[13] == 1 && dsg_enable == 0)) {
        ManufacturerBlockAccess_write(DSG_FET_Toggle);
    }


    if ((Manufacturing_Status_BIN[14] == 0 && chg_enable == 1) || (
            Manufacturing_Status_BIN[14] == 1 && chg_enable == 0)) {
        ManufacturerBlockAccess_write(CHG_FET_Toggle);
    }

    if ((Manufacturing_Status_BIN[15] == 0 && pchg_enable == 1) || (
            Manufacturing_Status_BIN[15] == 1 && pchg_enable == 0)) {
        ManufacturerBlockAccess_write(PCHG_FET_Toggle);
    }
}

/* 更新灯效 --------------------------------------------------------------*/
void UpdateLightState(void) {
    switch (current_light_state) {
        case LIGHT_STATE_OFF:
            if (last_timer_it_state) {
                HAL_TIM_Base_Stop_IT(&htim2);
                last_timer_it_state = 0;
            }
            chalie_led_timer_update(1);
            chalie_leds_set(1, GPIO_PIN_RESET);
            break;

        case LIGHT_STATE_BRIEF_DISPLAY:
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
            }
            chalie_led_code(code);
            break;

        case LIGHT_STATE_SHORT_LONG_PRESS:
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
                chalie_leds_set(1, GPIO_PIN_RESET);
            }
            chalie_led_code((HAL_GetTick() - key_press_start_time) / 250 * 2 + 1);
            break;

        case LIGHT_STATE_STEADY_DISPLAY:
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
            }
            chalie_led_code(code);
            break;

        case LIGHT_STATE_ERROR_DISPLAY:
            if (!last_timer_it_state) {
                HAL_TIM_Base_Start_IT(&htim2);
                last_timer_it_state = 1;
            }
            if (HAL_GetTick() % 1000 >= 500) {
                chalie_led_code(7);
            } else {
                chalie_led_code(0);
            }
            break;
    }
}

/* 检查PRES状态 ----------------------------------------------------------*/
void CheckPRES(void) {
    pres_state = Operation_Status_BIN[31];
}

/* 检查零负载是否超时 ------------------------------------------------------*/
void CheckZeroLoad(void) {
    if (current > -10 && current < 10) {
        if (first_time_zero_load == 0) {
            first_time_zero_load = 1;
            zero_load_time = HAL_GetTick();
        } else if (HAL_GetTick() - zero_load_time > 10000) {
            zero_load_state = 1;
            first_time_zero_load = 0;
        }
    } else {
        zero_load_state = 0;
        first_time_zero_load = 0;
    }
}

/* 检查系统FET电源的开关状态 -------------------------------------------------*/
void CheckFet(void) {
    if (Manufacturing_Status_BIN[13] == 1 && Manufacturing_Status_BIN[14] == 1) {
        current_fet_state = FET_STATE_ON;
    } else {
        current_fet_state = FET_STATE_OFF;
    }
}

void Enter_Standby_Mode(void) {
    // 1. 使能PWR时钟
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 2. 配置唤醒源
    // 使能RTC唤醒
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_RTC);
    // 使能WKUP引脚唤醒（可选）
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    // 使能定时器唤醒中断
    HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
    */

    // 3. 清除所有唤醒标志
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    // 4. 进入待机模式
    // HAL_PWR_EnterSTANDBYMode();
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
