/*INCLUDE*/

#include "dashboard.h"
#include "button.h"
#include "can.h"
#include "sdc_can_pwt_db_v1_0.h"
#include "tim.h"
#include "tlb_battery.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"
#include <stdio.h>

/* Dashboard State Machine */

/* State change triggers */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailbox;
uint8_t TxData[8] = {0};
uint8_t RxData[8] = {0};

/* Error Variables */
error_t error = ERROR_NONE;
uint8_t boards_timeouts = 0;

/* Cock LEDs flags */
volatile bool ASB_ERR = 1;
volatile bool BMS_ERR = 1;
volatile bool TSOFF = 0;
volatile bool IMD_ERR = 1;
volatile bool RTD_CMD = 1;

bool RTD_EN_ACK = 0;
bool EMERGENCY_ACK = 0;
bool AS_DRIVING_ACK = 0;

/* Button short press flags */
bool COCK_BUTTON = false;
bool EXT_BUTTON = false;

/*CUSTOM FUNCTIONS*/
void handle_can(CAN_HandleTypeDef *hcan);

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    handle_can(hcan);
}

void handle_can(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        // Error_Handler();
    }

    // Reset watchdog
    uint32_t now = HAL_GetTick();
    switch (RxHeader.StdId)
    {
    case EBS_CMD_ID_CAN:
    case BUZZER_CMD_ID_CAN:
    case AS_STATE_ID_CAN:
    case PWM_CMD_ID_CAN:
        // Reset dSpace timeout after boot
        wdg_timeouts_ms[WDG_BOARD_DSPACE] = 500;
        wdg_reset(WDG_BOARD_DSPACE, now);
        break;
    case TLB_BATTERY_TLB_BAT_INTRNL_FUNC_FRAME_ID:
        wdg_timeouts_ms[WDG_BOARD_TLB] = 500;
        wdg_reset(WDG_BOARD_TLB, now);
        break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((RxHeader.StdId == BOOTLOADER_RX_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
    {
        NVIC_SystemReset();
    }

    /*
     *
     * dSpace
     *
     */
    /* EBS commmand */
    else if ((RxHeader.StdId == EBS_CMD_ID_CAN) && (RxHeader.DLC == 1))
    {
        HAL_GPIO_WritePin(EBS_RELAY1_CMD_GPIO_Port, EBS_RELAY1_CMD_Pin, RxData[0] & 0b1);
        HAL_GPIO_WritePin(EBS_RELAY2_CMD_GPIO_Port, EBS_RELAY2_CMD_Pin, RxData[0] & 0b1);
        // HAL_GPIO_WritePin(EBS_RELAY2_CMD_GPIO_Port, EBS_RELAY2_CMD_Pin, (RxData[0] >> 1) & 0b1);
    }
    /* ASB command */
    else if (RxHeader.StdId == BUZZER_CMD_ID_CAN)
    {
        static uint8_t old_state = 0;

        if (old_state != RxData[0])
            switch (RxData[0])
            {
            case 0:
                RTD_CMD = 0;
                break;
            case 1:
                RTD_CMD = 1;
                RTD_EN_ACK = true;
                break;
            case 2:
                RTD_CMD = 1;
                AS_DRIVING_ACK = true;
                break;
            case 3:
                RTD_CMD = 0;
                EMERGENCY_ACK = true;
                break;
            }
        old_state = RxData[0];
        // Min PWM pulse=800 (800us)
        // Max PWM pulse=2100 (2.1ms)
        // PWM_ASB_MOTOR = RxData[0] * ((2100 - 800) / 255) + 800;
        //__HAL_TIM_SET_COMPARE(&ASB_MOTOR_PWM_TIM, ASB_MOTOR_PWM_CH, PWM_ASB_MOTOR);
    }
    /* AS state */
    else if ((RxHeader.StdId == AS_STATE_ID_CAN))
    {
        ASB_ERR = (bool)(RxData[0] & 0b1);
    }
    /* Cooling Command */
    else if (RxHeader.StdId == PWM_CMD_ID_CAN)
    {
        // Radiator fan and pump signals have been merged to make space for ASB signal.
        //__HAL_TIM_SET_COMPARE(&INVERTER_PUMP_PWM_TIM, INVERTER_PUMP_PWM_CH, RxData[0]);

        __HAL_TIM_SET_COMPARE(&BP_FAN_PWM_TIM, BP_FAN_PWM_CH, RxData[1]);
    }

    /*
     *
     * TLB
     *
     */
    /* Received TLB error byte in order to turn LEDs on or off */
    else if ((RxHeader.StdId == TLB_BATTERY_TLB_BAT_INTRNL_FUNC_FRAME_ID) && (RxHeader.DLC == TLB_BATTERY_TLB_BAT_INTRNL_FUNC_LENGTH))
    {
        struct tlb_battery_tlb_bat_intrnl_func_t status;
        tlb_battery_tlb_bat_intrnl_func_unpack(&status, RxData, RxHeader.DLC);

        TSOFF = tlb_battery_tlb_bat_intrnl_func_tsal_green_en_decode(status.tsal_green_en);
        BMS_ERR = tlb_battery_tlb_bat_intrnl_func_ams_err_ltch_decode(status.ams_err_ltch);
        IMD_ERR = tlb_battery_tlb_bat_intrnl_func_imd_err_ltch_decode(status.imd_err_ltch);
    }
}

void InitDashBoard()
{
    HAL_GPIO_WritePin(EBS_RELAY1_CMD_GPIO_Port, EBS_RELAY1_CMD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EBS_RELAY2_CMD_GPIO_Port, EBS_RELAY2_CMD_Pin, GPIO_PIN_RESET);

    // Turn on all LEDs
    HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ASB_ERR_CMD_GPIO_Port, ASB_ERR_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, GPIO_PIN_SET);

    HAL_Delay(900);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);

    // Test inputs
    if (button_get(BUTTON_COCK) ||
        button_get(BUTTON_EXT) ||
        button_get(BUTTON_MISSION))
    {
        error = ERROR_INIT_BTN;
    }
}

void cock_callback()
{
    COCK_BUTTON = true;
}

void ext_callback()
{
    EXT_BUTTON = true;
}

/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        HAL_GPIO_WritePin(ASB_ERR_CMD_GPIO_Port, ASB_ERR_CMD_Pin, ASB_ERR);

        if ((boards_timeouts >> WDG_BOARD_DSPACE) & 0b1)
        {
            HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
        }
        if ((boards_timeouts >> WDG_BOARD_TLB) & 0b1)
        {
            HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, ON);
            HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, ON);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, OFF);
        }
        else
        {
            HAL_GPIO_WritePin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin, BMS_ERR);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, TSOFF);
            HAL_GPIO_WritePin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin, IMD_ERR);
            HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, RTD_CMD);
        }
    }
}

/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&INVERTER_PUMP_PWM_TIM, INVERTER_PUMP_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&BP_FAN_PWM_TIM, BP_FAN_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&ASB_MOTOR_PWM_TIM, ASB_MOTOR_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    // button_set_shortpress_callback(BUTTON_COCK, cock_callback);
    // button_set_shortpress_callback(BUTTON_EXT, ext_callback);

    char msg[54] = {0};
    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\n\r", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 38, 20);
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_ms)
{
    static uint32_t delay_ms_last = 0;

    if (delay_fun(&delay_ms_last, delay_ms))
    {
        TxHeader.StdId = DASH_STATUS_ID_CAN;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 4;
        TxHeader.TransmitGlobalTime = DISABLE;
        TxData[0] = 0;
        TxData[1] = 0;
        TxData[2] = (HAL_GPIO_ReadPin(AMS_ERR_CMD_GPIO_Port, AMS_ERR_CMD_Pin)) |
                    (HAL_GPIO_ReadPin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin) << 1) |
                    (HAL_GPIO_ReadPin(IMD_ERR_CMD_GPIO_Port, IMD_ERR_CMD_Pin) << 2) |
                    (HAL_GPIO_ReadPin(RTD_CMD_GPIO_Port, RTD_CMD_Pin) << 3) |
                    (HAL_GPIO_ReadPin(ASB_ERR_CMD_GPIO_Port, ASB_ERR_CMD_Pin) << 4);
        TxData[3] = button_get(BUTTON_COCK) | button_get(BUTTON_EXT) << 1 | button_get(BUTTON_MISSION) << 2;

        CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);

        // Error state
        TxHeader.StdId = DASH_ERR_ID_CAN;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 2;
        TxHeader.TransmitGlobalTime = DISABLE;
        TxData[0] = error;
        TxData[1] = 0;
        switch (error)
        {
        case ERROR_CAN_WDG:
            TxData[1] = boards_timeouts;
            break;
        default:
            break;
        }
        CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
    }
}

/**
 * @brief Dash main loop
 */
void CoreDashBoard(void)
{
    // Blink green led to signal activity
    static uint32_t led_blink = 0;
    static uint32_t buzzer_counter = 0;
    static uint32_t timer = 0;

    LedBlinking(LED2_GPIO_Port, LED2_Pin, &led_blink, 200);

    // Update state Cockpit's LEDs
    UpdateCockpitLed(100);

    // Update buttons state
    button_sample();

    uint8_t timeouts = wdg_check();
    if (timeouts != 0)
    {
        error = ERROR_CAN_WDG;
        boards_timeouts = timeouts;
        // if ((boards_timeouts >> WDG_BOARD_DSPACE) & 0b1)
        //{
        //     EMERGENCY_ACK = 1;
        // }
    }
    else
    {
        error = ERROR_NONE;
        boards_timeouts = 0;
    }

    if (HAL_GetTick() - timer >= 10)
    {
        static uint32_t kek = 0;
        timer = HAL_GetTick();

        if (EMERGENCY_ACK && buzzer_counter < 800)
        {
            buzzer_counter++;
            if (timer - kek >= 125)
            {
                kek = timer;
                HAL_GPIO_TogglePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin);
                HAL_GPIO_TogglePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin);
            }
        }
        else if (AS_DRIVING_ACK && buzzer_counter < 200)
        {
            buzzer_counter++;
            HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);
        }
        else if (RTD_EN_ACK && buzzer_counter < 200)
        {
            buzzer_counter++;
            HAL_GPIO_TogglePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin);
            HAL_GPIO_TogglePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin);
        }
        else
        {
            HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);
            AS_DRIVING_ACK = 0;
            RTD_EN_ACK = 0;
            EMERGENCY_ACK = 0;
        }
    }

    // Send current state via CAN
    can_send_state(10);
}
