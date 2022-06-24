/*INCLUDE*/

#include "dashboard.h"
#include "as_fsm.h"
#include "button.h"
#include "can.h"
#include "mission.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"
#include "wdg.h"
#include <stdio.h>
/*EXTERNAL GLOBAL VARIABLES*/

extern char msg[80];
extern char value[60];

extern CAN_HandleTypeDef hcan;

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbox;
extern uint8_t TxData[8];
extern uint8_t RxData[8];

/*Error Variables*/
error_t error = ERROR_NONE;
uint8_t boards_timeouts;

extern bool ASB_ERR; // Autonomous System Brake
extern bool BMS_ERR;
extern bool NOHV; // TS Off
extern bool IMD_ERR;
extern bool TLB_ERR_RECEIVED;

bool COCK_BUTTON = false;
bool EXT_BUTTON = false;

/*PWM Variables*/
volatile uint8_t PWM_POWERTRAIN;
volatile uint8_t PWM_BAT_FAN;
volatile uint8_t PWM_ASB_MOTOR;

/*State Machine for RTD*/
typedef enum
{
    STATE_IDLE = 0,
    STATE_CTOR_EN_WAIT_ACK,
    STATE_TS_ON,
    STATE_RTD_WAIT_ACK,
    STATE_RTD,
    STATE_IDLE_WAIT_ACK,
    STATE_ERROR
} state;

typedef enum
{
    TRIG_NONE = 0,
    TRIG_COCK = 1,
    TRIG_EXT = 2
} state_trig;
state_trig STATE_CHANGE_TRIG = TRIG_NONE;

state rtd_fsm = STATE_IDLE;

/*RTD_FSM variables*/
extern bool CTOR_EN_ACK;
extern bool RTD_EN_ACK;
extern bool IDLE_ACK;
extern bool NACK;

/*Front brake pressure value*/
extern volatile uint16_t brake_pressure;

/*CUSTOM FUNCTIONS*/

/*Rx Message interrupt from CAN*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Transmission request Error */
        HAL_CAN_ResetError(hcan);
        Error_Handler();
    }

    // Reset watchdog
    uint32_t now = ReturnTime_100us();
    switch (RxHeader.StdId)
    {
    case ASB_CMD_ID_CAN:
    case AS_STATE_ID_CAN:
    case FSM_ACK_ID_CAN:
    case EBS_CMD_ID_CAN:
        // Reset dspace timeout after boot
        wdg_timeouts_100us[WDG_DSPACE] = 5000;
        wdg_reset(WDG_DSPACE, now);
        break;
    case TLB_ERROR_ID_CAN:
        wdg_reset(WDG_TLB, now);
        break;
    }

    /*Reboot Board - Received command byte from CAN*/
    if ((RxHeader.StdId == BOOTLOADER_ID_CAN) && (RxHeader.DLC == 2) && (RxData[0] == 0xFF) && (RxData[1] == 0x00))
    {
        NVIC_SystemReset();
    }
    /*
     *
     * dSpace
     *
     */
    /*EBS commmand*/
    else if ((RxHeader.StdId == EBS_CMD_ID_CAN) && (RxHeader.DLC == 1))
    {
        HAL_GPIO_WritePin(EBS_RELAY1_GPIO_Port, EBS_RELAY1_Pin, RxData[0] & 0b1);
        HAL_GPIO_WritePin(EBS_RELAY2_GPIO_Port, EBS_RELAY2_Pin, (RxData[0] >> 1) & 0b1);
    }
    /* ASB command */
    else if ((RxHeader.StdId == ASB_CMD_ID_CAN) && (RxHeader.DLC == 1))
    {
        if (RxData[0] <= 255)
        {
            // Min PWM pulse=800 (800us)
            // Max PWM pulse=2100 (2.1ms)
            PWM_ASB_MOTOR = RxData[0] * ((2100 - 800) / 255) + 800;
            __HAL_TIM_SET_COMPARE(&ASB_MOTOR_PWM_TIM, ASB_MOTOR_PWM_CH, PWM_ASB_MOTOR);
        }
        else
        {
            // TODO: handle ASB error?
        }
    }
    /* AS state */
    else if ((RxHeader.StdId == AS_STATE_ID_CAN) && (RxHeader.DLC == 2))
    {
        ASB_ERR = (bool)(RxData[0] & (1 << 7));
        if (RxData[1] < AS_TEST)
        {
            as_state = RxData[1];
        }
    }
    /* ACK from dSpace */
    else if ((RxHeader.StdId == FSM_ACK_ID_CAN) && (RxHeader.DLC == 1))
    {
        if ((RxData[0] == 1) && (rtd_fsm == STATE_CTOR_EN_WAIT_ACK))
        {
            CTOR_EN_ACK = true;
        }
        else if ((RxData[0] == 2) && (rtd_fsm == STATE_TS_ON || rtd_fsm == STATE_RTD_WAIT_ACK))
        {
            RTD_EN_ACK = true;
        }
        else if ((RxData[0] == 3) && (rtd_fsm == STATE_IDLE_WAIT_ACK || rtd_fsm == STATE_RTD))
        {
            IDLE_ACK = true;
        }
        else if ((RxData[0] == 4) && (rtd_fsm == STATE_CTOR_EN_WAIT_ACK || rtd_fsm == STATE_RTD_WAIT_ACK || rtd_fsm == STATE_IDLE_WAIT_ACK))
        {
            NACK = true;
        }
    }
    /* Cooling Command */
    else if ((RxHeader.StdId == PWM_CMD_ID_CAN) && (RxHeader.DLC == 2))
    {
        // Radiator fan and pump signals have been merged to make space for ASB signal.
        if (RxData[0] <= 100)
        {
            // TODO: map 0-255 to PWM values
            PWM_POWERTRAIN = RxData[0];
            __HAL_TIM_SET_COMPARE(&POWERTRAIN_COOLING_PWM_TIM, POWERTRAIN_COOLING_PWM_CH, PWM_POWERTRAIN);
        }
        else if (RxData[1] <= 100)
        {
            // TODO: map 0-255 to PWM values
            PWM_BAT_FAN = RxData[2];
            __HAL_TIM_SET_COMPARE(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH, PWM_BAT_FAN);
        }
    }
    /* Received TLB error byte in order to turn LEDs on or off */
    else if ((RxHeader.StdId == TLB_ERROR_ID_CAN) && (RxHeader.DLC == 1) && (RxData[0] < 16))
    {
        NOHV = (bool)(RxData[0] & 1);
        BMS_ERR = (bool)(RxData[0] & 4) || (bool)(RxData[0] & 2);
        IMD_ERR = (bool)(RxData[0] & 4);
        TLB_ERR_RECEIVED = true;
    }
}

void InitDashBoard()
{
    HAL_GPIO_WritePin(EBS_RELAY1_GPIO_Port, EBS_RELAY1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EBS_RELAY2_GPIO_Port, EBS_RELAY2_Pin, GPIO_PIN_RESET);

    // Turn on all LEDs
    HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ASB_CMD_GPIO_Port, ASB_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, GPIO_PIN_SET);

    mission_set(MISSION_NO);
    mission_run();

    as_state = AS_TEST;
    as_run();

    HAL_Delay(900);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_RESET);

    // Test input states
    if (button_get(BUTTON_COCK) ||
        button_get(BUTTON_EXT) ||
        button_get(BUTTON_MISSION))
    {
        error = ERROR_INIT_BTN;
        rtd_fsm = STATE_ERROR;
    }

    as_state = AS_OFF;
    mission_set(MISSION_ACCEL);
}

void cock_callback()
{
    if (rtd_fsm == STATE_IDLE || rtd_fsm == STATE_TS_ON || rtd_fsm == STATE_RTD)
    {
        COCK_BUTTON = true;
    }
}

void ext_callback()
{
    if (rtd_fsm == STATE_IDLE)
    {
        EXT_BUTTON = true;
    }
}

/*FSM*/
void ReadyToDriveFSM(uint32_t delay_100us)
{
    // Set first delay back to give time for the buttons to debounce
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        // static int counter_buzzer = 0;

        switch (rtd_fsm)
        {
        case STATE_IDLE:
            if (EXT_BUTTON)
            {
                EXT_BUTTON = false;
                STATE_CHANGE_TRIG = TRIG_EXT;
                rtd_fsm = STATE_CTOR_EN_WAIT_ACK;
            }
            else if (COCK_BUTTON)
            {
                COCK_BUTTON = false;
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_CTOR_EN_WAIT_ACK;
            }
            else
            {
                HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, OFF);
                // TxHeader.StdId = DASH_RTD_ID_CAN;
                // TxHeader.RTR = CAN_RTR_DATA;
                // TxHeader.IDE = CAN_ID_STD;
                // TxHeader.DLC = 1;
                // TxHeader.TransmitGlobalTime = DISABLE;
                // TxData[0] = 0x0;
                // CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_CTOR_EN_WAIT_ACK:

            if (NACK)
            {
                NACK = false;
                CTOR_EN_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            else if (CTOR_EN_ACK)
            {
                CTOR_EN_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_TS_ON;
            }
            else
            {
                // TxHeader.StdId = DASH_RTD_ID_CAN;
                // TxHeader.RTR = CAN_RTR_DATA;
                // TxHeader.IDE = CAN_ID_STD;
                // TxHeader.DLC = 1;
                // TxHeader.TransmitGlobalTime = DISABLE;
                // TxData[0] = 0x1;
                // CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_TS_ON:

            if (RTD_EN_ACK)
            {
                RTD_EN_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_RTD;
            }
            else if (COCK_BUTTON)
            {
                COCK_BUTTON = false;
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_RTD_WAIT_ACK;
            }
            else
            {
                // TxHeader.StdId = DASH_RTD_ID_CAN;
                // TxHeader.RTR = CAN_RTR_DATA;
                // TxHeader.IDE = CAN_ID_STD;
                // TxHeader.DLC = 1;
                // TxHeader.TransmitGlobalTime = DISABLE;
                // TxData[0] = 0x1;
                // CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_RTD_WAIT_ACK:
            if (NACK || IDLE_ACK)
            {
                NACK = false;
                IDLE_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            else if (RTD_EN_ACK)
            {
                RTD_EN_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_RTD;
            }
            else
            {
                // TxHeader.StdId = DASH_RTD_ID_CAN;
                // TxHeader.RTR = CAN_RTR_DATA;
                // TxHeader.IDE = CAN_ID_STD;
                // TxHeader.DLC = 1;
                // TxHeader.TransmitGlobalTime = DISABLE;
                // TxData[0] = 0x2;
                // CAN_Msg_Send(&hcan, &TxHeader, TxData, &TxMailbox, 30);
            }
            break;

        case STATE_RTD:
            // TODO: buzzer
            if (IDLE_ACK)
            {
                IDLE_ACK = false;
                rtd_fsm = STATE_IDLE;
            }
            else if (COCK_BUTTON)
            {
                COCK_BUTTON = false;
                STATE_CHANGE_TRIG = TRIG_COCK;
                rtd_fsm = STATE_IDLE_WAIT_ACK;
            }
            HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            // if (counter_buzzer < 40)
            //{
            //     counter_buzzer++;
            //     HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, ON);
            //     HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, ON);
            //     HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            // }
            // else
            //{
            //     HAL_GPIO_WritePin(BUZZEREV_CMD_GPIO_Port, BUZZEREV_CMD_Pin, OFF);
            //     HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, OFF);
            //      HAL_GPIO_WritePin(RTD_CMD_GPIO_Port, RTD_CMD_Pin, ON);
            //     counter_buzzer = 0;
            //     rtd_fsm = STATE_IDLE;
            // }
            break;

        case STATE_IDLE_WAIT_ACK:
            if (NACK)
            {
                NACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_RTD;
            }
            else if (IDLE_ACK)
            {
                IDLE_ACK = false;
                STATE_CHANGE_TRIG = TRIG_NONE;
                rtd_fsm = STATE_IDLE;
            }
            break;

        case STATE_ERROR:
            // TODO: uncomment
            // Engineers around the world are thankful this line is commented
            // as_state = AS_EMERGENCY;
            break;
        }
    }
}

/*Update Cockpit's LEDs*/
void UpdateCockpitLed(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        HAL_GPIO_WritePin(ASB_CMD_GPIO_Port, ASB_CMD_Pin, ASB_ERR);

        if (TLB_ERR_RECEIVED)
        {
            TLB_ERR_RECEIVED = false;
            HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, BMS_ERR);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, NOHV);
            HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, IMD_ERR);
            /*LED ON or OFF depending on ERR_RECEIVED*/
        }
        else
        {
            HAL_GPIO_WritePin(AMS_CMD_GPIO_Port, AMS_CMD_Pin, ON);
            HAL_GPIO_WritePin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin, OFF);
            HAL_GPIO_WritePin(IMD_CMD_GPIO_Port, IMD_CMD_Pin, ON);
            /*LED always ON, timeout can*/
        }
    }
}

/*Setup TIMER, CAN*/
void SetupDashBoard(void)
{

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&POWERTRAIN_COOLING_PWM_TIM, POWERTRAIN_COOLING_PWM_CH) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    /*Start timer for PWM*/
    if (HAL_TIM_PWM_Start(&BAT_FAN_PWM_TIM, BAT_FAN_PWM_CH) != HAL_OK)
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

    mission_setup();
    button_set_shortpress(BUTTON_COCK, cock_callback);
    button_set_shortpress(BUTTON_EXT, ext_callback);

    sprintf(msg, "Dashboard 2022 Boot - build %s @ %s\n\r", __DATE__, __TIME__);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);
    memset(msg, 0, strlen(msg));
    sprintf(msg, "Configuration complete\n\r\n\r");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);

#ifdef DEBUG
    memset(msg, 0, strlen(msg));
    sprintf(msg, "Debug Mode = ON\n\r\n\r");
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 30, 20);
#endif

    /*BootScreen with verion of the code and message of complete configuration*/
}

/*Send status data to CAN BUS*/
void can_send_state(uint32_t delay_100us)
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, delay_100us))
    {
        TxHeader.StdId = DASH_STATUS_ID_CAN;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = 4;
        TxHeader.TransmitGlobalTime = DISABLE;
        TxData[0] = rtd_fsm;
        TxData[1] = mission_is_confirmed() ? mission_get() : MISSION_NO;
        TxData[2] = (HAL_GPIO_ReadPin(AMS_CMD_GPIO_Port, AMS_CMD_Pin)) |
                    (HAL_GPIO_ReadPin(TSOFF_CMD_GPIO_Port, TSOFF_CMD_Pin) << 1) |
                    (HAL_GPIO_ReadPin(IMD_CMD_GPIO_Port, IMD_CMD_Pin) << 2) |
                    (HAL_GPIO_ReadPin(RTD_CMD_GPIO_Port, RTD_CMD_Pin) << 3) |
                    (HAL_GPIO_ReadPin(ASB_CMD_GPIO_Port, ASB_CMD_Pin) << 4);
        TxData[3] = button_get(BUTTON_COCK) | button_get(BUTTON_EXT) << 1 | button_get(BUTTON_MISSION) << 2 | STATE_CHANGE_TRIG << 3;

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
    LedBlinking(LED2_GPIO_Port, LED2_Pin, &led_blink, 2000);

    // Update state Cockpit's LEDs
    UpdateCockpitLed(1000);

    // Update buttons state
    button_sample();

    // RUN the ready to drive FSM
    ReadyToDriveFSM(500);

    // Run the AS FSM
    mission_run();
    as_run();

    uint8_t timeouts = wdg_check();
    if (rtd_fsm != STATE_ERROR && timeouts != 0)
    {
        error = ERROR_CAN_WDG;
        boards_timeouts = timeouts;
        rtd_fsm = STATE_ERROR;
    }

    // Send current state via CAN
    can_send_state(500);
}
