#include "as_fsm.h"
#include "main.h"
#include "utils.h"

#define PERIOD_4HZ_100us 1250 // Duty cycle time to get 4Hz total period (toggle at 8Hz)
as_state_t as_state = AS_OFF;

void as_run()
{
    static uint32_t delay_100us_last = 0;

    if (delay_fun(&delay_100us_last, PERIOD_4HZ_100us))
    {
        switch (as_state)
        {
        case AS_OFF:
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
            break;
        case AS_READY:
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_RESET);
            break;
        case AS_DRIVING:
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_TogglePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin); // Note: This will blink at the frequency of the FSM
            break;
        case AS_FINISHED:
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
            break;
        case AS_EMERGENCY:
            HAL_GPIO_TogglePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BUZZERAS_CMD_GPIO_Port, BUZZERAS_CMD_Pin, GPIO_PIN_SET);
            break;
        case AS_TEST:
            // Both lights on
            HAL_GPIO_WritePin(ASSI_BLUE_CMD_GPIO_Port, ASSI_BLUE_CMD_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ASSI_YELLOW_CMD_GPIO_Port, ASSI_YELLOW_CMD_Pin, GPIO_PIN_RESET);
            break;
        }
    }
}