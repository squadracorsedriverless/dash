#include "utils.h"

// Like systick, but with a resolution of 100us
volatile uint32_t counter;

/**
 * @brief Blink a given pin at the specified frequency
 *
 * @param GPIOx GPIO bus of the pin
 * @param GPIO_Pin Pin number
 * @param last Last toggle time
 * @param period Toggling period
 */
void LedBlinking(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *last, uint32_t period)
{
    if (delay_fun(last, period))
    {
        HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
    }
}

/**
 * @brief Returns true if a given time has passed
 *
 * @param delay_100us_last Starting time
 * @param delay_100us Desired period
 * @return true Period has elapsed
 * @return false Period has not elapsed
 */
bool delay_fun(uint32_t *delay_ms_last, uint32_t delay_ms)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time > *delay_ms_last && (current_time - *delay_ms_last) >= delay_ms)
    {
        *delay_ms_last = current_time;
        return true;
    }
    else if (current_time < *delay_ms_last && (UINT32_MAX - current_time - *delay_ms_last) >= delay_ms)
    {
        *delay_ms_last = current_time;
        return true;
    }
    /*In case of timer overflow, the delay is computed correctly*/

    return false;
}
