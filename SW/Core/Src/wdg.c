#include "wdg.h"
#include "utils.h"

uint32_t wdg_timestamps_ms[WDG_NUM_BOARDS];

void wdg_reset(wdg_boards board, uint32_t timestamp_ms)
{
    wdg_timestamps_ms[board] = timestamp_ms;
}

uint8_t wdg_check()
{
    uint8_t boards = 0;

    for (uint8_t iboard = 0; iboard < WDG_NUM_BOARDS; iboard++)
    {
        if (HAL_GetTick() - wdg_timestamps_ms[iboard] > wdg_timeouts_ms[iboard])
        {
            boards |= 1 << iboard;
        }
    }
    return boards;
}