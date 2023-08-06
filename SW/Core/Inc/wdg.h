/**
 * @file wdg.h
 * @author Matteo Bonora (matteo.bonora@studenti.polito.it)
 * @brief Functions to check the communications of other boards on the CAN bus
 * @date 2022-06-15
 */

#pragma once
#include <inttypes.h>

/**
 * @brief Watchdog board indexes
 */
typedef enum
{
    WDG_BOARD_DSPACE = 0,
    WDG_BOARD_TLB = 1,
    WDG_NUM_BOARDS
} wdg_boards;

/**
 * @brief Timeout values for each board
 */
static uint32_t wdg_timeouts_ms[WDG_NUM_BOARDS] = {
    [WDG_BOARD_DSPACE] = UINT32_MAX, // Wait for dSpace to boot
    [WDG_BOARD_TLB] = 2000};

/**
 * @brief Resets the timer of a board to the given timestamp
 *
 * @param board The board to reset
 * @param timestamp_ms The timestamp to set
 */
void wdg_reset(wdg_boards board, uint32_t timestamp_ms);

/**
 * @brief Checks for timeouts
 *
 * @return uint8_t bitset that holds the timeout state for each board
 */
uint8_t wdg_check();
