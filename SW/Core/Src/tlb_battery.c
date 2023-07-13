/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 38.0.2 Thu Jul 13 18:24:12 2023.
 */

#include <string.h>

#include "tlb_battery.h"

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

int tlb_battery_shtdwn_line_tsac_status_pack(
    uint8_t *dst_p,
    const struct tlb_battery_shtdwn_line_tsac_status_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 3);

    dst_p[0] |= pack_left_shift_u8(src_p->shtdwn_tsac_init_sec_in_active_lvl, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u8(src_p->shtdwn_post_ams_err_rly_active_lvl, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->shtdwn_post_imd_err_rly_active_lvl, 2u, 0x04u);
    dst_p[0] |= pack_left_shift_u8(src_p->shtdwn_tsac_fin_sec_in_active_lvl, 3u, 0x08u);
    dst_p[1] |= pack_left_shift_u8(src_p->shtdwn_post_prch_resist_voltage_lvl, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->shtdwn_post_dly_caps_voltage_lvl, 0u, 0xffu);

    return (3);
}

int tlb_battery_shtdwn_line_tsac_status_unpack(
    struct tlb_battery_shtdwn_line_tsac_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    dst_p->shtdwn_tsac_init_sec_in_active_lvl = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->shtdwn_post_ams_err_rly_active_lvl = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->shtdwn_post_imd_err_rly_active_lvl = unpack_right_shift_u8(src_p[0], 2u, 0x04u);
    dst_p->shtdwn_tsac_fin_sec_in_active_lvl = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->shtdwn_post_prch_resist_voltage_lvl = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->shtdwn_post_dly_caps_voltage_lvl = unpack_right_shift_u8(src_p[2], 0u, 0xffu);

    return (0);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_init_sec_in_active_lvl_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_init_sec_in_active_lvl_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_init_sec_in_active_lvl_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_post_ams_err_rly_active_lvl_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_post_ams_err_rly_active_lvl_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_post_ams_err_rly_active_lvl_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_post_imd_err_rly_active_lvl_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_post_imd_err_rly_active_lvl_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_post_imd_err_rly_active_lvl_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_fin_sec_in_active_lvl_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_fin_sec_in_active_lvl_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_tsac_fin_sec_in_active_lvl_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_post_prch_resist_voltage_lvl_encode(float value)
{
    return (uint8_t)(value / 0.0977f);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_post_prch_resist_voltage_lvl_decode(uint8_t value)
{
    return ((float)value * 0.0977f);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_post_prch_resist_voltage_lvl_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t tlb_battery_shtdwn_line_tsac_status_shtdwn_post_dly_caps_voltage_lvl_encode(float value)
{
    return (uint8_t)(value / 0.0977f);
}

float tlb_battery_shtdwn_line_tsac_status_shtdwn_post_dly_caps_voltage_lvl_decode(uint8_t value)
{
    return ((float)value * 0.0977f);
}

bool tlb_battery_shtdwn_line_tsac_status_shtdwn_post_dly_caps_voltage_lvl_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int tlb_battery_tlb_bat_intrnl_func_pack(
    uint8_t *dst_p,
    const struct tlb_battery_tlb_bat_intrnl_func_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 3);

    dst_p[0] |= pack_left_shift_u8(src_p->ams_err_ltch, 0u, 0x01u);
    dst_p[0] |= pack_left_shift_u8(src_p->imd_err_ltch, 1u, 0x02u);
    dst_p[0] |= pack_left_shift_u8(src_p->shtdwn_prch_rly_en, 2u, 0x04u);
    dst_p[0] |= pack_left_shift_u8(src_p->shrt2_gnd_air_neg, 3u, 0x08u);
    dst_p[0] |= pack_left_shift_u8(src_p->shrt2_gnd_air_pos, 4u, 0x10u);
    dst_p[0] |= pack_left_shift_u8(src_p->shrt2_gnd_air, 5u, 0x20u);
    dst_p[0] |= pack_left_shift_u8(src_p->dc_bus_vehicle_side_over60_v, 6u, 0x40u);
    dst_p[0] |= pack_left_shift_u8(src_p->air_neg_clsd_inten_state, 7u, 0x80u);
    dst_p[1] |= pack_left_shift_u8(src_p->air_pos_clsd_inten_state, 0u, 0x01u);
    dst_p[1] |= pack_left_shift_u8(src_p->dc_bus_prch_rly_en_inten_state, 1u, 0x02u);
    dst_p[1] |= pack_left_shift_u8(src_p->air_neg_clsd_aux_state, 2u, 0x04u);
    dst_p[1] |= pack_left_shift_u8(src_p->air_pos_clsd_aux_state, 3u, 0x08u);
    dst_p[1] |= pack_left_shift_u8(src_p->dc_bus_prch_rly_en_aux_state, 4u, 0x10u);
    dst_p[1] |= pack_left_shift_u8(src_p->air_neg_clsd_imp_err, 5u, 0x20u);
    dst_p[1] |= pack_left_shift_u8(src_p->air_pos_clsd_imp_err, 6u, 0x40u);
    dst_p[1] |= pack_left_shift_u8(src_p->dc_bus_prch_rly_en_imp_err, 7u, 0x80u);
    dst_p[2] |= pack_left_shift_u8(src_p->dc_bus_veh_side_over60_v_imp_err, 0u, 0x01u);
    dst_p[2] |= pack_left_shift_u8(src_p->any_imp_err, 1u, 0x02u);
    dst_p[2] |= pack_left_shift_u8(src_p->any_imp_err_ltch, 2u, 0x04u);
    dst_p[2] |= pack_left_shift_u8(src_p->tsal_green_en, 3u, 0x08u);

    return (3);
}

int tlb_battery_tlb_bat_intrnl_func_unpack(
    struct tlb_battery_tlb_bat_intrnl_func_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 3u) {
        return (-EINVAL);
    }

    dst_p->ams_err_ltch = unpack_right_shift_u8(src_p[0], 0u, 0x01u);
    dst_p->imd_err_ltch = unpack_right_shift_u8(src_p[0], 1u, 0x02u);
    dst_p->shtdwn_prch_rly_en = unpack_right_shift_u8(src_p[0], 2u, 0x04u);
    dst_p->shrt2_gnd_air_neg = unpack_right_shift_u8(src_p[0], 3u, 0x08u);
    dst_p->shrt2_gnd_air_pos = unpack_right_shift_u8(src_p[0], 4u, 0x10u);
    dst_p->shrt2_gnd_air = unpack_right_shift_u8(src_p[0], 5u, 0x20u);
    dst_p->dc_bus_vehicle_side_over60_v = unpack_right_shift_u8(src_p[0], 6u, 0x40u);
    dst_p->air_neg_clsd_inten_state = unpack_right_shift_u8(src_p[0], 7u, 0x80u);
    dst_p->air_pos_clsd_inten_state = unpack_right_shift_u8(src_p[1], 0u, 0x01u);
    dst_p->dc_bus_prch_rly_en_inten_state = unpack_right_shift_u8(src_p[1], 1u, 0x02u);
    dst_p->air_neg_clsd_aux_state = unpack_right_shift_u8(src_p[1], 2u, 0x04u);
    dst_p->air_pos_clsd_aux_state = unpack_right_shift_u8(src_p[1], 3u, 0x08u);
    dst_p->dc_bus_prch_rly_en_aux_state = unpack_right_shift_u8(src_p[1], 4u, 0x10u);
    dst_p->air_neg_clsd_imp_err = unpack_right_shift_u8(src_p[1], 5u, 0x20u);
    dst_p->air_pos_clsd_imp_err = unpack_right_shift_u8(src_p[1], 6u, 0x40u);
    dst_p->dc_bus_prch_rly_en_imp_err = unpack_right_shift_u8(src_p[1], 7u, 0x80u);
    dst_p->dc_bus_veh_side_over60_v_imp_err = unpack_right_shift_u8(src_p[2], 0u, 0x01u);
    dst_p->any_imp_err = unpack_right_shift_u8(src_p[2], 1u, 0x02u);
    dst_p->any_imp_err_ltch = unpack_right_shift_u8(src_p[2], 2u, 0x04u);
    dst_p->tsal_green_en = unpack_right_shift_u8(src_p[2], 3u, 0x08u);

    return (0);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_ams_err_ltch_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_ams_err_ltch_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_ams_err_ltch_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_imd_err_ltch_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_imd_err_ltch_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_imd_err_ltch_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_shtdwn_prch_rly_en_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_shtdwn_prch_rly_en_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_shtdwn_prch_rly_en_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_neg_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_neg_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_neg_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_pos_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_pos_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_pos_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_shrt2_gnd_air_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_dc_bus_vehicle_side_over60_v_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_dc_bus_vehicle_side_over60_v_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_dc_bus_vehicle_side_over60_v_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_inten_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_inten_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_inten_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_inten_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_inten_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_inten_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_inten_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_inten_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_inten_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_aux_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_aux_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_aux_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_aux_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_aux_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_aux_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_aux_state_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_aux_state_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_aux_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_imp_err_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_imp_err_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_neg_clsd_imp_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_imp_err_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_imp_err_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_air_pos_clsd_imp_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_imp_err_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_imp_err_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_dc_bus_prch_rly_en_imp_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_dc_bus_veh_side_over60_v_imp_err_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_dc_bus_veh_side_over60_v_imp_err_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_dc_bus_veh_side_over60_v_imp_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_any_imp_err_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_any_imp_err_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_any_imp_err_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_any_imp_err_ltch_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_any_imp_err_ltch_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_any_imp_err_ltch_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t tlb_battery_tlb_bat_intrnl_func_tsal_green_en_encode(float value)
{
    return (uint8_t)(value);
}

float tlb_battery_tlb_bat_intrnl_func_tsal_green_en_decode(uint8_t value)
{
    return ((float)value);
}

bool tlb_battery_tlb_bat_intrnl_func_tsal_green_en_is_in_range(uint8_t value)
{
    return (value <= 1u);
}
