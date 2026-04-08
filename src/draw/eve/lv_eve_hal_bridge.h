/**
 * @file lv_eve_hal_bridge.h
 *
 * Bridge between the EVE HAL (from EVE Screen Designer / eve_hal)
 * and Rudolph Riedel's FT800-FT813 library. Provides an op_cb
 * implementation that forwards the byte-level SPI/GPIO operations
 * used by the FT800-FT813 library through the EVE HAL's
 * transfer API.
 *
 * Guarded by LV_USE_EVE5 because that implies eve_hal is available.
 *
 * Copyright (C) 2025-2026  Bridgetek Pte Ltd
 * Author: Jan Boon <jan.boon@kaetemi.be>
 * SPDX-License-Identifier: MIT
 */
#ifndef LV_EVE_HAL_BRIDGE_H
#define LV_EVE_HAL_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "../../lv_conf_internal.h"

#if LV_USE_EVE5 && LV_USE_DRAW_EVE

#include "../../display/lv_display.h"
#include "lv_draw_eve_target.h"
#include "EVE_Hal.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the bridge and read display timing from the EVE HAL.
 *
 * Stores the HAL context for use by the op_cb. The EVE chip must
 * already be booted and configured via eve_hal.
 *
 * @param phost  pointer to an initialized EVE HAL context
 * @param params output: populated with current display timing
 *               read from EVE registers (pass to
 *               lv_draw_eve_display_create or
 *               lv_draw_eve_set_display_data)
 */
void lv_eve_hal_bridge_init(EVE_HalContext *phost, lv_draw_eve_parameters_t *params);

/**
 * The op_cb to pass to lv_draw_eve_display_create or
 * lv_draw_eve_set_display_data. Call lv_eve_hal_bridge_init first.
 */
void lv_eve_hal_bridge_op_cb(lv_display_t *disp, lv_draw_eve_operation_t operation,
                             void *data, uint32_t length);

#endif /* LV_USE_EVE5 && LV_USE_DRAW_EVE */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* LV_EVE_HAL_BRIDGE_H */
