/**
 * @file lv_eve_hal_bridge.c
 *
 * Bridge between the EVE HAL (from EVE Screen Designer / eve_hal)
 * and Rudolph Riedel's FT800-FT813 library. Implements an op_cb
 * that reconstructs EVE HAL transfer calls from the byte-level
 * SPI protocol used by the FT800-FT813 library.
 *
 * The FT800-FT813 library's SPI protocol is:
 *   CS_ASSERT -> spi_transmit(addr[23:16]) -> spi_transmit(addr[15:8])
 *   -> spi_transmit(addr[7:0]) -> [dummy byte for reads] -> payload -> CS_DEASSERT
 *
 * The EVE HAL combines CS assert + address + dummy into
 * EVE_Hal_startTransfer(), so we buffer the first 3 address bytes
 * after CS_ASSERT and call startTransfer lazily once the address
 * is complete.
 *
 * Flow control:
 * - Before each CMDB burst, waits for at least half the command
 *   FIFO to be free (the FT800-FT813 library has no flow control).
 * - After each CMDB burst, restores eve_hal's internal cmd state.
 * - Monitors REG_CMD_DL and voids CMDB writes when the display
 *   list is nearly full, letting only CMD_SWAP through so the
 *   frame still presents.
 *
 * Copyright (C) 2025-2026  Bridgetek Pte Ltd
 * Author: Jan Boon <jan.boon@kaetemi.be>
 * SPDX-License-Identifier: MIT
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_eve_hal_bridge.h"

#if LV_USE_EVE5 && LV_USE_DRAW_EVE

#include "lv_draw_eve.h"
#include "EVE_Cmd.h"

/*********************
 *      DEFINES
 *********************/

/* Start voiding CMDB writes when fewer than this many display list
 * entries remain. 128 entries = 512 bytes of DL space. */
#define DL_VOID_THRESHOLD 128

/* Estimated DL entries generated per CMDB word (worst case).
 * Widget commands like CMD_BUTTON can expand significantly. */
#define DL_ENTRIES_PER_CMDB_WORD 8

/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
    EVE_HalContext *phost;
    uint8_t addr_buf[3];
    uint8_t addr_idx;
    bool transfer_started;
    bool is_cmdb_write;      /* transfer is writing to REG_CMDB_WRITE */
    bool dl_voiding;         /* display list nearly full, voiding CMDB payload */
    bool dl_check_pending;   /* estimate hit threshold, check at next cmd boundary */
    uint16_t dl_estimate;    /* estimated DL entries used (reset on actual check or CMD_DLSTART) */
    uint8_t word_buf[4];     /* accumulator for 4-byte CMDB command detection */
    uint8_t word_idx;        /* bytes accumulated in word_buf (0-4) */
    uint8_t dummy_remaining; /* FT800-FT813 library's dummy bytes to absorb for reads */
} lv_eve_hal_bridge_t;

/**********************
 *  STATIC VARIABLES
 **********************/

static lv_eve_hal_bridge_t s_bridge;

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void bridge_start_transfer(lv_eve_hal_bridge_t *br);
static void bridge_cmdb_check_dl(lv_eve_hal_bridge_t *br);
static void bridge_cmdb_flush_word(lv_eve_hal_bridge_t *br);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_eve_hal_bridge_init(EVE_HalContext *phost, lv_draw_eve_parameters_t *params)
{
    lv_memzero(&s_bridge, sizeof(s_bridge));
    s_bridge.phost = phost;

    /* Read display timing from EVE registers */
    lv_memzero(params, sizeof(*params));

    params->hor_res        = (uint16_t)EVE_Hal_rd16(phost, REG_HSIZE);
    params->ver_res        = (uint16_t)EVE_Hal_rd16(phost, REG_VSIZE);
    params->hcycle         = (uint16_t)EVE_Hal_rd16(phost, REG_HCYCLE);
    params->hoffset        = (uint16_t)EVE_Hal_rd16(phost, REG_HOFFSET);
    params->hsync0         = (uint16_t)EVE_Hal_rd16(phost, REG_HSYNC0);
    params->hsync1         = (uint16_t)EVE_Hal_rd16(phost, REG_HSYNC1);
    params->vcycle         = (uint16_t)EVE_Hal_rd16(phost, REG_VCYCLE);
    params->voffset        = (uint16_t)EVE_Hal_rd16(phost, REG_VOFFSET);
    params->vsync0         = (uint16_t)EVE_Hal_rd16(phost, REG_VSYNC0);
    params->vsync1         = (uint16_t)EVE_Hal_rd16(phost, REG_VSYNC1);
    params->swizzle        = (uint8_t)EVE_Hal_rd8(phost, REG_SWIZZLE);
    params->pclkpol        = (uint8_t)EVE_Hal_rd8(phost, REG_PCLK_POL);
    params->cspread        = (uint8_t)EVE_Hal_rd8(phost, REG_CSPREAD);
    params->pclk           = (uint8_t)EVE_Hal_rd8(phost, REG_PCLK);
    params->has_crystal    = true;  /* already booted, value used by EVE_init() */
    params->has_gt911      = false; /* already booted, value used by EVE_init() */
    params->backlight_pwm  = (uint8_t)EVE_Hal_rd8(phost, REG_PWM_DUTY);
    params->backlight_freq = (uint16_t)EVE_Hal_rd16(phost, REG_PWM_HZ);
}

void lv_eve_hal_bridge_op_cb(lv_display_t *disp, lv_draw_eve_operation_t operation,
                             void *data, uint32_t length)
{
    lv_eve_hal_bridge_t *br = &s_bridge;
    LV_UNUSED(disp);

    switch(operation) {
        case LV_DRAW_EVE_OPERATION_CS_ASSERT:
            br->addr_idx = 0;
            br->transfer_started = false;
            br->is_cmdb_write = false;
            br->word_idx = 0;
            br->dummy_remaining = 0;
            break;

        case LV_DRAW_EVE_OPERATION_CS_DEASSERT:
            if(br->transfer_started) {
                /* Flush any partial word accumulator */
                if(br->is_cmdb_write && br->word_idx > 0) {
                    bridge_cmdb_flush_word(br);
                }
                EVE_Hal_endTransfer(br->phost);
                /* Restore eve_hal's internal cmd tracking after the
                 * FT800-FT813 library wrote to the command buffer */
                if(br->is_cmdb_write) {
                    EVE_Cmd_restore(br->phost);
                }
                br->transfer_started = false;
                br->is_cmdb_write = false;
            }
            break;

        case LV_DRAW_EVE_OPERATION_SPI_SEND: {
            uint8_t *src = (uint8_t *)data;
            uint32_t remaining = length;

            if(!br->transfer_started) {
                /* Accumulate address bytes */
                while(br->addr_idx < 3 && remaining > 0) {
                    br->addr_buf[br->addr_idx++] = *src++;
                    remaining--;
                }
                if(br->addr_idx == 3) {
                    bridge_start_transfer(br);
                }
            }

            if(br->transfer_started && remaining > 0) {
                /* Absorb dummy bytes if this is a read setup */
                while(br->dummy_remaining > 0 && remaining > 0) {
                    br->dummy_remaining--;
                    src++;
                    remaining--;
                }

                if(br->is_cmdb_write) {
                    /* CMDB path: accumulate 4-byte words for
                     * display list overflow detection */
                    for(uint32_t i = 0; i < remaining; i++) {
                        br->word_buf[br->word_idx++] = src[i];
                        if(br->word_idx == 4) {
                            bridge_cmdb_flush_word(br);
                        }
                    }
                }
                else {
                    /* Non-CMDB path: forward bytes directly */
                    for(uint32_t i = 0; i < remaining; i++) {
                        EVE_Hal_transfer8(br->phost, src[i]);
                    }
                }
            }
            break;
        }

        case LV_DRAW_EVE_OPERATION_SPI_RECEIVE: {
            uint8_t *dst = (uint8_t *)data;

            if(!br->transfer_started) {
                /* Edge case: receive before address complete — shouldn't happen
                 * in normal operation, return zeros */
                lv_memzero(dst, length);
                break;
            }

            /* Absorb dummy bytes first */
            while(br->dummy_remaining > 0) {
                EVE_Hal_transfer8(br->phost, 0);
                br->dummy_remaining--;
            }

            for(uint32_t i = 0; i < length; i++) {
                dst[i] = EVE_Hal_transfer8(br->phost, 0);
            }
            break;
        }

        case LV_DRAW_EVE_OPERATION_POWERDOWN_SET:
        case LV_DRAW_EVE_OPERATION_POWERDOWN_CLEAR:
            /* No-op: EVE is already booted and managed by eve_hal.
             * The FT800-FT813 library calls these during EVE_init(),
             * but the bridge assumes the chip is already running. */
            break;
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Check display list fullness by closing the CMDB transfer,
 * letting the coprocessor drain, reading REG_CMD_DL, and reopening.
 */
static void bridge_cmdb_check_dl(lv_eve_hal_bridge_t *br)
{
    EVE_HalContext *phost = br->phost;

    /* Close current CMDB transfer so we can read registers */
    EVE_Hal_endTransfer(phost);
    EVE_Cmd_restore(phost);

    /* Wait for most commands to be processed, allowing up to
     * 8 words still in flight. Avoids a full stall while
     * keeping REG_CMD_DL reasonably up to date. */
    EVE_Cmd_waitSpace(phost, EVE_CMD_FIFO_SIZE - 32);

    /* Get actual display list usage and reset estimate */
    uint16_t dl_offset = EVE_Hal_rd16(phost, REG_CMD_DL);
    br->dl_estimate = dl_offset / 4; /* convert bytes to entries */

    uint16_t dl_count = EVE_DL_SIZE / 4;
    if(br->dl_estimate >= dl_count - DL_VOID_THRESHOLD) {
        br->dl_voiding = true;
    }

    /* Reopen CMDB transfer */
    EVE_Hal_startTransfer(phost, EVE_TRANSFER_WRITE, REG_CMDB_WRITE);
}

/**
 * Check if a word is a coprocessor command (CMD_*) or raw DL instruction,
 * i.e. a safe boundary for flushing. CMD_* words have 0xFFFFFF__ prefix.
 * Raw DL instructions have opcodes 0x00-0x3F in the top byte.
 */
static inline bool is_cmd_boundary(uint32_t word)
{
    return (word & 0xFFFFFF00) == 0xFFFFFF00  /* CMD_* */
        || (word >> 24) <= 0x3F;              /* raw DL instruction */
}

/**
 * Process one complete 4-byte CMDB word.
 * Tracks estimated DL usage and flushes to get the real value
 * when the estimate approaches the limit. Voids commands when
 * the DL is nearly full, except CMD_SWAP and CMD_DLSTART.
 */
static void bridge_cmdb_flush_word(lv_eve_hal_bridge_t *br)
{
    EVE_HalContext *phost = br->phost;
    uint32_t word = (uint32_t)br->word_buf[0]
                  | ((uint32_t)br->word_buf[1] << 8)
                  | ((uint32_t)br->word_buf[2] << 16)
                  | ((uint32_t)br->word_buf[3] << 24);
    br->word_idx = 0;

    if(br->dl_voiding) {
        /* Only let through commands that control frame presentation */
        if(word == CMD_SWAP || word == CMD_DLSTART) {
            EVE_Hal_transfer32(phost, word);
            br->dl_voiding = false;
            br->dl_check_pending = false;
            br->dl_estimate = 0;
        }
        /* Everything else is silently dropped */
        return;
    }

    /* If a DL check is pending, do it at the next safe boundary
     * (before forwarding) so we don't flush mid-command */
    if(br->dl_check_pending && is_cmd_boundary(word)) {
        br->dl_check_pending = false;
        bridge_cmdb_check_dl(br);
        if(br->dl_voiding) {
            /* DL is full — apply voiding logic to this word too */
            if(word == CMD_SWAP || word == CMD_DLSTART) {
                EVE_Hal_transfer32(phost, word);
                br->dl_voiding = false;
                br->dl_estimate = 0;
            }
            return;
        }
    }

    /* Forward the word normally */
    EVE_Hal_transfer32(phost, word);

    if(word == CMD_DLSTART) {
        br->dl_voiding = false;
        br->dl_check_pending = false;
        br->dl_estimate = 0;
        return;
    }

    /* Conservative estimate: each CMDB word may generate
     * up to DL_ENTRIES_PER_CMDB_WORD display list entries */
    br->dl_estimate += DL_ENTRIES_PER_CMDB_WORD;

    uint16_t dl_count = EVE_DL_SIZE / 4;
    if(br->dl_estimate >= dl_count - DL_VOID_THRESHOLD) {
        /* Estimate says we're close — defer actual check to
         * the next command boundary */
        br->dl_check_pending = true;
    }
}

/**
 * Finalize address accumulation and call EVE_Hal_startTransfer.
 */
static void bridge_start_transfer(lv_eve_hal_bridge_t *br)
{
    EVE_HalContext *phost = br->phost;
    uint32_t addr = ((uint32_t)br->addr_buf[0] << 16)
                  | ((uint32_t)br->addr_buf[1] << 8)
                  | (uint32_t)br->addr_buf[2];

    /* Bit 23 set = write, clear = read */
    EVE_TRANSFER_T rw;
    if(addr & 0x800000) {
        rw = EVE_TRANSFER_WRITE;
        addr &= 0x3FFFFF;
    }
    else {
        rw = EVE_TRANSFER_READ;
        addr &= 0x3FFFFF;
        /* The FT800-FT813 library always sends exactly 1 dummy byte
         * after the 3 address bytes for reads (via spi_transmit_32
         * packing). EVE_Hal_startTransfer handles the real hardware
         * dummy bytes internally, so we absorb and discard this 1
         * library-level dummy from the byte stream. */
        br->dummy_remaining = 1;
    }

    /* When writing to the command buffer, ensure at least half
     * the FIFO is free. The FT800-FT813 library streams commands
     * in burst mode without flow control, so this is the only
     * opportunity to wait for space. */
    if(rw == EVE_TRANSFER_WRITE && addr == REG_CMDB_WRITE) {
        EVE_Cmd_waitSpace(phost, EVE_CMD_FIFO_SIZE >> 1);
        br->is_cmdb_write = true;
        br->word_idx = 0;

        /* Seed estimate from actual DL usage before opening */
        if(!br->dl_voiding) {
            uint16_t dl_offset = EVE_Hal_rd16(phost, REG_CMD_DL);
            br->dl_estimate = dl_offset / 4;
            uint16_t dl_count = EVE_DL_SIZE / 4;
            if(br->dl_estimate >= dl_count - DL_VOID_THRESHOLD) {
                br->dl_voiding = true;
            }
        }
    }

    EVE_Hal_startTransfer(phost, rw, addr);
    br->transfer_started = true;
}

#endif /* LV_USE_EVE5 && LV_USE_DRAW_EVE */
