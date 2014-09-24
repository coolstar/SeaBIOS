/*-
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
 * Copyright (c) 2014 Sage Electronic Engineering.  All rights reserved.
 *
 * Originated from FreeBSD source file source/dev/mmc/mmc.c
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Portions of this software may have been developed with reference to
 * the SD Simplified Specification.  The following disclaimer may apply:
 *
 * The following conditions apply to the release of the simplified
 * specification ("Simplified Specification") by the SD Card Association and
 * the SD Group. The Simplified Specification is a subset of the complete SD
 * Specification which is owned by the SD Card Association and the SD
 * Group. This Simplified Specification is provided on a non-confidential
 * basis subject to the disclaimers below. Any implementation of the
 * Simplified Specification may require a license from the SD Card
 * Association, SD Group, SD-3C LLC or other third parties.
 *
 * Disclaimers:
 *
 * The information contained in the Simplified Specification is presented only
 * as a standard specification for SD Cards and SD Host/Ancillary products and
 * is provided "AS-IS" without any representations or warranties of any
 * kind. No responsibility is assumed by the SD Group, SD-3C LLC or the SD
 * Card Association for any damages, any infringements of patents or other
 * right of the SD Group, SD-3C LLC, the SD Card Association or any third
 * parties, which may result from its use. No license is granted by
 * implication, estoppel or otherwise under any patent or other rights of the
 * SD Group, SD-3C LLC, the SD Card Association or any third party. Nothing
 * herein shall be construed as an obligation by the SD Group, the SD-3C LLC
 * or the SD Card Association to disclose or distribute any technical
 * information, know-how or other confidential information to any third party.
 */

#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include "util.h"
#include "malloc.h"
#include "output.h"
#include "config.h"
#include "sd.h"
#include "mmc.h"

/**
 * @brief    mmc_send_op_cond - This function executes CMD1 - SEND_OP_COND.
 *                 This function transitions the card from the idle state
 *                 to the ready state and takes a minimum of one second to
 *                 complete.
 *
 * @param    sdcard_t* card_p - Pointer to the MMC card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
bool mmc_send_op_cond(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;

    // double check it is idle
    status = sd_idle(card_p);

    // read the OCR using OP_COND command 1
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SEND_OP_COND_CMD1;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // Standard capacity SD cards ignore the HCS bit, so set it for HC/XC types
    // eMMC cards support all voltages so it is expected to respond
    // with 0x00ff8080.  If it is a HC/XC card, it should respond
    // with 0x40ff8080.  When it is no longer busy, it will set bit 31
    // in the response: 0xc0ff8080
    xfer.arg1 = HCS | MMC_OCR_VOLTAGE_MASK;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            // initialization takes 1 second to complete, and we query the busy bit
            // in the response to know when to stop
            if (xfer.response[0] & OCR_DONE_BSY_N) {
                card_p->ocr = xfer.response[0];
                card_p->hcxc_card = (HCS == (card_p->ocr & HCS));
                break;
            } else {
                udelay(100);
                status = false;
            }
        }
    }
    return status;
}

/**
 * @brief    mmc_set_relative_addr - This function executes CMD3 - SET_RELATIVE_ADDRESS, of the
 *                 card initialization sequence.  After successful completion, the relative card
 *                 address (RCA) of the card is set, and the SD card enters standby mode from
 *                 card identification mode
 *
 * @param    sdcard_t* card_p - pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
bool mmc_set_relative_addr(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_TRY_AGAIN;

    card_p->rca = 0x0001;// No particular meaning, just a value to identify this device

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SET_RELATIVE_ADDR_CMD3;
    xfer.arg1 = card_p->rca << 16;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            if (xfer.response[0] & RCAERROR_MSK) {
                dprintf(DEBUG_HDL_SD,
                        "MMC: Set RCA request responded with error status: 0x%08x\n",
                        xfer.response[0] & RCAERROR_MSK);
                status = false;
            }
            dprintf(7, "MMC: Successfully set RCA to %x with CMD3\n",
                    card_p->rca);
            break;
        }
    }
    return status;
}

/**
 * @brief    mmc_send_ext_csd - Execute CMD 8 - SEND_EXT_CSD.  This function is
 *                           used to request the values from the Ext CSD register.
 *
 * @param    sdcard_t* card_p - Pointer to the MMC card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
bool mmc_send_ext_csd(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_TRY_AGAIN;
    uint32_t i, size;

    if (!card_p->is_selected) {
        // select the card (CMD7)
        sd_select_deselect_card(card_p);
    }

    size = sizeof(card_p->ext_csd) / sizeof(uint32_t);

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SEND_EXT_CSD_CMD8;
    xfer.arg1 = (uint32_t) (card_p->rca) << 16;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = rdxfer_e;
    xfer.data_p = card_p->ext_csd;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            dprintf(7, "SD: Ext CSD:");
            for (i = 0; i < size; i++) {
                if (!(i % 8))    // every 8th dword, start a new line
                    dprintf(7, "\n");
                else
                    dprintf(7, " ");
                dprintf(7, "%08x", card_p->ext_csd[i]);
            }
            dprintf(7, "\n");
            decode_ext_csd(card_p->ext_csd, &card_p->decode.ext_csd_decode);
            break;
        }

    }
    return status;
}
