/*-
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
 * Copyright (c) 2012 - 2014 Sage Electronic Engineering.  All rights reserved.
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

// Host controller access functions CMD/DATA and ACMD
static bool sd_host_xfer(sdhc_t* host_p, sdxfer_t* xfer_p);
static bool sd_app_specific_host_xfer(sdhc_t* host_p, sdxfer_t* xfer_p);

// Card control functions
static bool sd_select_deselect_card(sdcard_t* card_p);

// Card initialization functions
static bool sd_idle(sdcard_t* card_p);
static bool sd_send_if_cond(sdcard_t* card_p);
static bool sd_send_op_cond(sdcard_t* card_p);
static bool sd_send_csd(sdcard_t* card_p);
static bool sd_all_send_cid(sdcard_t* card_p);
static bool sd_send_relative_addr(sdcard_t* card_p);
static bool sd_card_identification_mode(sdcard_t* card_p);

/**
 * @brief     sd_host_xfer - Execute the host controller callback to perform
 *                 a transfer from the host to the card and read responses and
 *                 data from the card
 *
 * @param     sdhc_t* host_p - Pointer to the host controller abstraction structure
 * @param     sdxfer_t* xfer_p - Pointer to the host style transfer structure
 *
 * @return    bool - True if successful, false otherwise
 */
static bool sd_host_xfer(sdhc_t* host_p, sdxfer_t* xfer_p) {
    bool status = false;

    dprintf(DEBUG_HDL_SD, "SD card: Sending CMD%u with arg1 0x%08x\n",
            xfer_p->cmd_idx, xfer_p->arg1);

    status = host_p->sdhc_cmd(host_p, xfer_p);
    if (status) {
        switch (xfer_p->rsp_type) {
        case rsp136_e:
            dprintf(DEBUG_HDL_SD,
                    "SD card: Response 136: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
                    xfer_p->response[0], xfer_p->response[1], xfer_p->response[2],
                    xfer_p->response[3]);
            break;

        case rsp48_e:
            dprintf(DEBUG_HDL_SD, "SD card: Response 48: 0x%08x\n",
                    xfer_p->response[0]);
            break;

        case rsp48_busy_e:
            dprintf(DEBUG_HDL_SD,
                    "SD card: Response 48 with busy signal: 0x%08x\n",
                    xfer_p->response[0]);
            break;

        case rsp_none_e:
        default:
            break;

        }
    } else {
        dprintf(DEBUG_HDL_SD, "SD card:  Failed to respond to CMD%u\n",
                xfer_p->cmd_idx);
    }

    return status;
}

/**
 * @brief    sd_app_specific_host_xfer - This function performs the necessary steps to
 *                 allow the SD card to accept application specific commands (namely, it
 *                 sends CMD55 prior to sending the ACMD<XX>).  CMD55 puts the card into
 *                 application specific mode.
 *
 * @param     sdhc_t* host_p - Pointer to the host controller abstraction structure
 * @param     sdxfer_t* xfer_p - Pointer to the host style transfer structure
 *
 * @return    bool - true if successful, false otherwise
 */
static bool sd_app_specific_host_xfer(sdhc_t* host_p, sdxfer_t* xfer_p) {
    bool status = false;
    sdxfer_t xfer;

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_APP_CMD55;
    xfer.arg1 = 0;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // send CMD55 to place card in APP specific mode
    status = sd_host_xfer(host_p, &xfer);
    if (status) {
        // send APP specific CMD
        status = sd_host_xfer(host_p, xfer_p);
    }

    return status;
}

/**
 * @brief    sd_select_deselect_card - This function implements CMD7 SELECT/DESELECT_CARD
 *                 to toggle a card between standby and transfer state
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_select_deselect_card(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;

    // set up a transaction structure
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SELECT_CARD_CMD7;
    xfer.arg1 = (uint32_t) (card_p->rca) << 16;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_busy_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // invoke the host controller callback
    status = sd_host_xfer(card_p->host_p, &xfer);

    if (status) {
        card_p->is_selected = card_p->is_selected == true ? false : true;
    }

    return status;
}

/**
 * @brief    sd_read_single_block - Read a single block from the card
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 * @param    uint8_t* data_p - Pointer to the data buffer to read into
 * @param    uint32_t addr - Logical block address to read
 *
 * @return   bool - True on success false otherwise
 */
bool sd_read_single_block(sdcard_t* card_p, uint8_t* data_p, uint32_t addr) {
    bool status = false;
    sdxfer_t xfer;

    if (!card_p->is_selected) {
        // select the card (CMD7)
        sd_select_deselect_card(card_p);
    }

    // set up a transaction structure
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_READ_SINGLE_BLOCK_CMD17;
    xfer.arg1 = addr;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = rdxfer_e;
    xfer.data_p = data_p;
    dprintf(DEBUG_HDL_SD, "---- data_p address: 0x%08x ----\n",
            (uint32_t )data_p);

    // invoke the host controller callback
    status = sd_host_xfer(card_p->host_p, &xfer);

    return status;
}

bool sd_read_multiple_block(sdcard_t* card_p) {
    bool status = false;
    // stub
    return status;
}

bool sd_stop_transmission(sdcard_t* card_p) {
    bool status = false;
    // stub
    return status;
}

/**
 * @brief    sd_idle - This function implements the CMD0 GO_IDLE_MODE,
 *                 which resets the SD card and prepares it for initialization
 *
 * @param    sdcard_t* card_p - pointer to the SD card abstraction structure
 *
 * @return   bool - true if successful, false otherwise
 */
static bool sd_idle(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;

    // set up a transaction structure
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_GO_IDLE_STATE_CMD0;
    xfer.arg1 = 0;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp_none_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // invoke the host controller callback
    status = sd_host_xfer(card_p->host_p, &xfer);

    return status;
}

/**
 * @brief    sd_send_if_cond - This function executes CMD8 SEND_IF_COND, to determine
 *                 whether or not the host controller is compatible with the SD card.
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_send_if_cond(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;

    /*
     * Send CMD8 to determine whether or not the host controller's voltage setting
     * is correct for the card
     * NOTE: it should always be correct for SD cards, as there is really only one
     * voltage range currently supported as of 2013.  This may change in the future
     * though.  If this fails, the card is not an SD, SDHC, or SDXC card and the card
     * is unusable.
     */
    // set up a transaction structure
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = SD_SEND_IF_COND_CMD8;

    // voltage range 2.7 to 3.6 v and test pattern = 0xAA
    xfer.arg1 = SD_VOLTAGE_RANGE_270_360 | SD_IF_COND_ECHO;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // check the echo response
    while (tries--) {
        if (sd_host_xfer(card_p->host_p, &xfer)) {
            if (xfer.response[0] == xfer.arg1) {
                status = true;
                break;
            }
        }
    }

    return status;
}

/**
 * @brief    sd_send_op_cond - This function executes the ACMD41 SEND_OP_COND, which
 *                 transitions the card from the idle state to the ready state.  This
 *                 transition requires a minimum of one second to complete, and is
 *                 required for SDHC and SDXC card enumeration.
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_send_op_cond(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;

    // Send Application specific command ACMD41 in inquiry mode
    // (no voltage bits set) to read the OCR
    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = SD_APP_SEND_OP_COND_CMD41;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    // setup the voltages based on what is supported by the host
    xfer.arg1 |= card_p->host_p->capabilities.cap1 & SDHCI_CAN_VDD_180 ?
                 VDD_S18A : 0;
    xfer.arg1 |= card_p->host_p->capabilities.cap1 & SDHCI_CAN_VDD_300 ?
                 VDD_RANGE_27_30 : 0;
    xfer.arg1 |= card_p->host_p->capabilities.cap1 & SDHCI_CAN_VDD_330 ?
                 VDD_RANGE_30_33 : 0;

    // normal SD cards ignore the HCS bit, so set it for HC/XC types
    xfer.arg1 |= HCS;

    while (tries--) {
        status = sd_app_specific_host_xfer(card_p->host_p, &xfer);
        if (status) {
            // initialization takes 1 second to complete, and we query the busy bit
            // in the response to know when to stop
            if (xfer.response[0] & OCR_DONE_BSY_N) {
                card_p->ocr = xfer.response[0];
                card_p->hcxc_card = (HCS == (card_p->ocr & HCS));
                break;
            } else {
                udelay(100);
            }
        }
    }
    return status;
}

/**
 * @brief    sd_send_csd - Request the values from CSD register
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_send_csd(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SEND_CSD_CMD9;
    xfer.arg1 = (uint32_t) (card_p->rca) << 16;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp136_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            // the Host Response places bits [127:8] in bits [119:0] of the response (hence the shift)
            memmove(xfer.response, (void*) (((uint8_t*) (xfer.response)) - 1),
                    sizeof(xfer.response));
            card_p->csd[0] = xfer.response[3];
            card_p->csd[1] = xfer.response[2];
            card_p->csd[2] = xfer.response[1];
            card_p->csd[3] = xfer.response[0];
            decode_csd_sd(card_p->csd, &card_p->decode.csd_decode);
            break;
        }

    }
    return status;
}

/**
 * @brief    sd_all_send_cid - This function executes CMD2 ALL_SEND_CID, which transitions
 *                 the card from the ready state to the identification state, and allows the
 *                 card to transmit the contents of the CID register
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_all_send_cid(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_ALL_SEND_CID_CMD2;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp136_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            // the Host Response places bits [127:8] in bits [119:0] of the response (hence the shift)
            memmove(xfer.response, (void*) (((uint8_t*) (xfer.response)) - 1),
                    sizeof(xfer.response));

            // the utility functions swap the result prior to use
            card_p->cid[0] = xfer.response[3];
            card_p->cid[1] = xfer.response[2];
            card_p->cid[2] = xfer.response[1];
            card_p->cid[3] = xfer.response[0];
            decode_cid_sd(card_p->cid, &card_p->decode.cid_decode);
            break;
        }

    }
    return status;

}

/**
 * @brief    sd_send_relative_addr - This function executes CMD3 - SEND_RELATIVE_ADDRESS, of the
 *                 card initialization sequence.  After successful completion, the relative card
 *                 address (RCA) of the card is known, and the SD card enters standby mode from
 *                 card identification mode
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_send_relative_addr(sdcard_t* card_p) {
    bool status = false;
    sdxfer_t xfer;
    uint32_t tries = SD_STATE_CHANGE_ATTEMPTS;
    sd_card_state_e current_state = invalid_e;

    memset(&xfer, 0, sizeof(sdxfer_t));
    xfer.cmd_idx = MMC_SET_RELATIVE_ADDR_CMD3;
    xfer.cmd_type = cmd_normal_e;
    xfer.rsp_type = rsp48_e;
    xfer.xfer_type = wrxfer_e;
    xfer.data_p = NULL;

    while (tries--) {
        status = sd_host_xfer(card_p->host_p, &xfer);
        if (status) {
            // the response is the R6 response containing the relative card
            // address, and current status info
            card_p->rca = (uint16_t) (xfer.response[0] >> 16);
            if (xfer.response[0] & RCAERROR_MSK) {
                dprintf(DEBUG_HDL_SD,
                        "SD card: RCA request responded with error status: 0x%08x\n",
                        xfer.response[0] & RCAERROR_MSK);
                status = false;
                // here there should probably be a full status check
            }

            // now the card should be in stby mode (this might not be reflected until
            // the next command (the offset is bit 9 : 12 for state info)
            current_state = (sd_card_state_e) ((xfer.response[0]
                    & RCASTATUS_CURRENT_STATE) >> 9);
            if (current_state == stby_e) {
                dprintf(DEBUG_HDL_SD,
                        "SD card: Current state is stby: 0x%02x - \n",
                        current_state);
                break;
            }
        }
    }
    return status;

}

/**
 * @brief    sd_card_identification_mode - This function executes the call sequence/state
 *                 machine outlined in section 4.2 of "Physical Layer Simplified Specification
 *                 Version 4.10"
 *
 * @param    sdcard_t* card_p - Pointer to the SD card abstraction structure
 *
 * @return   bool - True if successful, false otherwise
 */
static bool sd_card_identification_mode(sdcard_t* card_p) {
    bool status = false;
    uint32_t tries = NUM_INIT_ATTEMPTS;

    ASSERT32FLAT();
    while (tries--) {
        // CMD8 - SEND_IF_COND
        if (!(status = sd_send_if_cond(card_p)))
            continue;

        // ACMD41 - SEND_OP_COND to properly initialize SDHC and SDXC cards...
        if (!(status = sd_send_op_cond(card_p)))
            continue;

        // If the voltage needs to change based on the response for ACMD41, change it using CMD11
        // @TODO: implement CMD11 voltage switch if necessary... probably don't need to support this
        // for a while

        // Else Send CMD2 to get the CID, the card should be in identification state
        if (!(status = sd_all_send_cid(card_p)))
            continue;

        // Send CMD3 to get the relative card address (RCA) used for broad cast commands
        if (!(status = sd_send_relative_addr(card_p)))
            continue;

        // Send CMD9 to get the CSD register info
        if (!(status = sd_send_csd(card_p)))
            continue;

        // the card will now be in standby state, and is ready for data transfers
        if (status == true) {
            dprintf(DEBUG_HDL_SD,
                    "SD card: Initialization of SD card is complete:\n");
            dprintf(DEBUG_HDL_SD, "  - currrent card state:  standby\n");
            dprintf(DEBUG_HDL_SD, "  - RCA: 0x%04x\n", card_p->rca);
            dprintf(DEBUG_HDL_SD, "  - OCR: 0x%08x\n", card_p->ocr);
            dprintf(DEBUG_HDL_SD, "  - CID: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
                    card_p->cid[0], card_p->cid[1], card_p->cid[2], card_p->cid[3]);
            dprintf(DEBUG_HDL_SD, "  - CSD: 0x%08x, 0x%08x, 0x%08x, 0x%08x\n",
                    card_p->csd[0], card_p->csd[1], card_p->csd[2], card_p->csd[3]);
            dprintf(DEBUG_HDL_SD, "  - card type is ");
            if (card_p->hcxc_card) {
                dprintf(DEBUG_HDL_SD, "SDHC/SDXC\n");
            } else {
                dprintf(DEBUG_HDL_SD, "Standard Capacity SD\n");
            }

            break;
        }
    }
    return status;
}

/**
 * @brief    sd_card_bus_init - Public function to initialize the SD card-bus.
 *                 This function is called by the host controller after the host
 *                 controller has completed its initial "pessimistic" configuration.
 *                 This function implements the steps in the sequence diagram of figure
 *                 4-1 of the "Physical Layer Simplified Specification Version 4.10" of the
 *                 SD card specification documents.
 *
 * @param    sdhc_t* host_ctrl_p - Pointer to the host controller structure
 *
 * @return   bool - status, true = success, false = failure of the card bus initialization
 */
bool sd_card_bus_init(sdhc_t* host_ctrl_p) {
    bool status = false;

    // allocate the card
    sdcard_t* card_p = malloc_fseg(sizeof(sdcard_t));
    if (!card_p) {
        dprintf(DEBUG_HDL_SD, "SD: card failed to allocate\n");
        return status;
    }
    dprintf(DEBUG_HDL_SD, "SD:  card_p address: 0x%08x\n", (uint32_t )card_p);

    // set the host pointer
    card_p->host_p = host_ctrl_p;

    // Set Idle Mode with CMD0
    status = sd_idle(card_p);

    if (status) {
        // execute the initialization/identification procedure
        card_p->initialized = sd_card_identification_mode(card_p);
    }

    if (!card_p->initialized) {
        dprintf(DEBUG_HDL_SD, "SD: Card init failed, check card...\n");
        status = card_p->initialized;
        free(card_p);
    } else {
        // If we get here, the card is in standby mode, so give a
        // reference to the host controller
        host_ctrl_p->card_p = card_p;
    }
    return status;
}
