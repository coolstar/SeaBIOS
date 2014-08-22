/*-
 * Copyright (c) 2012 - 2014 Sage Electronic Engineering.  All rights reserved.
 *
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
#include "malloc.h"
#include "util.h"
#include "output.h"
#include "biosvar.h"
#include "pci.h"
#include "pci_ids.h"
#include "std/disk.h"
#include "bar.h"
#include "sdhci.h"
#include "sd.h"
#include "sdhc_generic.h"

// host controller functions
static void sdhc_read_response(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p);
static bool sdhc_poll_intr_status(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p,
        uint32_t intr_flags, uint32_t poll_cnt);
static void sdhc_get_ver_info(sdhc_t* sd_ctrl_p);
static void sdhc_set_clock(sdhc_t* sd_ctrl_p, uint32_t clk_val);
static void sdhc_set_power(sdhc_t* sd_ctrl_p, uint32_t pwr_mode);
static bool sdhc_reset(sdhc_t* sd_ctrl_p, uint8_t reset_flags);
static void sdhc_read_block(sdhc_t* sd_ctrl_p, uint8_t* buf_p, uint32_t count);

// callback function for performing command/response transactions
static bool sdhc_cmd(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p);

/**
 * @brief    sdhc_read_response - Helper function to read different response types
 *              based on a transfer request
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller abstraction structure
 * @param    sdxfer_t* xfer_p - Pointer to the host style transfer structure
 *
 * @return   none
 */
static void sdhc_read_response(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p) {
    memset(xfer_p->response, 0, sizeof(xfer_p->response[0] * 4));
    xfer_p->rsp_valid = false;

    switch (xfer_p->rsp_type) {
    case rsp136_e:
        xfer_p->response[0] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE);
        xfer_p->response[1] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE + 0x04);
        xfer_p->response[2] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE + 0x08);
        xfer_p->response[3] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE + 0x0C);
        xfer_p->rsp_valid = true;
        break;

    case rsp48_e:
        xfer_p->response[0] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE);
        xfer_p->rsp_valid = true;
        break;

    case rsp48_busy_e:
        xfer_p->response[0] = bar_read32(sd_ctrl_p->bar_address,
                SDHCI_RESPONSE + 0x0C);
        xfer_p->rsp_valid = true;
        break;

    case rsp_none_e:
    default:
        break;
    }
}

/**
 * @brief    sdhc_poll_intr_status - Interrupt status polling routine
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller struct
 * @param    uint32_t intr_flags - The requested interrupt number(s) to check
 * @param    uint32_t poll_cnt - The number of iterations to poll
 * @param    uint32_t usec_tmo - Microsecond delay per polling iteration
 *
 * @return   bool - True if the requested interrupt was set during this
 *              request cycle, false otherwise
 */
static bool sdhc_poll_intr_status(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p,
        uint32_t intr_flags, uint32_t poll_cnt) {
    bool status = false;
    uint32_t reg32 = 0;

    // check at least once
    if (!poll_cnt) {
        poll_cnt = 1;
    }

    // check for the interrupt flag
    while (poll_cnt--) {
        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS);

        // check that all requested interrupts were set
        if ((intr_flags & reg32) == intr_flags) {
            // in the case of response to command, save the response
            if (intr_flags == SDHCI_INT_RESPONSE) {
                sdhc_read_response(sd_ctrl_p, xfer_p);
            }

            // clear the interrupt flag(s)
            bar_write32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS, intr_flags);
            status = true;
            dprintf(DEBUG_HDL_SD,
                    "SDHC: requested interrupt (%x) occurred: %x\n", intr_flags,
                    reg32);
            break;
        } else if (reg32 & SDHCI_INT_ERROR_MASK) {
            // notify debug of timeout error
            if (reg32 & SDHCI_INT_TIMEOUT) {
                dprintf(DEBUG_HDL_SD, "SDHC: ERROR Timeout\n");
            }
            // reset the card on fatal errors
            else {
                dprintf(DEBUG_HDL_SD,
                        "SDHC: ERROR interrupt occurred, clearing interrupt and resetting card\n");
                sdhc_reset(sd_ctrl_p, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
            }

            bar_write32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS,
                    ~SDHCI_INT_ERROR_MASK);
            status = false;
            break;
        }
        udelay(10);
    }
    dprintf(DEBUG_HDL_SD, "SDHC: Current interrupt status register: 0x%08x\n",
            reg32);

    return status;
}

/**
 * @brief    sdhc_get_ver_info - Read the host controller vendor specific id and host
 *                 SD specification supported by the controller.  Also get the slot
 *                 type.
 *                 Slot Types:
 *                 0: Removable Card Slot
 *                 1: Embedded Slot for One Device
 *                 2: Shared Bus Slot
 *                 3: Reserved
 *
 * @param    sdhc_t* sd_ctrl_pl - Pointer to the host controller struct
 *
 * @return   none
 */
static void sdhc_get_ver_info(sdhc_t* sd_ctrl_p) {
    uint16_t reg16 = 0;

    reg16 = bar_read16(sd_ctrl_p->bar_address, SDHCI_HOST_VERSION);
    sd_ctrl_p->host_vendor_id = (uint8_t) ((reg16 & SDHCI_VENDOR_VER_MASK)
            >> SDHCI_VENDOR_VER_SHIFT);
    sd_ctrl_p->host_spec_ver = (uint8_t) ((reg16 & SDHCI_SPEC_VER_MASK)
            >> SDHCI_SPEC_VER_SHIFT);

    dprintf(DEBUG_HDL_SD,
            "SDHC: Found Vendor ID %x and Host Spec Version %x.00\n",
            sd_ctrl_p->host_vendor_id, sd_ctrl_p->host_spec_ver + 1);

    sd_ctrl_p->slot_type = (sd_ctrl_p->capabilities.cap1 & SDHCI_SLOT_TYPE_MASK)
            >> SDHCI_SLOT_TYPE_SHIFT;
    dprintf(DEBUG_HDL_SD, "SDHC: Host Controller Slot Type is %x\n",
            sd_ctrl_p->slot_type);
}

/**
 * @brief    sdhc_set_power - Setup the power state of the host controller based on its
 *                 reported capabilities.  The first time this is called, the power is setup
 *                 in the highest condition to support all types of cards.  After card enumeration
 *                 the voltage can be lowered.
 *
 * @param     sdhc_t* sd_ctrl_p - Pointer to the host controller abstraction structure
 * @param     uint32_t pwr_mode - The requested voltage setting for the power
 *
 * @return    none
 */
static void sdhc_set_power(sdhc_t* sd_ctrl_p, uint32_t pwr_mode) {
    uint8_t pwr = 0;

    sd_ctrl_p->pwr_mode = pwr_mode;

    // turn off the power
    bar_write8(sd_ctrl_p->bar_address, SDHCI_POWER_CONTROL, pwr);

    if (pwr_mode == 0)
        return;

    // set voltage
    switch (pwr_mode) {
    case SDHCI_CAN_VDD_180:
        pwr |= SDHCI_POWER_180;
        break;
    case SDHCI_CAN_VDD_300:
        pwr |= SDHCI_POWER_300;
        break;
    case SDHCI_CAN_VDD_330:
        pwr |= SDHCI_POWER_330;
        break;
    }

    bar_write8(sd_ctrl_p->bar_address, SDHCI_POWER_CONTROL, pwr);
    // turn on the power
    pwr |= SDHCI_POWER_ON;
    bar_write8(sd_ctrl_p->bar_address, SDHCI_POWER_CONTROL, pwr);
}

/**
 * @brief    sdhc_set_clock - Set the clock of the host controller.  This function executes the
 *                 clock divisor algorithm described in the "SD Host Controller Simplified
 *                 Specification Version 3.00" to generate the highest clock frequency that the
 *                 card host can handle based on the requested clock.
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller abstraction structure
 * @param    uint32_t clk_val - Desired clock frequency
 *
 * @return   none
 */
static void sdhc_set_clock(sdhc_t* sd_ctrl_p, uint32_t clk_val) {
    uint16_t clk = 0;
    uint32_t div = 0;    // divider to use for clock generation
    int32_t timeout = 0;

    dprintf(DEBUG_HDL_SD, "SDHC: Requested clock frequency is %u Hz\n",
            clk_val);
    // disable the clock
    bar_write16(sd_ctrl_p->bar_address, SDHCI_CLOCK_CONTROL, 0);

    // calculate the highest possible frequency <= the maximum clock frequency reported by the card
    //  V3.00: clk = max_clk/(2 * div) where div is 1 - 2046
    //  V2.00: clk = max_clk/(2 * div) where div is 1, 2, 4, 8, 16, 32, 64, 128
    //  div = 0 indicates to use the base clock
    if (clk_val < sd_ctrl_p->max_clk) {
        if (sd_ctrl_p->host_spec_ver >= SDHCI_SPEC_300) {    // version 3.00
            for (div = 2; div < SDHCI_V3_MAX_DIV; div += 2) {
                if ((sd_ctrl_p->max_clk / div) <= clk_val)
                    break;
            }
        } else {    // version 1.00 and 2.00
            for (div = 1; div < SDHCI_MAX_DIV; div <<= 1) {
                if ((sd_ctrl_p->max_clk / div) <= clk_val)
                    break;
            }
        }
    } else {
        div = 1;
    }

    sd_ctrl_p->current_clk = sd_ctrl_p->max_clk / div;

    // shift down to divide by 2, giving the correct value to write to the register
    //   ex: found div = 10h, want to write 8 to the register to get 2 * 8 = 10h
    div >>= 1;

    // fixup the divider and write it to the register
    //  V3.00: 10bit divider, div[7:0] located in bits 15:8, div[9:8] located in bits 7:6
    //  V2.00: 8bit divider located in bits 15:8
    clk = (div & SDHCI_DIVIDER_MASK) << SDHCI_DIVIDER_SHIFT;
    clk |= ((div & SDHCI_DIVIDER_EXT_MASK) >> SDHCI_DIVIDER_SHIFT)
            << SDHCI_DIVIDER_EXT_SHIFT;
    clk |= SDHCI_CLOCK_INT_EN;
    bar_write16(sd_ctrl_p->bar_address, SDHCI_CLOCK_CONTROL, clk);

    // Wait up to 10ms until it stabilizes
    timeout = 10;
    while (!((clk = bar_read16(sd_ctrl_p->bar_address, SDHCI_CLOCK_CONTROL))
            & SDHCI_CLOCK_INT_STABLE)) {
        if (timeout == 0) {
            dprintf(DEBUG_HDL_SD, "SDHC: Internal clock never stabilized\n");
            break;
        }
        timeout--;
        udelay(1000);
    }

    if (timeout > 0) {
        // clock is stable so enable the clock signal for the card bus
        dprintf(DEBUG_HDL_SD, "SDHC: card internal clock stabilized at %u Hz\n",
                sd_ctrl_p->current_clk);
        clk |= SDHCI_CLOCK_CARD_EN;
        bar_write16(sd_ctrl_p->bar_address, SDHCI_CLOCK_CONTROL, clk);
    }
}

/**
 * @brief    sdhc_reset - Issue the SD reset sequence described in the SD Host Controller Spec V3.00
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller struct
 * @param    uint8_t reset_flags - A logical OR mask of the three possible reset types:
 *                                     SDHCI_RESET_ALL - Reset entire host controller
 *                                     SDHCI_RESET_CMD - Reset command circuit
 *                                     SDHCI_RESET_DATA - Reset data & DMA circuit
 *
 * @return   bool - True if the request did not timeout, false otherwise
 */
static bool sdhc_reset(sdhc_t* sd_ctrl_p, uint8_t reset_flags) {
    uint8_t resetResult = 0;
    uint32_t timeout = 0;
    bool status = false;

    // send the reset command
    bar_write8((uint32_t) sd_ctrl_p->bar_address, SDHCI_SOFTWARE_RESET,
            reset_flags);

    // wait for all of the requested reset flags to clear until the timeout occurs
    timeout = 10;
    while ((resetResult = bar_read8(sd_ctrl_p->bar_address,
            SDHCI_SOFTWARE_RESET) & reset_flags)) {
        if (timeout == 0) {
            dprintf(DEBUG_HDL_SD,
                    "SDHC: Reset Timeout for reset request type: 0x%02x\n",
                    reset_flags);
            break;
        }
        timeout--;
        udelay(100);
    }

    if (timeout > 0) {
        dprintf(DEBUG_HDL_SD,
                "SDHC: Reset Successful for reset request type: 0x%02x\n",
                reset_flags);
        status = true;
    }
    return status;
}

/**
 * @brief    sdhc_cmd - This is the call-back entry point for the card bus driver to access
 *                 the bus via the host controller.  This function must be registered with the
 *                 card-bus driver in order for it to be able to function properly.  It is the
 *                 entry point for both command and data transfers.
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller abstraction
 * @param    sdxfer_t* xfer_p - Pointer to the command/data transfer, filled out by the
 *                 card-bus driver
 * @return   bool - True on success, false otherwise
 */
static bool sdhc_cmd(sdhc_t* sd_ctrl_p, sdxfer_t* xfer_p) {
    uint32_t state_mask = 0;
    uint8_t reg8 = 0;
    uint8_t tmo = 10;
    uint16_t mode = 0;

    // setup the state mask for the transfer
    state_mask = SDHCI_CMD_INHIBIT;
    state_mask |= (xfer_p->rsp_type == rsp48_busy_e) ? SDHCI_DAT_INHIBIT : 0;

    // wait for the state mask to clear
    while (bar_read32(sd_ctrl_p->bar_address, SDHCI_PRESENT_STATE) & state_mask) {
        if (tmo == 0) {
            dprintf(DEBUG_HDL_SD,
                    "SDHC: unable to access CMD/DAT bus, its always busy\n");
            return false;
        }
        tmo--;
        udelay(100);
    }

    if (tmo > 0) {

        // clear any existing interrupts
        uint32_t reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS);
        bar_write32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS, reg32);

        // set command argument
        bar_write32(sd_ctrl_p->bar_address, SDHCI_ARGUMENT, xfer_p->arg1);

        if (xfer_p->data_p && xfer_p->xfer_type == rdxfer_e) {
            // set data transfer mode for reading data
            mode = bar_read16(sd_ctrl_p->bar_address, SDHCI_TRANSFER_MODE);
            mode |= ( SDHCI_TRNS_READ);
            mode &= ~( SDHCI_TRNS_MULTI | SDHCI_TRNS_DMA);
            bar_write16(sd_ctrl_p->bar_address, SDHCI_TRANSFER_MODE, mode);
        } else {
            // set data transfer mode for commanding
            mode = bar_read16(sd_ctrl_p->bar_address, SDHCI_TRANSFER_MODE);
            mode &= ~( SDHCI_TRNS_READ | SDHCI_TRNS_MULTI | SDHCI_TRNS_DMA);
            bar_write16(sd_ctrl_p->bar_address, SDHCI_TRANSFER_MODE, mode);
        }

        // build the command transaction type
        reg8 = (sd_ctrl_p->crc_chk_en) ? SDHCI_CMD_CRC : 0;
        reg8 |= (sd_ctrl_p->idx_chk_en) ? SDHCI_CMD_INDEX : 0;
        reg8 |= (xfer_p->data_p != NULL) ? SDHCI_CMD_DATA : 0;
        reg8 |= (uint8_t) (xfer_p->cmd_type);
        reg8 |= (uint8_t) (xfer_p->rsp_type);
        bar_write8(sd_ctrl_p->bar_address, SDHCI_COMMAND_FLAGS, reg8);

        // initiate the transfer
        bar_write8(sd_ctrl_p->bar_address, SDHCI_COMMAND, xfer_p->cmd_idx);

        tmo = 10;
        if (xfer_p->rsp_type != rsp_none_e) {
            if (!sdhc_poll_intr_status(sd_ctrl_p, xfer_p, SDHCI_INT_RESPONSE, 1000)) {
                dprintf(DEBUG_HDL_SD,
                        "SDHC: failed to receive response to command\n");
                return false;
            }

            // if this is a read block transaction break out here to get the data
            if ((xfer_p->data_p) && (xfer_p->xfer_type == rdxfer_e)) {
                sdhc_read_block(sd_ctrl_p, xfer_p->data_p, BLOCK_SIZE8);
            }
        }

    }

    return true;
}

/**
 * @brief    sdhc_read_block - Read a block from the SD card, this function is only
 *                 executed if the data pointer and read flag are set in the sdhc_cmd
 *                 request
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller abstraction
 * @param    uint8_t* buf_p - Buffer to fill in with data (typically a block)
 * @param    uint32_t count - Number of bytes to read in the transaction
 *
 * @return   none
 */
static void sdhc_read_block(sdhc_t* sd_ctrl_p, uint8_t* buf_p, uint32_t count) {
    uint32_t lim = 0;
    uint32_t data_reg = 0;
    uint8_t* local_buf_p = NULL;

    local_buf_p = buf_p;

    //lim = min(BLOCK_SIZE8, count);
    lim = BLOCK_SIZE8;

    // ensure that there is data available
    //usleep(10000);
    // @NOTE: This usleep call was to throttle transactions during development,
    //        it can be commented back in to throttle block read transactions
    //        while modifying the driver
    if (bar_read32(sd_ctrl_p->bar_address,
            SDHCI_PRESENT_STATE) & SDHCI_DATA_AVAILABLE) {
        dprintf(DEBUG_HDL_SD, "SDHC: reading %d bytes of data\n", lim);

        // calculate the number of 32 bit values to read by converting the limit to
        // dwords (additional bytes handled later)
        lim >>= 2;
        while (lim > 0) {
            data_reg = bar_read32(sd_ctrl_p->bar_address, SDHCI_BUFFER);
            local_buf_p[0] = (uint8_t) (data_reg);
            local_buf_p[1] = (uint8_t) (data_reg >> 8);
            local_buf_p[2] = (uint8_t) (data_reg >> 16);
            local_buf_p[3] = (uint8_t) (data_reg >> 24);
            local_buf_p += 4;
            lim--;
        }

        // handle the remainder
        lim = count & 0x03;
        if (lim > 0) {
            data_reg = bar_read32(sd_ctrl_p->bar_address, SDHCI_BUFFER);
            while (lim > 0) {
                *(local_buf_p++) = (uint8_t) data_reg;
                data_reg >>= 8;
                lim--;
            }
        }
    }
#if(CONFIG_DEBUG_LEVEL > 9)
    hexdump(buf_p, BLOCK_SIZE8);
#endif

}

/**
 * @brief    sdhc_prep_boot - Post initialization function to perform changes
 *                 to the operating mode of the card prior to boot, and to
 *                 take it out of enumeration mode.
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller struct
 *
 * @return   none
 */
void sdhc_prep_boot(sdhc_t* sd_ctrl_p) {
    // boost the clock speed for boot
    //@TODO:  should check if this speed is supported first, (most newer cards do)
    sdhc_set_clock(sd_ctrl_p, 25 * MHZ);

    //@TODO: do other changes to card operating mode here
}

/**
 * @brief   sdhc_is_initialized - Check to see if the host controller is initialized
 *
 * @param   sdhc_t* sd_ctrl_p - Pointer to the host controller struct
 *
 * @return  bool - True if host initialized, false otherwise
 */
bool sdhc_is_initialized(sdhc_t* sd_ctrl_p) {
    return sd_ctrl_p->is_initialized;
}

/**
 * @brief    sdhc_init - Performs the minimum necessary steps outlined in the SD
 *                 host controller specification to prepare the SD card/host
 *                 controller for use
 *
 * @param    sdhc_t* sd_ctrl_p - Pointer to the host controller struct
 *
 * @return   bool - True if reset succeeded, false otherwise
 */
bool sdhc_init(sdhc_t* sd_ctrl_p) {
    uint32_t reg32 = 0;
    uint8_t reg8 = 0;

    // reset the the host controller
    if (sdhc_reset(sd_ctrl_p, SDHCI_RESET_ALL)) {
        // read the capabilities register
        sd_ctrl_p->capabilities.cap1 = bar_read32(sd_ctrl_p->bar_address,
        SDHCI_CAPABILITIES);
        sd_ctrl_p->capabilities.cap2 = bar_read32(sd_ctrl_p->bar_address,
                (SDHCI_CAPABILITIES + 4));
        dprintf(DEBUG_HDL_SD, "SDHC: Capabilities are 0x%08x%08x\n",
                sd_ctrl_p->capabilities.cap2, sd_ctrl_p->capabilities.cap1);

        // record the vendor and SD spec info of the host controller
        sdhc_get_ver_info(sd_ctrl_p);

        // check the power
        reg8 = bar_read8(sd_ctrl_p->bar_address, SDHCI_POWER_CONTROL);
        if (reg8 & SDHCI_POWER_ON) {
            // make sure the power is off
            sdhc_set_power(sd_ctrl_p, 0);
        }

        // setup the power for the card
        dprintf(DEBUG_HDL_SD, "SDHC: Card currently not powered, power on to ");
        // setup the power for the card
        if (sd_ctrl_p->capabilities.cap1 & SDHCI_CAN_VDD_330) {
            sdhc_set_power(sd_ctrl_p, SDHCI_CAN_VDD_330);
            dprintf(DEBUG_HDL_SD, "3.3V\n");
        } else if (sd_ctrl_p->capabilities.cap1 & SDHCI_CAN_VDD_300) {
            sdhc_set_power(sd_ctrl_p, SDHCI_CAN_VDD_300);
            dprintf(DEBUG_HDL_SD, "3.0V\n");
        } else if (sd_ctrl_p->capabilities.cap1 & SDHCI_CAN_VDD_180) {
            sdhc_set_power(sd_ctrl_p, SDHCI_CAN_VDD_180);
            dprintf(DEBUG_HDL_SD, "1.8V\n");
        }

        // determine the base clock frequency reported by the card
        if (sd_ctrl_p->host_spec_ver >= SDHCI_SPEC_300)
            sd_ctrl_p->max_clk = (sd_ctrl_p->capabilities.cap1
                    & SDHCI_V3_CLOCK_BASE_MASK) >> SDHCI_CLOCK_BASE_SHIFT;
        else
            sd_ctrl_p->max_clk = (sd_ctrl_p->capabilities.cap1
                    & SDHCI_CLOCK_BASE_MASK) >> SDHCI_CLOCK_BASE_SHIFT;

        // make sure it is non-zero and then change from MHz to Hz
        if (sd_ctrl_p->max_clk == 0) {
            dprintf(DEBUG_HDL_SD,
                    "SDHC: No base clock frequency specified by card capabilities\n");

            // @TODO:  If the clock was not set, need to set it ?
        } else {
            sd_ctrl_p->max_clk *= MHZ;
            dprintf(DEBUG_HDL_SD, "SDHC: Base clock frequency %u Hz\n",
                    sd_ctrl_p->max_clk);
        }

        // determine the base timeout frequency
        sd_ctrl_p->tmo_clk = (sd_ctrl_p->capabilities.cap1
                & SDHCI_TIMEOUT_CLK_MASK) >> SDHCI_TIMEOUT_CLK_SHIFT;
        if (sd_ctrl_p->tmo_clk == 0) {
            dprintf(DEBUG_HDL_SD,
                    "SDHC: no timeout clock frequency specified by card capabilities\n");
        } else {
            // if the units are specified in MHz adjust the frequency to reflect that
            sd_ctrl_p->tmo_clk =
                    (sd_ctrl_p->capabilities.cap1 & SDHCI_TIMEOUT_CLK_UNIT) ?
                            sd_ctrl_p->tmo_clk * MHZ : sd_ctrl_p->tmo_clk * KHZ;
            dprintf(DEBUG_HDL_SD, "SDHC: timeout frequency clock %u\n",
                    sd_ctrl_p->tmo_clk);

            // test max timeout
            bar_write8(sd_ctrl_p->bar_address, SDHCI_TIMEOUT_CONTROL, 0x0e);
        }

        // the "always supported" block size is 512, so force it
        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_BLOCK_SIZE);

        // mask off the block bits
        sd_ctrl_p->block_size &= BLOCK_MASK;
        if (sd_ctrl_p->block_size == 0) {
            dprintf(DEBUG_HDL_SD, "SDHC: no block size set...\n");
        } else {
            dprintf(DEBUG_HDL_SD, "SDHC: current block size: %u\n",
                    sd_ctrl_p->block_size);
        }

        // if necessary set the block size to the default
        if (sd_ctrl_p->block_size != BLOCK_SIZE8) {
            sd_ctrl_p->block_size = BLOCK_SIZE8;
            dprintf(DEBUG_HDL_SD, "  - setting new block size to 512 bytes\n");

            // clear the current block size bits
            reg32 &= ~BLOCK_MASK;
            reg32 |= sd_ctrl_p->block_size;
            bar_write32(sd_ctrl_p->bar_address, SDHCI_BLOCK_SIZE, reg32);

            // check that the new value was written
            reg32 = bar_read32(sd_ctrl_p->bar_address,
            SDHCI_BLOCK_SIZE) & BLOCK_MASK;
            if (reg32 != BLOCK_SIZE8) {
                dprintf(DEBUG_HDL_SD, "  - set new block size failed!");
                //@TODO: Probably should fail now?
            } else {
                dprintf(DEBUG_HDL_SD, "  - new block size set to: %u\n",
                        sd_ctrl_p->block_size);
            }
        }

        // test reset after config
        sdhc_reset(sd_ctrl_p, SDHCI_RESET_CMD | SDHCI_RESET_DATA);

        // setup the interrupts
        bar_write32(sd_ctrl_p->bar_address, SDHCI_INT_ENABLE,
            SDHCI_INT_BUS_POWER |
            SDHCI_INT_DATA_END_BIT |
            SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
            SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
            SDHCI_INT_CARD_REMOVE | SDHCI_INT_CARD_INSERT |
            SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL |
            SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE |
            SDHCI_INT_ACMD12ERR);

        // and signals
        bar_write32(sd_ctrl_p->bar_address, SDHCI_SIGNAL_ENABLE,
            SDHCI_INT_BUS_POWER |
            SDHCI_INT_DATA_END_BIT |
            SDHCI_INT_DATA_CRC | SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_INDEX |
            SDHCI_INT_END_BIT | SDHCI_INT_CRC | SDHCI_INT_TIMEOUT |
            SDHCI_INT_CARD_REMOVE | SDHCI_INT_CARD_INSERT |
            SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL |
            SDHCI_INT_DMA_END | SDHCI_INT_DATA_END | SDHCI_INT_RESPONSE |
            SDHCI_INT_ACMD12ERR);

        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_INT_ENABLE);
        dprintf(DEBUG_HDL_SD, "SDHC: Interrupts enabled to: 0x%08x\n", reg32);

        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_INT_STATUS);
        dprintf(DEBUG_HDL_SD, "SDHC: Current interrupt status: 0x%08x\n", reg32);

        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_SIGNAL_ENABLE);
        dprintf(DEBUG_HDL_SD, "SDHC: signals enabled to: 0x%08x\n", reg32);

        reg32 = bar_read32(sd_ctrl_p->bar_address, SDHCI_PRESENT_STATE);
        dprintf(DEBUG_HDL_SD, "SDHC: Present State: 0x%08x\n", reg32);

        // setup the callback(s) for the underlying card bus
        sd_ctrl_p->sdhc_cmd = &sdhc_cmd;

        // setup the cards internal clock to always be normal speed mode to blanket support all card types
        // the SD spec defines normal speed mode as 25MHz, and enumeration as 400KHz
        sdhc_set_clock(sd_ctrl_p, 400000);

        sd_ctrl_p->is_initialized = true;

        // wait for 1ms + 74 clocks - 1.5ms should be more than enough
        udelay(1500);
    }

#if(CONFIG_DEBUG_LEVEL > 7)
    hexdump((void *)sd_ctrl_p->bar_address, 0x60);
#endif
    dprintf(DEBUG_HDL_SD, "SDHC: Controller initialized successfully\n");
    return (sd_ctrl_p->is_initialized);
}

