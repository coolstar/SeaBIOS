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

#include "stacks.h"
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
#include "sdhc_generic.h"
#include "sd_if.h"

static int sd_disk_read(struct disk_op_s* op);
static int sd_disk_read_aligned(struct disk_op_s* op);

/**
 * @brief    sd_disk_init - Finalize the SeaBIOS drive initialization and
 *                            register the drive
 *
 * @param    sdif_t* sdif_p - Pointer to SD disk interface structure
 *
 * @return   none
 */
static void sd_disk_init(sdif_t* sdif_p) {
    sdif_p->drive.blksize = sdif_p->hostctrl_p->block_size;
    sdif_p->drive.type = DTYPE_SD;
    sdif_p->drive.cntl_id = 0; // @TODO: Only one SD card is supported!
    sdif_p->drive.removable = false;
    sdif_p->drive.translation = TRANSLATION_LBA;
    // shift by 9 (divide by 512 block size)
    sdif_p->drive.sectors =
            sdif_p->hostctrl_p->card_p->decode.csd_decode.capacity >> 9;

    if (sdif_p->hostctrl_p->slot_type == removable_card_e) {
        // generate host vendor/spec string
        sdif_p->desc = znprintf(MAXDESCSIZE,
                "SD Card Vendor ID: %d %c%c%c%c%c Rev %d.%d",
                sdif_p->hostctrl_p->host_vendor_id,
                sdif_p->hostctrl_p->card_p->cid[0] & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 24) & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 16) & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 8) & 0xff,
                sdif_p->hostctrl_p->card_p->cid[1] & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[2] >> 20) & 0xf,
                (sdif_p->hostctrl_p->card_p->cid[2] >> 16) & 0xf);
    } else {
        sdif_p->desc = znprintf(MAXDESCSIZE,
                "MMC Vendor ID: %d %c%c%c%c%c%c Rev %d.%d",
                sdif_p->hostctrl_p->host_vendor_id,
                sdif_p->hostctrl_p->card_p->cid[0] & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 24) & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 16) & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[1] >> 8) & 0xff,
                sdif_p->hostctrl_p->card_p->cid[1] & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[2] >> 24) & 0xff,
                (sdif_p->hostctrl_p->card_p->cid[2] >> 20) & 0xf,
                (sdif_p->hostctrl_p->card_p->cid[2] >> 16) & 0xf);
    }

    sdif_p->boot_priority = bootprio_find_pci_device(
            (struct pci_device*) sdif_p->pci_p);
    dprintf(DEBUG_HDL_SD, "SD card boot priority: 0x%08x\n",
            sdif_p->boot_priority);

    // register the device as an SD/MMC drive
    boot_add_sd(&sdif_p->drive, sdif_p->desc, (int) sdif_p->boot_priority);
}

/**
 * @brief    sd_host_setup - Setup the host controller driver, if the host
 *                 is successfully initialized, setup the SD card initialization
 *
 * @param    sdif_t* sdif_p - Pointer to the SD disk interface structure
 *
 * @return   bool - True if the card was successfully initialized and prepared
 *                   for boot, false otherwise
 */
static bool sd_host_setup(sdif_t* sdif_p) {
    bool status = false;
    // perform the SD card initialization sequence
    if (sdhc_init(sdif_p->hostctrl_p)) {
        // if the card passes initialization and goes to standby mode, it is
        // ready for boot, so setup the disk info
        if (sd_card_bus_init(sdif_p->hostctrl_p)) {
            sd_disk_init(sdif_p);

            // the card is now enumerated, prepare it for boot (operational mode)
            sdhc_prep_boot(sdif_p->hostctrl_p);
            status = true;
        }
    }
    return status;
}

/**
 * @brief    sd_card_detect - Check to see if the card detect indicates that a card is present
 *                 if there is a card, setup the host and initialize the underlying card
 *
 * @param    sdif_t* sdif_p - Pointer to SD disk interface structure
 *
 * @return   bool - Status of the card detect bit
 */
static bool sd_card_detect(sdif_t* sdif_p) {
    bool status = false;
    uint32_t reg32 = 0;

    // check to see if the card is present
    reg32 = bar_read32(sdif_p->hostctrl_p->bar_address, SDHCI_PRESENT_STATE);
    status = (reg32 & SDHCI_CARD_PRESENT) == SDHCI_CARD_PRESENT;

    // if the card is present, register it in the boot sequence
    if (status) {
        status = sd_host_setup(sdif_p);
    }

    return status;
}

/**
 * @brief    sd_config_setup - Setup the SD host controller driver and allocate resources.
 *
 * @param    struct pci_device* pci - Pointer to the SDHCI controller PCI device.
 *
 * @return   none
 */
static void sd_config_setup(struct pci_device* pci) {
    sdif_t* dev_p;

    dprintf(DEBUG_HDL_SD, "sd_config_setup on device %02x:%02x.%x \n",
            pci_bdf_to_bus(pci->bdf), pci_bdf_to_dev(pci->bdf),
            pci_bdf_to_fn(pci->bdf));

    // allocate the PCI to SD interface structure
    dev_p = (sdif_t*) malloc_fseg(sizeof(*dev_p));
    if (!dev_p) {
        warn_noalloc();
        return;
    }
    memset(dev_p, 0, sizeof(*dev_p));

    // allocate the host controller
    dev_p->hostctrl_p = (sdhc_t*) malloc_fseg(sizeof(*dev_p->hostctrl_p));
    if (!(dev_p->hostctrl_p)) {
        warn_noalloc();
        free(dev_p);
        return;
    }
    memset(dev_p->hostctrl_p, 0, sizeof(*dev_p->hostctrl_p));
    dev_p->hostctrl_p->is_initialized = false;

    // assign the PCI device and set up the host controller
    dev_p->pci_p = pci;

    // setup bar0
    dev_p->hostctrl_p->bar_address = pci_config_readl(dev_p->pci_p->bdf, 0x10)
            & 0xFFFFFF00;

    // check for card detect
    if (!sd_card_detect(dev_p)) {
        dprintf(DEBUG_HDL_SD, "No SD card detected\n");
        free(dev_p->hostctrl_p);
        free(dev_p);
    }

    else {
        dprintf(DEBUG_HDL_SD, "SD card is inserted\n");

        // initialize bounce buffer
        if (create_bounce_buf() < 0) {
            warn_noalloc();
            free(dev_p->hostctrl_p);
            free(dev_p);
        }
    }
}

/**
 * @brief    sd_scan - SeaBIOS function to scan for the SD controller on the PCI bus
 *
 * @param    none
 *
 * @return   none
 */
static void sd_scan(void) {
    struct pci_device *pci = NULL;
    dprintf(DEBUG_HDL_SD, "SD: Scanning for SD Host controllers\n");

    // Scan PCI bus for SD/MMC host controllers
    foreachpci(pci)
    {
        if (pci->class != PCI_CLASS_SYSTEM_SDHCI) {
            continue;
        }

        if (pci->prog_if != 1) {
            continue;
        }
        dprintf(DEBUG_HDL_SD, "Found PCI SDHCI controller\n");

        // setup the SD host controller hardware
        sd_config_setup(pci);
    }
}

/**
 * @brief    sd_setup - SeaBIOS function to set up SD drives
 *
 * @param    none
 *
 * @return   none
 */
void sd_setup(void) {
    ASSERT32FLAT();
    if (CONFIG_SD) {
        dprintf(3, "init SD drives\n");
        sd_scan();
    }
}

/**
 * @brief    sd_cmd_data - SeaBIOS entry point for command/data disk transactions
 *
 * @param    struct disk_op_s *op - Pointer to the disk operations structure
 * @param    uint16_t blocksize - The size of the blocks for the block device transactions
 *
 * @return   int - disk operation status
 */
int sd_cmd_data(struct disk_op_s *op, void *cdbcmd, uint16_t blocksize) {
    int ret = DISK_RET_EPARAM;
    dprintf(3, "sd_cmd_data\n");

    switch (op->command) {
    case CMD_READ:
        ret = sd_disk_read(op);
        break;
    case CMD_WRITE:
        break;
    case CMD_RESET:
    case CMD_ISREADY:
    case CMD_FORMAT:
    case CMD_VERIFY:
    case CMD_SEEK:
        ret = DISK_RET_SUCCESS;
        break;
    default:
        op->count = 0;
        ret = DISK_RET_EPARAM;
        break;
    }
    return ret;
}

/**
 * @brief    sd_disk_read_aligned - Read into an aligned buffer a block of data from the SD card
 *
 * @param    struct disk_op_s* op - Pointer to the disk operation request
 *
 * @return   int - Disk operation status
 */
static int sd_disk_read_aligned(struct disk_op_s* op) {
    int ret = DISK_RET_SUCCESS;
    sdif_t *sdif_p = container_of(op->drive_gf, sdif_t, drive);
    uint16_t i = 0;
    uint8_t* cur_position = (uint8_t*) op->buf_fl;

    if (op->count > 1) {
        dprintf(7, "sd disk read, lba %6x, count %3x, buf %p\n",
                (u32 )op->lba, op->count, cur_position);
        if (!sd_read_multiple_block(sdif_p->hostctrl_p->card_p, cur_position,
                (uint32_t) (op->lba), op->count)) {
            dprintf(DEBUG_HDL_SD, "SD: Multiple Block Read Failed\n");
            ret = DISK_RET_EPARAM;
        }
    } else {
        for (i = 0; i < op->count; i++) {
            dprintf(7, "sd disk read, lba %6x, count %3x, buf %p\n",
                    (u32 )op->lba + i, op->count - i, cur_position);
            if (!sd_read_single_block(sdif_p->hostctrl_p->card_p, cur_position,
                    (uint32_t) (op->lba + i))) {
                dprintf(DEBUG_HDL_SD, "SD: Single Block Read Failed\n");
                ret = DISK_RET_EPARAM;
                break;
            } else {
                cur_position += BLOCK_SIZE8;
            }
        }
    }

    dprintf(8, "return from read retval = %u\n", ret);
    return ret;
}

/**
 * @brief    sd_disk_read - If the requested buffer is word aligned, performs the disk read operation
 *
 * @param    struct disk_op_s* op - Pointer to the disk operation request
 *
 * @return   int - Disk operation status
 */
static int sd_disk_read(struct disk_op_s* op) {
    int ret = DISK_RET_EPARAM;
    struct disk_op_s local_op;
    uint8_t* aligned_buf = NULL;
    uint8_t* cur_position = NULL;
    uint16_t i = 0;

    // check if the callers buffer is word aligned, if so use it directly
    if (((uint32_t) op->buf_fl & 1) == 0) {
        dprintf(8, "sd read: buffer already aligned\n");
        ret = sd_disk_read_aligned(op);
    } else {
        dprintf(DEBUG_HDL_SD,
                "SD read: unaligned buffer, performing realigned read\n");
        // get access to an aligned buffer for the disk operation
        local_op = *op;
        aligned_buf = GET_GLOBAL( bounce_buf_fl );
        cur_position = op->buf_fl;

        // execute the aligned to unaligned access one operation at a time
        local_op.buf_fl = aligned_buf;
        local_op.count = 1;

        for (i = 0; i < op->count; i++) {
            ret = sd_disk_read_aligned(&local_op);
            if (ret) {
                dprintf(DEBUG_HDL_SD, "  - aligned read fail\n");
                break;
            }
            memcpy_fl(cur_position, aligned_buf, BLOCK_SIZE8);
            cur_position += BLOCK_SIZE8;
            local_op.lba++;
        }

    }
    return ret;
}

/**
 * @brief    process_sd_op - Entry point for disk IO operations for SeaBIOS
 *
 * @param    struct disk_op_s *op - Disk IO functions
 *
 * @return   int - Disk return value from disk.h
 *
 * @NOTE:    The macro VISIBLE32FLAT is indicative of a call to 32 bit mode
 *             from 16-bit mode it forces the compiler to prepend the
 *             <function name> with <_cfunc32flat_><function name>, so in
 *             this case if you were to perform a search for the caller of
 *             process_sd_op, you may not find it unless you search for
 *             _cfunc32flat_process_sd_op
 */

int VISIBLE32FLAT process_sd_op(struct disk_op_s *op) {
    int ret = DISK_RET_EPARAM;

    // ensure the configuration exists
    if (!CONFIG_SD)
        return 0;

    dprintf(8, "Executing SD disk transaction:  %d\r\n", op->command);
    // execute a command
    switch (op->command) {
    case CMD_READ:
        dprintf(8, "SD CMD_READ: lba: 0x%08x%08x, op->count = %d\n",
                (uint32_t )(op->lba >> 32), (uint32_t )(op->lba), op->count);
        ret = sd_disk_read(op);
        break;
    case CMD_WRITE:
        break;
    case CMD_RESET:
    case CMD_ISREADY:
    case CMD_FORMAT:
    case CMD_VERIFY:
    case CMD_SEEK:
        ret = DISK_RET_SUCCESS;
        break;
    default:
        op->count = 0;
        ret = DISK_RET_EPARAM;
        break;

    }
    return ret;
}
