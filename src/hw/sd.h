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

#ifndef __SD_H
#define __SD_H

#include <stdint.h>
#include <stdbool.h>
#include "sd_utils.h"

/** @file sd.h */

#include "sdhci.h"

/** @brief SD host capabilities registers */
typedef struct {
    uint32_t cap1;
    uint32_t cap2;
} capabilities_t;

typedef enum {
    removable_card_e = 0,
    embedded_slot_e = 1,
    shared_bus_e = 2,
    rsvd_e = 3
} sdslot_type_e;

typedef enum {
    cmd_normal_e = SDHCI_CMD_TYPE_NORMAL,
    cmd_suspend_e = SDHCI_CMD_TYPE_SUSPEND,
    cmd_resume_e = SDHCI_CMD_TYPE_RESUME,
    cmd_abort_e = SDHCI_CMD_TYPE_ABORT
} sdcmd_type_e;

typedef enum {
    rsp_none_e = SDHCI_CMD_RESP_NONE,
    rsp136_e = SDHCI_CMD_RESP_LONG,
    rsp48_e = SDHCI_CMD_RESP_SHORT,
    rsp48_busy_e = SDHCI_CMD_RESP_SHORT_BUSY
} sdrsp_type_e;

typedef enum {
    sd_specv_1_00_e = 0,
    sd_specv_2_00_e,
    sd_specv_3_00_e
} sd_spec_ver_e;

typedef enum {
    rdxfer_e = 0, wrxfer_e
} sdxfer_type_e;

typedef struct {
    bool idx_chk_en;
    bool crc_chk_en;
} sdxfer_flags_t;

/** @brief struct used for host to card and card to host transfers */
typedef struct {
    uint8_t cmd_idx;
    uint32_t arg1;
    sdcmd_type_e cmd_type;
    sdrsp_type_e rsp_type;
    sdxfer_type_e xfer_type;
    uint32_t response[4];
    bool rsp_valid;
    void* data_p;
} sdxfer_t;

/** @brief struct to hold decoded CID and CSD registers (see SD Specification) */
typedef struct {
    sd_cid_t cid_decode;
    sd_csd_t csd_decode;
} sdreg_decode_t;

// resolve forward declarations
struct sdcard_t;
typedef struct sdcard_t sdcard_t;

typedef struct sdhc_t {
    uint32_t bar_address;
    capabilities_t capabilities;
    uint32_t max_clk;
    uint32_t tmo_clk;
    uint32_t current_clk;
    uint32_t block_size;
    uint32_t pwr_mode;
    bool crc_chk_en;
    bool idx_chk_en;
    bool hcxc_supported;
    bool is_initialized;
    uint8_t host_vendor_id;
    uint8_t host_spec_ver;
    sdslot_type_e slot_type;
    sdcard_t* card_p;

    // call back to host for sending commands to the card via the host controller interface
    bool (*sdhc_cmd)(struct sdhc_t* sd_ctrl_p, sdxfer_t* xfer);
} sdhc_t;

/**
 * SD Memory Card operational states
 */
typedef enum {
    idle_e = 0,
    ready_e,
    ident_e,
    stby_e,
    tran_e,
    data_e,
    rcv_e,
    prg_e,
    dis_e,
    invalid_e
} sd_card_state_e;

char *sd_state_names[16] = {
        "Idle",
        "Ready",
        "Identification",
        "Standby",
        "Transfer",
        "Data",
        "Receive",
        "Program",
        "Disabled",
        "Invalid"
};

/**
 * SD Memory Card Registers
 */
typedef struct sdcard_t {
    sdhc_t* host_p;
    uint32_t cid[4];
    uint16_t rca;
    uint16_t dsr;
    uint32_t csd[4];
    uint32_t scr[2];
    uint32_t ocr;
    uint32_t ssr[16];
    uint32_t csr;
    sdreg_decode_t decode;
    bool hcxc_card;
    bool initialized;
    sd_card_state_e state;
    bool is_selected;
} sdcard_t;

#define BLOCK_SIZE8      512
#define MHZ 1000000
#define KHZ 1000
#define BLOCK_MASK 0x00000FFF

// MMC Card Commands that mostly overlap with the SD specification
// some have slightly different meaning or results
#define MMC_GO_IDLE_STATE_CMD0            0
#define MMC_SEND_OP_COND_CMD1             1
#define MMC_ALL_SEND_CID_CMD2             2
#define MMC_SET_RELATIVE_ADDR_CMD3        3
#define MMC_SET_DSR_CMD4                  4
#define MMC_SWITCH_CMD6                   6
#define MMC_SELECT_CARD_CMD7              7
#define MMC_SEND_EXT_CSD_CMD8             8
#define MMC_SEND_CSD_CMD9                 9
#define MMC_SEND_CID_CMD10                10
#define MMC_STOP_TRANSMISSION_CMD12       12
#define MMC_SEND_STATUS_CMD13             13
#define MMC_SET_BLOCKLEN_CMD16            16
#define MMC_READ_SINGLE_BLOCK_CMD17       17
#define MMC_READ_MULTIPLE_BLOCK_CMD18     18
#define MMC_WRITE_SINGLE_BLOCK_CMD24      24
#define MMC_WRITE_MULTIPLE_BLOCK_CMD25    25
#define MMC_ERASE_GROUP_START_CMD35       35
#define MMC_ERASE_GROUP_END_CMD36         36
#define MMC_ERASE_CMD38                   38
#define MMC_APP_CMD55                     55
#define MMC_SPI_READ_OCR_CMD58            58
#define MMC_SPI_CRC_ON_OFF_CMD59          59

// SD card specific commands
#define SD_SEND_RELATIVE_ADDR_CMD3        3
#define SD_SWITCH_FUNC_CMD6               6
#define SD_SEND_IF_COND_CMD8              8
#define SD_APP_SET_BUS_WIDTH_CMD6         6
#define SD_ERASE_WR_BLK_START_CMD32       32
#define SD_ERASE_WR_BLK_END_CMD33         33
#define SD_APP_SEND_OP_COND_CMD41         41
#define SD_APP_SEND_SCR_CMD51             51

// other useful defines
#define SD_VOLTAGE_RANGE_270_360          (0x1 << 8)
#define SD_IF_COND_ECHO                   0xAA
#define SD_STATE_CHANGE_ATTEMPTS          100
#define SD_TRY_AGAIN                      10

// ACMD41 bit shifts and masks
#define OCR_DONE_BSY_N        (1 << 31)
#define HCS_SHIFT 30
#define HCS  (1 << HCS_SHIFT)
#define XPC_SHIFT 28
#define XPC  (1 << XPC_SHIFT)
#define S18R_SHIFT 24
#define S18R (1 << S18R_SHIFT)
#define OCR_SHIFT 8
#define OCR  (0xFF << OCR_SHIFT)

// OCR register voltage ranges
#define VDD_27_28    (1 << 15)
#define VDD_28_29    (1 << 16)
#define VDD_29_30    (1 << 17)
#define VDD_30_31    (1 << 18)
#define VDD_31_32    (1 << 19)
#define VDD_32_33    (1 << 20)
#define VDD 33_34    (1 << 21)
#define VDD_34_35    (1 << 22)
#define VDD_35_36    (1 << 23)
#define VDD_S18A     (1 << 24)

#define VDD_MASK        ( VDD_27_28 | VDD_28_29 | VDD_29_30 | \
                          VDD_30_31 | VDD_31_32 | VDD_32_33 | \
                          VDD 33_34 | VDD_34_35 | VDD_35_36 )
#define VDD_RANGE_27_30 ( VDD_27_28 | VDD_28_29 | VDD_29_30 )
#define VDD_RANGE_30_33 ( VDD_30_31 | VDD_31_32 | VDD_32_33 )
#define VDD_RANGE_33_36 ( VDD 33_34 | VDD_34_35 | VDD_35_36 )

#define CARD_UHS_II_STATUS          (1 << 29)
#define CARD_CAPACITY_STATUS        (1 << 30)
#define CARD_POWER_UP_BUSY          (1 << 31)

// Physical Layer Simplified Specification Version 4.10 "Card Status" bits from Table 4-41
#define SDCARD_OUT_OF_RANGE         (1 << 31)
#define SDCARD_ADDRESS_ERROR        (1 << 30)
#define SDCARD_BLOCK_LEN_ERROR      (1 << 29)
#define SDCARD_ERASE_SEQ_ERROR      (1 << 28)
#define SDCARD_ERASE_PARAM          (1 << 27)
#define SDCARD_WP_VIOLATION         (1 << 26)
#define SDCARD_CARD_IS_LOCKED       (1 << 25)
#define SDCARD_LOCK_UNLOCK_FAILED   (1 << 24)
#define SDCARD_COM_CRC_ERROR        (1 << 23)
#define SDCARD_ILLEGAL_COMMAND      (1 << 22)
#define SDCARD_CARD_ECC_FAILED      (1 << 21)
#define SDCARD_CC_ERROR             (1 << 20)
#define SDCARD_ERROR                (1 << 19)
#define SDCARD_CSD_OVERWRITE        (1 << 16)
#define SDCARD_WP_ERASE_SKIP        (1 << 15)
#define SDCARD_CARD_ECC_DISABLED    (1 << 14)
#define SDCARD_ERASE_RESET          (1 << 13)
#define SDCARD_CURRENT_STATE        (0xF << 9)
#define SDCARD_READY_FOR_DATA       (1 << 8)
#define SDCARD_APP_CMD              (1 << 5)
#define SDCARD_AKE_SEQ_ERROR        (1 << 3)

// RCA RESPONSE STATUS BITS
#define RCASTATUS_COM_CRC_ERROR     (1 << 15)
#define RCASTATUS_ILLEGAL_COMMAND   (1 << 14)
#define RCASTATUS_ERROR             (1 << 13)
#define RCASTATUS_CURRENT_STATE     (SDCARD_CURRENT_STATE)
#define RCASTATUS_READY_FOR_DATA    (SDCARD_READY_FOR_DATA)
#define RCASTATUS_APP_CMD           (SDCARD_APP_CMD)
#define RCASTATUS_AKE_SEQ_ERROR     (SDCARD_AKE_SEQ_ERROR)

#define RCAERROR_MSK    (RCASTATUS_COM_CRC_ERROR | RCASTATUS_ILLEGAL_COMMAND | RCASTATUS_ERROR)

#define NUM_INIT_ATTEMPTS 2

bool sd_card_bus_init(sdhc_t* hostctrl_p);
bool sd_read_single_block(sdcard_t* card_p, uint8_t* data_p, uint32_t addr);
bool sd_read_multiple_block(sdcard_t* card_p);
bool sd_stop_transmission(sdcard_t* card_p);

#endif // __SD_H
