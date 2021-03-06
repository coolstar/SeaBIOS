/*-
 * Copyright (c) 2006 Bernd Walter.  All rights reserved.
 * Copyright (c) 2006 M. Warner Losh.  All rights reserved.
 * Copyright (c) 2013 - 2014 Sage Electronic Engineering.  All rights reserved.
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

/*
 * NOTE:  This is refactored code from FreeBSD modified to be OS agnostic and
 *           independent for use in bootloading.
 */

#ifndef __SD_UTILS_H
#define __SD_UTILS_H

#include <stdint.h>

// CSD -- decoded structure
typedef struct {
    uint32_t mid;
    uint8_t pnm[8];
    uint32_t psn;
    uint16_t oid;
    uint16_t mdt_year;
    uint8_t mdt_month;
    uint8_t prv;
    uint8_t fwrev;
} sd_cid_t;

// CSD -- decoded structure
typedef struct {
    uint8_t csd_structure;
    uint8_t spec_vers;
    uint16_t ccc;
    uint32_t taac;
    uint32_t nsac;
    uint32_t r2w_factor;
    uint32_t tran_speed;
    uint32_t read_bl_len;
    uint32_t write_bl_len;
    uint16_t c_size;
    uint8_t c_size_mult;
    uint32_t vdd_r_curr_min;
    uint32_t vdd_r_curr_max;
    uint32_t vdd_w_curr_min;
    uint32_t vdd_w_curr_max;
    uint32_t wp_grp_size;
    uint32_t erase_sector;
    uint64_t capacity;
    uint32_t read_bl_partial :1,
             read_blk_misalign :1,
             write_bl_partial :1,
             write_blk_misalign :1,
             dsr_imp :1,
             erase_blk_en :1,
             wp_grp_enable :1;
} sd_csd_t;

/*
 * EXT CSD -- decoded structure
 *   this only contains a subset of the entire 512 byte structure
 */
typedef struct {
    // Properties segment (320 bytes)
    uint8_t min_perf_ddr_w;
    uint8_t min_perf_ddr_r;
    uint8_t min_perf_w_8_52;
    uint8_t min_perf_r_8_52;
    uint8_t min_perf_w_8_26_4_52;
    uint8_t min_perf_r_8_26_4_52;
    uint8_t min_perf_w_4_26;
    uint8_t min_perf_r_4_26;
    uint32_t sec_count;
    uint8_t device_type;
    uint8_t csd_structure;
    uint8_t ext_csd_rev;

    // Modes segment (192 bytes)
    uint8_t cmd_set;
    uint8_t cmd_set_rev;
    uint8_t power_class;
    uint8_t hs_timing;
    uint8_t bus_width;
    uint32_t max_enh_size_mult;
    uint32_t gp_size_mult1;
    uint32_t gp_size_mult2;
    uint32_t gp_size_mult3;
    uint32_t enh_size_mult;
    uint32_t enh_size_addr;

    // Calculated values
    uint64_t capacity;
} ext_csd_t;

void decode_cid_sd(uint32_t *raw_cid, sd_cid_t* cid);
void decode_csd_sd(uint32_t *raw_csd, sd_csd_t* csd);
void decode_ext_csd(uint32_t *raw_ext_csd, ext_csd_t* ext_csd);

#endif
