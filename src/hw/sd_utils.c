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

#include "config.h"
#include "string.h"
#include "output.h"
#include "sd_utils.h"
#include "sd.h"

static uint32_t get_bits(uint32_t *bits, int bit_len, int start, int size);

static const int exp[8] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 };

static const int mant[16] = { 0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55,
        60, 70, 80 };

static const int cur_min[8] = { 500, 1000, 5000, 10000, 25000, 35000, 60000,
        100000 };

static const int cur_max[8] = { 1000, 5000, 10000, 25000, 35000, 45000, 800000,
        200000 };

static uint32_t get_bits(uint32_t *bits, int bit_len, int start, int size) {
    const int i = (bit_len / 32) - (start / 32) - 1;
    const int shift = start & 31;
    uint32_t retval = bits[i] >> shift;
    if (size + shift > 32)
        retval |= bits[i - 1] << (32 - shift);
    return (retval & ((1llu << size) - 1));
}

void decode_cid_sd(uint32_t *raw_cid, sd_cid_t* cid) {
    int i;

    // there's no version info, so we take it on faith
    memset(cid, 0, sizeof(*cid));
    cid->mid = get_bits(raw_cid, 128, 120, 8);
    cid->oid = get_bits(raw_cid, 128, 104, 16);
    for (i = 0; i < 5; i++)
        cid->pnm[i] = get_bits(raw_cid, 128, 96 - i * 8, 8);
    cid->pnm[5] = 0;
    cid->prv = get_bits(raw_cid, 128, 56, 8);
    cid->psn = get_bits(raw_cid, 128, 24, 32);
    cid->mdt_year = get_bits(raw_cid, 128, 12, 8) + 2000;
    cid->mdt_month = get_bits(raw_cid, 128, 8, 4);
}

void decode_csd_sd(uint32_t *raw_csd, sd_csd_t* csd) {
    int m;
    int e;

    memset(csd, 0, sizeof(*csd));
    csd->csd_structure = get_bits(raw_csd, 128, 126, 2);
    csd->spec_vers = get_bits(raw_csd, 128, 122, 4);
    m = get_bits(raw_csd, 128, 115, 4);
    e = get_bits(raw_csd, 128, 112, 3);
    csd->taac = (exp[e] * mant[m]) / 10;
    csd->nsac = get_bits(raw_csd, 128, 104, 8) * 100;
    m = get_bits(raw_csd, 128, 99, 4);
    e = get_bits(raw_csd, 128, 96, 3);
    csd->tran_speed = exp[e] * 10000 * mant[m];
    csd->ccc = get_bits(raw_csd, 128, 84, 12);
    csd->read_bl_len = 1 << get_bits(raw_csd, 128, 80, 4);
    csd->read_bl_partial = get_bits(raw_csd, 128, 79, 1);
    csd->write_blk_misalign = get_bits(raw_csd, 128, 78, 1);
    csd->read_blk_misalign = get_bits(raw_csd, 128, 77, 1);
    csd->dsr_imp = get_bits(raw_csd, 128, 76, 1);
    csd->c_size = get_bits(raw_csd, 128, 62, 12);
    csd->vdd_r_curr_min = cur_min[get_bits(raw_csd, 128, 59, 3)];
    csd->vdd_r_curr_max = cur_max[get_bits(raw_csd, 128, 56, 3)];
    csd->vdd_w_curr_min = cur_min[get_bits(raw_csd, 128, 53, 3)];
    csd->vdd_w_curr_max = cur_max[get_bits(raw_csd, 128, 50, 3)];
    csd->c_size_mult = get_bits(raw_csd, 128, 47, 3);
    csd->erase_blk_en = get_bits(raw_csd, 128, 46, 1);
    csd->erase_sector = get_bits(raw_csd, 128, 39, 7) + 1;
    csd->wp_grp_size = get_bits(raw_csd, 128, 32, 5);
    csd->wp_grp_enable = get_bits(raw_csd, 128, 31, 1);
    csd->r2w_factor = 1 << get_bits(raw_csd, 128, 26, 3);
    csd->write_bl_len = 1 << get_bits(raw_csd, 128, 22, 4);
    csd->write_bl_partial = get_bits(raw_csd, 128, 21, 1);

    // calculate the card capacity
    switch (csd->csd_structure) {
    case 0:
        m = get_bits(raw_csd, 128, 62, 12);
        e = get_bits(raw_csd, 128, 47, 3);
        csd->capacity = ((1 + m) << (e + 2)) * csd->read_bl_len;
        break;
    case 1:
    case 2:
    case 3:
    case 4:
        csd->capacity = ((uint64_t) get_bits(raw_csd, 128, 48, 22) + 1) * 512
                * 1024;
        break;
    default:
        break;
    }

    dprintf(DEBUG_HDL_SD, "CSD Register Info:\n");
    dprintf(DEBUG_HDL_SD, "  - CSD Version %d.0\n", csd->csd_structure + 1);
    dprintf(DEBUG_HDL_SD, "  - READ_BL_LEN: %d\n", csd->read_bl_len);
    dprintf(DEBUG_HDL_SD, "  - CAPACITY: 0x%08x%08x\n",
            (uint32_t )(csd->capacity >> 32), (uint32_t )(csd->capacity));
    dprintf(DEBUG_HDL_SD, "  - MAX FREQ: %d Hz\n", csd->tran_speed);
    dprintf(DEBUG_HDL_SD, "  - TAAC: %dns\n", csd->taac);
    dprintf(DEBUG_HDL_SD, "  - NSAC: %d clock cycles\n", csd->nsac);
}

static void print_performance_class(ext_csd_t* ext_csd) {
    uint8_t w, s;

    w = ext_csd->bus_width;
    s = ext_csd->hs_timing;

    if ((w == DDR_8BIT) && (s == HS_TIMING_HIGH_SPEED)) {
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_W_DDR: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_ddr_w],
                ext_csd->min_perf_ddr_w * 4 * 150);
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_R_DDR: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_ddr_r],
                ext_csd->min_perf_ddr_r * 4 * 150);
    } else if ((w == SDR_8BIT) && (s == HS_TIMING_HIGH_SPEED)) {
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_W_8_52: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_w_8_52],
                ext_csd->min_perf_w_8_52 * 2 * 150);
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_R_8_52: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_r_8_52],
                ext_csd->min_perf_r_8_52 * 2 * 150);
    } else if (((w == SDR_4BIT) && (s == HS_TIMING_HIGH_SPEED))
            || ((w == SDR_8BIT) && (s == HS_TIMING_COMPATIBILITY))) {
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_W_8_26_4_52: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_w_8_26_4_52],
                ext_csd->min_perf_w_8_26_4_52 * 2 * 150);
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_R_8_26_4_52: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_r_8_26_4_52],
                ext_csd->min_perf_r_8_26_4_52 * 2 * 150);
    } else if ((w == SDR_4BIT) && (s == HS_TIMING_COMPATIBILITY)) {
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_W_4_26: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_w_4_26],
                ext_csd->min_perf_w_4_26 * 2 * 150);
        dprintf(DEBUG_HDL_SD, "  - MIN_PERF_R_4_26: %s (%dKB/s)\n",
                sdMmcClassStr[ext_csd->min_perf_r_4_26],
                ext_csd->min_perf_r_4_26 * 2 * 150);
    }
}

void decode_ext_csd(uint32_t *raw_ext_csd, ext_csd_t *ext_csd) {
    memset(ext_csd, 0, sizeof(*ext_csd));
    // Properties segment - mostly RO
    ext_csd->min_perf_ddr_w = (uint8_t) (raw_ext_csd[MIN_PERF_DDR_W / 4]);
    ext_csd->min_perf_ddr_w = (uint8_t) (raw_ext_csd[MIN_PERF_DDR_R / 4] >> 8);
    ext_csd->min_perf_w_8_52 =
            (uint8_t) (raw_ext_csd[MIN_PERF_W_8_52 / 4] >> 16);
    ext_csd->min_perf_r_8_52 =
            (uint8_t) (raw_ext_csd[MIN_PERF_R_8_52 / 4] >> 8);
    ext_csd->min_perf_w_8_26_4_52 = (uint8_t) (raw_ext_csd[MIN_PERF_W_8_26_4_52
            / 4]);
    ext_csd->min_perf_r_8_26_4_52 = (uint8_t) (raw_ext_csd[MIN_PERF_R_8_26_4_52
            / 4] >> 24);
    ext_csd->min_perf_w_4_26 =
            (uint8_t) (raw_ext_csd[MIN_PERF_W_4_26 / 4] >> 16);
    ext_csd->min_perf_r_4_26 =
            (uint8_t) (raw_ext_csd[MIN_PERF_R_4_26 / 4] >> 8);
    ext_csd->sec_count = raw_ext_csd[SEC_COUNT / 4];
    ext_csd->capacity = (uint64_t) ext_csd->sec_count * 512;
    ext_csd->device_type = (uint8_t) (raw_ext_csd[DEVICE_TYPE / 4] & 0x1f);
    ext_csd->csd_structure = (uint8_t) ((raw_ext_csd[EXT_CSD_STRUCTURE / 4]
            >> 16) & 0x3);
    ext_csd->ext_csd_rev = (uint8_t) (raw_ext_csd[EXT_CSD_REV / 4] & 0x3f);

    // Modes segment - mostly R/W using CMD6 - Switch
    ext_csd->cmd_set = (uint8_t) raw_ext_csd[CMD_SET / 4];
    ext_csd->cmd_set_rev = (uint8_t) raw_ext_csd[CMD_SET_REV / 4];
    ext_csd->power_class = (uint8_t) raw_ext_csd[POWER_CLASS / 4] & 0xf;
    ext_csd->hs_timing = (uint8_t) raw_ext_csd[HS_TIMING / 4] & 0xf;
    ext_csd->bus_width = (uint8_t) raw_ext_csd[BUS_WIDTH / 4] & 0x3f;

    dprintf(DEBUG_HDL_SD, "EXT_CSD Register Info:\n");
    dprintf(DEBUG_HDL_SD, "  - EXT_CSD Version: 1.%d\n", ext_csd->ext_csd_rev);
    dprintf(DEBUG_HDL_SD, "  - SEC_COUNT: 0x%x\n", ext_csd->sec_count);
    dprintf(DEBUG_HDL_SD, "  - CAPACITY: 0x%08x%08x\n",
            (uint32_t )(ext_csd->capacity >> 32),
            (uint32_t )(ext_csd->capacity));
    dprintf(DEBUG_HDL_SD, "  - DEVICE TYPE: 0x%x\n", ext_csd->device_type);
    dprintf(DEBUG_HDL_SD, "  - CMD SET: %d\n", ext_csd->cmd_set);
    dprintf(DEBUG_HDL_SD, "  - HS TIMING: %d\n", ext_csd->hs_timing);
    dprintf(DEBUG_HDL_SD, "  - BUS WIDTH: %d\n", ext_csd->bus_width);
    print_performance_class(ext_csd);
}

