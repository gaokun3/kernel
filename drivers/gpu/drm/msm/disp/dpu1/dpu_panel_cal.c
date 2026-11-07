// SPDX-License-Identifier: GPL-2.0-only
/*
 * Factory panel calibration of the Huawei MateBook E Go (gaokun3).
 *
 * The firmware reads the factory 3D LUTs from the DPP partition and publishes
 * them in an ACPI table with the signature "DLUT". The kernel boots from a
 * device tree here, so ACPI is not parsed, but ACPI reclaim memory stays mapped
 * and the table can be reached from the EFI configuration table.
 *
 * The driver has no 3D LUT support, so reduce each LUT to a 3x3 matrix for the
 * PCC block. PCC works on gamma-encoded pixels, and near grey the best such
 * matrix equals the linear-light one: take the LUT response to each primary
 * into linear light, normalize each row and scale it to the LUT white.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <linux/efi.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/pfn.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "dpu_panel_cal.h"

#define ACPI_HDR_LEN		36
#define RSDP_LEN		36
#define RSDP_XSDT_OFF		24

/* 52-byte table header, then the records at a fixed stride */
#define DLUT_LEN		0x20034
#define DLUT_HDR_LEN		52
#define DLUT_REC_COUNT		2
#define DLUT_REC_STEP		0x6400
#define DLUT_REC_MAGIC		0x10
#define DLUT_REC_TYPE		0x50
#define DLUT_REC_LUT		0x90
#define DLUT_TYPE_SRGB		1
#define DLUT_TYPE_P3		2

/* Big-endian [R low 8 | G 12 | B 12] words, then a plane of R high 4 bits */
#define DLUT_GRID		17
#define DLUT_NODES		(DLUT_GRID * DLUT_GRID * DLUT_GRID)
#define DLUT_NODE_MAX		16
/* High enough to be accurate, low enough to stay clear of gamut clipping */
#define DLUT_NODE_MID		12
#define DLUT_VAL_MAX		4095

/* (i / 32) ^ 2.2 in Q15, the panel gamma the LUTs are built for */
static const u16 dpu_panel_cal_pow[33] = {
	0, 16, 74, 179, 338, 552, 824, 1157,
	1552, 2011, 2536, 3127, 3787, 4516, 5316, 6188,
	7132, 8149, 9241, 10408, 11652, 12972, 14370, 15846,
	17401, 19037, 20752, 22549, 24427, 26387, 28431, 30557,
	32768,
};

static int dpu_panel_cal_read(phys_addr_t pa, void *dst, size_t len)
{
	if (!pfn_valid(PHYS_PFN(pa)) || !pfn_valid(PHYS_PFN(pa + len - 1)))
		return -EINVAL;

	return copy_from_kernel_nofault(dst, phys_to_virt(pa), len);
}

static int dpu_panel_cal_find(phys_addr_t *out)
{
	u8 rsdp[RSDP_LEN], hdr[ACPI_HDR_LEN];
	phys_addr_t xsdt;
	u64 *entries;
	u32 len, n, i;
	int ret;

	ret = dpu_panel_cal_read(efi.acpi20, rsdp, sizeof(rsdp));
	if (ret)
		return ret;
	if (memcmp(rsdp, "RSD PTR ", 8))
		return -ENODEV;

	xsdt = get_unaligned_le64(rsdp + RSDP_XSDT_OFF);
	ret = dpu_panel_cal_read(xsdt, hdr, sizeof(hdr));
	if (ret)
		return ret;
	if (memcmp(hdr, "XSDT", 4))
		return -ENODEV;

	len = get_unaligned_le32(hdr + 4);
	if (len <= ACPI_HDR_LEN || len > PAGE_SIZE)
		return -EINVAL;

	n = (len - ACPI_HDR_LEN) / sizeof(u64);
	entries = kmalloc_array(n, sizeof(u64), GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	ret = dpu_panel_cal_read(xsdt + ACPI_HDR_LEN, entries, n * sizeof(u64));
	if (ret)
		goto out;

	ret = -ENOENT;
	for (i = 0; i < n; i++) {
		phys_addr_t pa = get_unaligned_le64(&entries[i]);

		if (dpu_panel_cal_read(pa, hdr, sizeof(hdr)))
			continue;
		if (!memcmp(hdr, "DLUT", 4) &&
		    get_unaligned_le32(hdr + 4) == DLUT_LEN) {
			*out = pa;
			ret = 0;
			break;
		}
	}
out:
	kfree(entries);
	return ret;
}

static void dpu_panel_cal_node(const u8 *rec, u32 r, u32 g, u32 b, u32 *rgb)
{
	u32 idx = (r * DLUT_GRID + g) * DLUT_GRID + b;
	u32 word = get_unaligned_be32(rec + DLUT_REC_LUT + idx * 4);
	u8 high = rec[DLUT_REC_LUT + DLUT_NODES * 4 + idx];

	rgb[0] = (high & 0xf) << 8 | word >> 24;
	rgb[1] = (word >> 12) & 0xfff;
	rgb[2] = word & 0xfff;
}

static u32 dpu_panel_cal_linearize(u32 val)
{
	u32 pos = val * 32;
	u32 i = min(pos / DLUT_VAL_MAX, 31U);
	u32 frac = pos - i * DLUT_VAL_MAX;
	u32 low = dpu_panel_cal_pow[i], high = dpu_panel_cal_pow[i + 1];

	return low + (high - low) * frac / DLUT_VAL_MAX;
}

static int dpu_panel_cal_reduce(const u8 *rec, s32 *cal)
{
	u32 white[3], node[3][3];
	int r, c;

	dpu_panel_cal_node(rec, DLUT_NODE_MAX, DLUT_NODE_MAX, DLUT_NODE_MAX, white);
	dpu_panel_cal_node(rec, DLUT_NODE_MID, 0, 0, node[0]);
	dpu_panel_cal_node(rec, 0, DLUT_NODE_MID, 0, node[1]);
	dpu_panel_cal_node(rec, 0, 0, DLUT_NODE_MID, node[2]);

	for (r = 0; r < 3; r++) {
		u32 row[3], sum = 0;

		for (c = 0; c < 3; c++) {
			row[c] = dpu_panel_cal_linearize(node[c][r]);
			sum += row[c];
		}
		if (!sum)
			return -EINVAL;

		for (c = 0; c < 3; c++)
			cal[r * 3 + c] = div_u64((u64)row[c] * white[r] * DPU_PANEL_CAL_ONE,
						 sum * DLUT_VAL_MAX);

		/* A sane calibration keeps every channel dominant in its row */
		if (cal[r * 3 + r] < DPU_PANEL_CAL_ONE / 2)
			return -EINVAL;
	}

	return 0;
}

/**
 * dpu_panel_cal_load - reduce the factory calibration to color matrices
 * @cal: receives a row-major Q15 matrix per enum dpu_panel_cal_gamut
 *
 * Return: 0 on success, or a negative errno if this device has no usable
 * calibration, in which case the panel stays uncorrected.
 */
int dpu_panel_cal_load(s32 (*cal)[DPU_PANEL_CAL_LEN])
{
	phys_addr_t pa;
	u8 *table;
	u32 i, found = 0, sum = 0;
	int ret;

	if (!efi_enabled(EFI_BOOT) || !of_machine_is_compatible("huawei,gaokun3"))
		return -ENODEV;
	if (efi.acpi20 == EFI_INVALID_TABLE_ADDR)
		return -ENODEV;

	ret = dpu_panel_cal_find(&pa);
	if (ret)
		return ret;

	table = vmalloc(DLUT_LEN);
	if (!table)
		return -ENOMEM;

	ret = dpu_panel_cal_read(pa, table, DLUT_LEN);
	if (ret)
		goto out;

	for (i = 0; i < DLUT_LEN; i++)
		sum += table[i];

	ret = -EINVAL;
	if (sum & 0xff)
		goto out;

	for (i = 0; i < DLUT_REC_COUNT; i++) {
		const u8 *rec = table + DLUT_HDR_LEN + i * DLUT_REC_STEP;
		enum dpu_panel_cal_gamut gamut;
		s32 *m;

		if (memcmp(rec + DLUT_REC_MAGIC, "$HUA02", 6))
			continue;

		if (rec[DLUT_REC_TYPE] == DLUT_TYPE_SRGB)
			gamut = DPU_PANEL_CAL_SRGB;
		else if (rec[DLUT_REC_TYPE] == DLUT_TYPE_P3)
			gamut = DPU_PANEL_CAL_P3;
		else
			continue;

		m = cal[gamut];
		ret = dpu_panel_cal_reduce(rec, m);
		if (ret)
			goto out;

		found |= BIT(gamut);
		pr_info("factory calibration %d: %d %d %d, %d %d %d, %d %d %d\n", gamut,
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	}

	ret = found == GENMASK(DPU_PANEL_CAL_GAMUTS - 1, 0) ? 0 : -EINVAL;
out:
	vfree(table);
	return ret;
}
