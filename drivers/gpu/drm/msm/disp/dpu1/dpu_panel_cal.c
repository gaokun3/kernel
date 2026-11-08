// SPDX-License-Identifier: GPL-2.0-only
/*
 * Factory panel calibration of the Huawei MateBook E Go (gaokun3).
 *
 * The firmware reads the factory 3D LUTs from the DPP partition and publishes
 * them in an ACPI table with the signature "DLUT". The kernel boots from a
 * device tree here, so ACPI is not parsed, but ACPI reclaim memory stays mapped
 * and the table can be reached from the EFI configuration table.
 *
 * Each record is a 17x17x17 LUT for one gamut. The DSPP 3D LUT has 13 points
 * per axis in the mode that keeps two banks, so resample both and load them
 * once: the panel then switches between sRGB and Display P3 per frame.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <linux/efi.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/pfn.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "dpu_kms.h"
#include "dpu_hw_dspp.h"
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
#define DLUT_VAL_MAX		4095

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

/* Reject tables that cannot be a calibration of this panel */
static int dpu_panel_cal_check(const u8 *rec)
{
	u32 white[3], prim[3];
	int k;

	dpu_panel_cal_node(rec, DLUT_NODE_MAX, DLUT_NODE_MAX, DLUT_NODE_MAX, white);
	for (k = 0; k < 3; k++) {
		u32 at[3] = { 0, 0, 0 };

		at[k] = DLUT_NODE_MAX;
		dpu_panel_cal_node(rec, at[0], at[1], at[2], prim);
		if (white[k] < DLUT_VAL_MAX / 2 || prim[k] <= prim[(k + 1) % 3] ||
		    prim[k] <= prim[(k + 2) % 3])
			return -EINVAL;
	}

	return 0;
}

/*
 * Node n of a 13-point axis sits at 16 n / 12 of the 17-point grid: blend the
 * eight 17-point nodes around it, with weights in twelfths.
 */
static void dpu_panel_cal_resample(const u8 *rec, struct dpu_hw_gamut_lut *lut)
{
	u32 n[3], base[3], frac[3], v[3];
	int c, k;

	for (n[2] = 0; n[2] < DPU_GAMUT_GRID; n[2]++) {
		for (n[1] = 0; n[1] < DPU_GAMUT_GRID; n[1]++) {
			for (n[0] = 0; n[0] < DPU_GAMUT_GRID; n[0]++) {
				u32 acc[3] = { 0, 0, 0 };
				u16 *out;

				for (k = 0; k < 3; k++) {
					base[k] = min_t(u32, n[k] * DLUT_NODE_MAX /
							(DPU_GAMUT_GRID - 1),
							DLUT_NODE_MAX - 1);
					frac[k] = n[k] * DLUT_NODE_MAX -
						  base[k] * (DPU_GAMUT_GRID - 1);
				}

				for (c = 0; c < 8; c++) {
					u32 w = 1;

					for (k = 0; k < 3; k++)
						w *= (c >> k & 1) ? frac[k] :
						     DPU_GAMUT_GRID - 1 - frac[k];
					if (!w)
						continue;

					dpu_panel_cal_node(rec, base[0] + (c & 1),
							   base[1] + (c >> 1 & 1),
							   base[2] + (c >> 2 & 1), v);
					for (k = 0; k < 3; k++)
						acc[k] += v[k] * w;
				}

				out = lut->node[(n[2] * DPU_GAMUT_GRID + n[1]) *
						DPU_GAMUT_GRID + n[0]];
				for (k = 0; k < 3; k++)
					out[k] = (acc[k] + 864) / 1728;
			}
		}
	}
}

/**
 * dpu_panel_cal_load - turn the factory calibration into 3D LUTs
 * @lut: receives a LUT per enum dpu_panel_cal_gamut
 *
 * Return: 0 on success, or a negative errno if this device has no usable
 * calibration, in which case the panel stays uncorrected.
 */
int dpu_panel_cal_load(struct dpu_hw_gamut_lut *lut)
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
		u32 white[3];

		if (memcmp(rec + DLUT_REC_MAGIC, "$HUA02", 6))
			continue;

		if (rec[DLUT_REC_TYPE] == DLUT_TYPE_SRGB)
			gamut = DPU_PANEL_CAL_SRGB;
		else if (rec[DLUT_REC_TYPE] == DLUT_TYPE_P3)
			gamut = DPU_PANEL_CAL_P3;
		else
			continue;

		ret = dpu_panel_cal_check(rec);
		if (ret)
			goto out;

		dpu_panel_cal_resample(rec, &lut[gamut]);
		found |= BIT(gamut);
		dpu_panel_cal_node(rec, DLUT_NODE_MAX, DLUT_NODE_MAX, DLUT_NODE_MAX, white);
		pr_info("factory calibration %d: white %u %u %u\n", gamut,
			white[0], white[1], white[2]);
	}

	ret = found == GENMASK(DPU_PANEL_CAL_GAMUTS - 1, 0) ? 0 : -EINVAL;
out:
	vfree(table);
	return ret;
}
