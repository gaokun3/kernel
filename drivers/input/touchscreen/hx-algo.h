/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Himax HX83121A touch algorithm — data structures and interface.
 *
 * All algorithm state lives in struct hx_algo, which is allocated once in
 * probe via devm_kzalloc and referenced from struct himax_ts_data.
 */
#ifndef HX_ALGO_H
#define HX_ALGO_H

#include <linux/input/mt.h>
#include <linux/types.h>

/* Grid dimensions — must match the firmware raw frame layout. */
#define HX_ROWS        40
#define HX_COLS        60
#define HX_PIXELS      (HX_ROWS * HX_COLS)  /* 2400 */

/* Detection limits */
#define HX_MAX_ZONES   20
#define HX_MAX_PEAKS   20
#define HX_ZONE_PX_MAX 300  /* pixels stored per macro-zone; palm zones exceed
			     * this but are caught before the limit matters */

/* Maximum simultaneous reported contacts. */
#define HIMAX_MAX_TOUCH 10

/**
 * struct hx_peak - single detected local-maximum candidate.
 * @r:          row in the grid (0-based)
 * @c:          column in the grid
 * @z:          signal value at the peak
 * @nbr_sum:    sum of all 8-neighbour signals (used for Z8 filter)
 * @zone_area:  area of the macro-zone this peak belongs to
 */
struct hx_peak {
	u8  r;
	u8  c;
	s16 z;
	s32 nbr_sum;
	u16 zone_area;
};

/**
 * struct hx_contact - sub-pixel coordinate after centroid expansion.
 * @x, @y:      Q8.8 fixed-point grid coordinates
 * @area:       number of pixels contributing to this contact
 * @signal_sum: integrated signal over the contact area
 * @is_edge:    true when the centroid is within one cell of the grid boundary
 */
struct hx_contact {
	s32  x;
	s32  y;
	u16  area;
	s32  signal_sum;
	bool is_edge;
};

/**
 * struct hx_track - persistent touch slot state.
 * @active:     slot is in use
 * @x, @y:      current output coordinates [0, 65535]
 * @vx, @vy:    velocity in output units per frame (for prediction)
 * @signal_sum: integrated signal (forwarded to pressure reporting)
 * @age:        frames the slot has been active
 * @missed:     consecutive frames the slot had no matching detection
 * @debounce:   remaining debounce frames before the slot is reported
 */
struct hx_track {
	bool active;
	s32  x;
	s32  y;
	s32  vx;
	s32  vy;
	s32  signal_sum;
	u8   age;
	u8   missed;
	u8   debounce;
};

/**
 * struct hx_macro_zone - contiguous above-threshold region.
 * @pixels:     1-D indices (r*HX_COLS+c) of up to HX_ZONE_PX_MAX pixels
 * @area:       total pixel count (may exceed HX_ZONE_PX_MAX for palm zones)
 * @signal_sum: sum of positive pixel values within the zone
 * @min_r … max_c: bounding box
 */
struct hx_macro_zone {
	u16 pixels[HX_ZONE_PX_MAX];
	u16 area;
	s32 signal_sum;
	u8  min_r;
	u8  max_r;
	u8  min_c;
	u8  max_c;
};

/**
 * struct hx_algo - all algorithm state, allocated once in probe.
 *
 * Memory budget: ~40 KB.  Never allocate on the stack.
 */
struct hx_algo {
	/* ---- Frame buffers ---- */
	s16 frame[HX_ROWS][HX_COLS];        /* baseline-subtracted signal  */
	s16 iir_history[HX_ROWS][HX_COLS];  /* IIR temporal-filter history */
	bool iir_initialized;

	/* ---- Scratch buffers (shared between pipeline stages) ---- */
	u8  visited[HX_PIXELS];              /* BFS visited flags           */
	u8  zone_map[HX_PIXELS];             /* per-pixel zone-ID map       */
	u16 bfs_queue[HX_PIXELS];            /* ring-buffer BFS queue       */

	/* ---- Detection results ---- */
	struct hx_macro_zone zones[HX_MAX_ZONES];
	u8   zone_count;

	struct hx_peak peaks[HX_MAX_PEAKS];
	u8   peak_count;

	struct hx_contact contacts[HIMAX_MAX_TOUCH];
	u8   contact_count;

	/* ---- Tracking state ---- */
	struct hx_track tracks[HIMAX_MAX_TOUCH];
	bool touch_active;
	u8   touch_start_frames;

	/* ---- Tunable parameters (sysfs-writable, atomically updated) ---- */
	/* Preprocessing */
	bool cmf_enabled;          /* CMF on/off (default: true)           */
	s16  cmf_exclusion;        /* exclude pixels > this from CMF mean  */
	s16  cmf_max_correction;   /* clamp per-row/col offset             */
	bool iir_enabled;          /* IIR temporal filter on/off           */
	u16  iir_decay_weight;     /* blend weight 0-256 (256 = no blend)  */
	u16  iir_decay_step;       /* per-frame decay in signal units      */
	s16  iir_noise_floor;      /* clamp-to-zero below this             */
	s16  iir_gate_floor;       /* min dynamic threshold                */
	u8   iir_gate_ratio_q8;    /* dyn threshold = max * ratio/256      */
	/* Detection */
	s16  macro_threshold;      /* minimum pixel value to seed BFS      */
	s16  peak_threshold;       /* minimum peak signal                  */
	bool palm_enabled;         /* palm-rejection on/off                */
	u8   palm_area_threshold;  /* area >= this → palm                  */
	s32  palm_signal_threshold;/* signal_sum >= this → palm            */
	s16  palm_density_low;     /* signal/area < this → palm            */
	/* Pressure / touch-major reporting */
	bool pressure_enabled;     /* report PRESSURE + TOUCH_MAJOR        */
	/* Edge compensation */
	bool edge_comp_enabled;    /* edge compensation on/off             */
	s16  edge_boost_pct;       /* signal boost for border pixels (%)   */
	s16  edge_push_q8;         /* max outward push in Q8.8 (128=0.5)  */
	s16  edge_blend_q8;        /* blend range in Q8.8 (512=2 cells)   */
	/* Tracking */
	s32  track_dist2_max;      /* max squared distance for match       */
	u8   track_lost_frames;    /* missed frames before slot release    */
	u8   debounce_base;        /* new-slot debounce count              */
	bool track_smoothing;      /* position smoothing on/off            */
	bool track_active_guard;   /* kill stray tracks before 1st stable  */
	u8   track_start_debounce; /* frames to confirm touch_active       */
	s32  track_jump_dist2;     /* position jump → force lift+repress   */
};

/* ---- Public API ---- */

void hx_algo_init_defaults(struct hx_algo *algo);

/* Phase 1: preprocessing (baseline subtraction, CMF, IIR) */
void hx_preprocess_frame(struct hx_algo *algo, const u16 *raw);

/* Phase 2A: macro-zone detection */
void hx_detect_macro_zones(struct hx_algo *algo);

/* Phase 2B: palm rejection */
void hx_reject_palms(struct hx_algo *algo);

/* Phase 2C: peak detection within surviving zones */
void hx_detect_peaks(struct hx_algo *algo);

/* Phase 2D: zone expansion + weighted centroid → contacts → output positions */
void hx_expand_and_resolve(struct hx_algo *algo,
			    struct input_mt_pos *pos, int *cnt);

/* Phase 3A: greedy tracker update (with velocity prediction) */
void hx_track_contacts(struct hx_algo *algo,
		       struct input_mt_pos *det, int det_cnt);

/* Count slots that have passed debounce */
int hx_count_stable_tracks(struct hx_algo *algo);

#endif /* HX_ALGO_H */
