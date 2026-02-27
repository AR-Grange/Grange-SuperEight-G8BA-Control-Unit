/**
 * @file    ignition_map.h
 * @brief   Ignition Calibration Tables — Hyundai Tau G8BA (Racing Setup)
 *
 * This module owns ignition advance data and per-cylinder knock retard state.
 * ignition_control.c calls into this module to obtain final advance angles.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Advance table convention
 * ──────────────────────────────────────────────────────────────────────────
 *   - Values are BASE advance in °BTDC (positive = advance)
 *   - Table covers RPM 600–7800 (16 points) × MAP 20–105 kPa (10 points)
 *   - Final advance = base_table(RPM, MAP) − per_cyl_knock_retard
 *   - Maximum advance is clamped to G8BA_IGN_ADVANCE_MAX (45 °)
 *   - Minimum advance is clamped to 0 ° (no retard past TDC for safety)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Per-cylinder knock retard
 * ──────────────────────────────────────────────────────────────────────────
 *   - On knock event    : retard += IGNMAP_KNOCK_RETARD_STEP_DEG (immediate)
 *   - On clean cycle    : retard -= IGNMAP_KNOCK_ADVANCE_STEP_DEG (gradual)
 *   - Max accumulated   : IGNMAP_KNOCK_MAX_RETARD_DEG
 *   - Recovery is SLOW (0.5°/cycle) to avoid re-entering knock zone
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Rev limiter (ignition-cut method)
 * ──────────────────────────────────────────────────────────────────────────
 *   Soft cut (7600 RPM): alternating cylinder-mask ignition cut, 50 % power.
 *                        Fuel is NOT cut (prevents lean spike / cat heat).
 *   Hard cut (7800 RPM): all cylinders disabled. Fuel cut separately by
 *                        engine_protection.c.
 *
 *   Cylinder cut masks are firing-order-aware so that alternating banks
 *   share the torque reduction evenly:
 *     IGNMAP_SOFT_CUT_MASK_A — odd  firing positions in sequence (4 of 8)
 *     IGNMAP_SOFT_CUT_MASK_B — even firing positions in sequence (4 of 8)
 *
 *   Bit assignment in mask: bit N = cylinder index N (0-based, LSB = cyl 0)
 *   1 = cylinder ENABLED (fires),  0 = cylinder DISABLED (spark skipped).
 */

#ifndef IGNITION_MAP_H
#define IGNITION_MAP_H

#include "g8ba_config.h"   /* rpm_t, g8ba_status_t, G8BA_CYLINDERS, etc. */

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE DIMENSIONS
 * ══════════════════════════════════════════════════════════════════════════ */

#define IGNMAP_RPM_POINTS   16u
#define IGNMAP_MAP_POINTS   10u

/* ══════════════════════════════════════════════════════════════════════════
 * KNOCK RETARD CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Immediate retard applied per knock event (°CA).  Aggressive but safe. */
#define IGNMAP_KNOCK_RETARD_STEP_DEG    2.0f

/** Advance recovery per clean (knock-free) engine cycle (°CA).
 *  Kept intentionally slow: 4 × faster retard than recovery rate. */
#define IGNMAP_KNOCK_ADVANCE_STEP_DEG   0.5f

/** Maximum per-cylinder knock retard accumulation (°CA). */
#define IGNMAP_KNOCK_MAX_RETARD_DEG     15.0f

/* ══════════════════════════════════════════════════════════════════════════
 * REV LIMITER CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * Soft-cut start RPM — alternating ignition cut begins here.
 * Matches G8BA_RPM_SOFT_CUT (7600 RPM) from g8ba_config.h.
 */
#define IGNMAP_SOFT_CUT_RPM     G8BA_RPM_SOFT_CUT    /* 7600 RPM */

/**
 * Hard-cut RPM — all cylinders disabled above this speed.
 * Matches G8BA_RPM_MAX (7800 RPM) from g8ba_config.h.
 */
#define IGNMAP_HARD_CUT_RPM     G8BA_RPM_MAX          /* 7800 RPM */

/**
 * Hysteresis band below IGNMAP_SOFT_CUT_RPM before rev limit fully clears.
 * Prevents rapid oscillation around the soft-cut threshold.
 */
#define IGNMAP_SOFT_CUT_HYST_RPM   150u

/**
 * Cylinder enable masks for alternating soft-cut pattern.
 *
 * Firing order: 1-2-7-8-4-5-6-3  → cylinder indices 0,1,6,7,3,4,5,2
 *
 * MASK_A fires positions 1,3,5,7 in firing sequence (4 cylinders):
 *   fire-pos 1→cyl-idx 0, fire-pos 3→cyl-idx 6, fire-pos 5→cyl-idx 3,
 *   fire-pos 7→cyl-idx 5  → bits 0,3,5,6 set  = 0b01101001 = 0x69
 *
 * MASK_B fires positions 2,4,6,8 in firing sequence (4 cylinders):
 *   fire-pos 2→cyl-idx 1, fire-pos 4→cyl-idx 7, fire-pos 6→cyl-idx 4,
 *   fire-pos 8→cyl-idx 2  → bits 1,2,4,7 set  = 0b10010110 = 0x96
 *
 * The masks are complements: MASK_A | MASK_B = 0xFF, MASK_A & MASK_B = 0x00.
 * Alternating between them ensures each cylinder fires every other soft-cut
 * cycle (50 % average power reduction), distributed evenly across both banks.
 */
/* IG-4 FIX: Labels were previously swapped ("even"/"odd" were backwards).
 * MASK_A enables ODD  firing-sequence positions (1,3,5,7) → cylinders 1,7,4,6.
 * MASK_B enables EVEN firing-sequence positions (2,4,6,8) → cylinders 2,8,5,3. */
#define IGNMAP_SOFT_CUT_MASK_A  0x69u   /**< Enable ODD  firing positions (1,3,5,7) */
#define IGNMAP_SOFT_CUT_MASK_B  0x96u   /**< Enable EVEN firing positions (2,4,6,8) */

/** Mask enabling ALL cylinders — used to exit soft cut. */
#define IGNMAP_ALL_CYL_MASK     0xFFu

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Per-cylinder knock retard state (opaque to callers; exposed for logging) */
typedef struct {
    float    retard_deg;    /**< Current accumulated retard (0 … MAX)    */
    uint16_t knock_events;  /**< Total knock events on this cylinder      */
    uint16_t clean_cycles;  /**< Consecutive clean cycles (for recovery)  */
} ign_cyl_knock_t;

/** Rev limiter operating state */
typedef enum {
    IGNMAP_REV_OFF = 0,    /**< Below soft-cut threshold                 */
    IGNMAP_REV_SOFT,       /**< 7600–7799 RPM — alternating cylinder cut */
    IGNMAP_REV_HARD,       /**< ≥ 7800 RPM — all cylinders cut           */
} ign_rev_state_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE (read-only for callers)
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * IG-5 FIX: volatile required — retard_deg is written by thd_fast_ctrl
 * (ign_map_knock_event) and read by thd_medium_ctrl (ign_map_get_advance,
 * ign_map_clean_cycle).  Without volatile the compiler may cache the value
 * in a register across the thread boundary within a translation unit.
 */
extern volatile ign_cyl_knock_t g_ign_knock[G8BA_CYLINDERS];

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXIS DECLARATIONS (defined in ignition_map.c)
 * ══════════════════════════════════════════════════════════════════════════ */

extern const uint16_t IGNMAP_RPM_AXIS[IGNMAP_RPM_POINTS];
extern const uint8_t  IGNMAP_MAP_AXIS[IGNMAP_MAP_POINTS];

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise ignition map module.
 *         Clears per-cylinder knock retard, validates table axes.
 * @return G8BA_OK always; G8BA_ERR_RANGE if axis sanity fails (debug only).
 */
g8ba_status_t ign_map_init(void);

/**
 * @brief  Look up base advance angle from RPM × MAP table (°BTDC).
 *         Performs bilinear interpolation; clamps to table range.
 * @param  rpm      Engine speed (RPM)
 * @param  map_kpa  Manifold pressure (kPa)
 * @return Base advance in °BTDC (e.g. 28.5).  Always ≥ 0.
 */
float ign_map_get_base_advance(float rpm, float map_kpa);

/**
 * @brief  Compute net advance for a specific cylinder.
 *         net = base_advance(RPM, MAP) − per_cylinder_knock_retard
 *         Clamped to [0.0, G8BA_IGN_ADVANCE_MAX].
 * @param  cyl_index  0-based cylinder index (0 = Cyl1 … 7 = Cyl8)
 * @param  rpm        Engine speed (RPM)
 * @param  map_kpa    Manifold pressure (kPa)
 * @return Net advance in °BTDC.
 */
float ign_map_get_advance(uint8_t cyl_index, float rpm, float map_kpa);

/**
 * @brief  Register a knock event on a specific cylinder.
 *         Immediately applies IGNMAP_KNOCK_RETARD_STEP_DEG retard.
 *         Resets the clean-cycle counter for this cylinder.
 *         Thread-safe (uses per-cylinder atomic update; no mutex required
 *         because retard_deg is only written here and read in ign_map_get_advance).
 *
 * @param  cyl_index  0-based cylinder index
 */
void ign_map_knock_event(uint8_t cyl_index);

/**
 * @brief  Register a clean (knock-free) engine cycle for a cylinder.
 *         Recovers IGNMAP_KNOCK_ADVANCE_STEP_DEG per call.
 *         Should be called once per cylinder per 720° cycle (from thd_medium_ctrl).
 * @param  cyl_index  0-based cylinder index
 */
void ign_map_clean_cycle(uint8_t cyl_index);

/**
 * @brief  Update rev limiter state from current RPM.
 *         Should be called every TASK_PERIOD_MEDIUM_MS (10 ms).
 *
 * @param  rpm  Current engine speed (RPM)
 * @return Cylinder enable mask to pass to ignition_set_soft_cut_mask():
 *           IGNMAP_ALL_CYL_MASK (0xFF)  — no cut active
 *           IGNMAP_SOFT_CUT_MASK_A/B    — alternating soft cut
 *           0x00                         — hard cut, all cylinders disabled
 */
uint8_t ign_map_update_rev_limiter(rpm_t rpm);

/**
 * @brief  Return current rev limiter state.
 */
ign_rev_state_t ign_map_get_rev_state(void);

/**
 * @brief  Return current accumulated knock retard for a cylinder (°CA).
 * @param  cyl_index  0-based cylinder index
 * @return Retard in degrees (0.0 = no retard active).
 */
float ign_map_get_cyl_retard(uint8_t cyl_index);

#endif /* IGNITION_MAP_H */
