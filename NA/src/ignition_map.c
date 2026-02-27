/**
 * @file    ignition_map.c
 * @brief   Ignition Calibration Tables — Hyundai Tau G8BA (Racing Setup)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Calibration baseline (requires dyno verification before use)
 * ──────────────────────────────────────────────────────────────────────────
 *   Engine : Tau 4.6 G8BA V8, stock cams (D-CVVT @ 0° phasing as baseline)
 *   Fuel   : 98 RON premium pump fuel
 *   Note   : Advance values are MBT (Minimum advance for Best Torque) targets.
 *            Knock-limited regions must be pulled back during calibration;
 *            per-cylinder knock retard provides closed-loop correction.
 *
 *   ALL TABLE VALUES ARE FIRST-PASS ESTIMATES.
 *   Replace with measured MBT values from dyno sweeps before production use.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Soft-cut cylinder mask derivation
 * ──────────────────────────────────────────────────────────────────────────
 *   Firing order : 1-2-7-8-4-5-6-3
 *   Cyl indices  : 0-1-6-7-3-4-5-2
 *
 *   MASK_A fires firing-sequence positions 1,3,5,7 (half the cylinders):
 *     pos 1 → idx 0, pos 3 → idx 6, pos 5 → idx 3, pos 7 → idx 5
 *     bits 0,3,5,6 = 0b01101001 = 0x69
 *
 *   MASK_B fires firing-sequence positions 2,4,6,8 (the other half):
 *     pos 2 → idx 1, pos 4 → idx 7, pos 6 → idx 4, pos 8 → idx 2
 *     bits 1,2,4,7 = 0b10010110 = 0x96
 *
 *   Alternating MASK_A / MASK_B every medium-ctrl tick (10 ms) gives each
 *   cylinder a 50 % firing rate during soft cut.
 */

#include "ignition_map.h"
#include "g8ba_config.h"
#include <string.h>   /* memset */

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Engine speed axis (RPM) — 16 points, strictly monotone increasing */
const uint16_t IGNMAP_RPM_AXIS[IGNMAP_RPM_POINTS] = {
     600u, 800u, 1000u, 1500u, 2000u, 2500u, 3000u, 3500u,
    4000u, 4500u, 5000u, 5500u, 6000u, 6500u, 7000u, 7800u
};

/** MAP axis (kPa absolute) — 10 points, 20–105 kPa */
const uint8_t IGNMAP_MAP_AXIS[IGNMAP_MAP_POINTS] = {
    20u, 30u, 40u, 50u, 60u, 70u, 80u, 90u, 100u, 105u
};

/* ══════════════════════════════════════════════════════════════════════════
 * BASE ADVANCE TABLE  [IGNMAP_RPM_POINTS][IGNMAP_MAP_POINTS]
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Units: degrees BTDC (whole degrees, stored as uint8_t).
 *   - Low MAP (vacuum/cruise): higher advance acceptable (detonation-safe)
 *   - High MAP + high RPM: limited by knock tolerance (98 RON assumed)
 *   - Peak MBT region: ~4500–5000 RPM WOT ≈ 25–27 °BTDC for a 10.4:1 CR V8
 *   - Cranking advance is not in this table — ignition_control.c uses
 *     G8BA_IGN_ADVANCE_CRANK (5 °) directly during cranking mode.
 *
 * Columns:  MAP  20  30  40  50  60  70  80  90 100 105 kPa
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint8_t s_adv_table[IGNMAP_RPM_POINTS][IGNMAP_MAP_POINTS] = {
    /* RPM  600 */ { 14, 12, 10,  8,  7,  6,  5,  5,  5,  5 },
    /* RPM  800 */ { 18, 15, 12, 10,  9,  8,  7,  6,  6,  5 },
    /* RPM 1000 */ { 22, 18, 15, 12, 11, 10,  9,  8,  7,  6 },
    /* RPM 1500 */ { 26, 22, 18, 15, 13, 12, 11, 10,  9,  8 },
    /* RPM 2000 */ { 30, 26, 22, 18, 16, 14, 13, 12, 11, 10 },
    /* RPM 2500 */ { 34, 30, 26, 22, 18, 16, 15, 14, 13, 12 },
    /* RPM 3000 */ { 38, 34, 30, 26, 22, 19, 17, 16, 15, 14 },
    /* RPM 3500 */ { 40, 36, 33, 29, 26, 22, 20, 18, 17, 16 },
    /* RPM 4000 */ { 42, 39, 36, 32, 29, 26, 24, 22, 21, 20 },
    /* RPM 4500 */ { 44, 41, 38, 34, 31, 29, 27, 26, 25, 24 },
    /* RPM 5000 */ { 45, 42, 39, 36, 33, 31, 29, 28, 27, 26 },
    /* RPM 5500 */ { 45, 43, 40, 37, 34, 32, 30, 29, 28, 27 },
    /* RPM 6000 */ { 44, 42, 39, 36, 33, 31, 30, 28, 27, 26 },
    /* RPM 6500 */ { 43, 41, 38, 35, 32, 30, 28, 27, 26, 25 },
    /* RPM 7000 */ { 42, 40, 37, 34, 31, 29, 27, 26, 25, 24 },
    /* RPM 7800 */ { 40, 38, 35, 32, 29, 27, 25, 24, 23, 22 },
};

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile ign_cyl_knock_t g_ign_knock[G8BA_CYLINDERS];   /* IG-5: shared across threads */

/** Rev limiter internal state */
static struct {
    ign_rev_state_t state;
    uint8_t         mask_toggle;   /* 0 → MASK_A next, 1 → MASK_B next */
} s_rev;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Linear interpolation between y0 and y1, fraction in [0,1]. */
static float lerp(float y0, float y1, float frac)
{
    return y0 + frac * (y1 - y0);
}

/**
 * @brief  Bilinear interpolation on the 2-D advance table.
 *         Identical algorithm to fuel_map.c — kept local to avoid coupling.
 */
static float adv_bilinear(float rpm, float map_kpa)
{
    uint8_t ri = 0u, mi = 0u;

    /* --- RPM bounding index ----------------------------------------------- */
    for (uint8_t i = 0u; i < IGNMAP_RPM_POINTS - 1u; i++) {
        if (rpm < (float)IGNMAP_RPM_AXIS[i + 1u]) {
            ri = i;
            break;
        }
        ri = IGNMAP_RPM_POINTS - 2u;
    }

    /* --- MAP bounding index ----------------------------------------------- */
    for (uint8_t j = 0u; j < IGNMAP_MAP_POINTS - 1u; j++) {
        if (map_kpa < (float)IGNMAP_MAP_AXIS[j + 1u]) {
            mi = j;
            break;
        }
        mi = IGNMAP_MAP_POINTS - 2u;
    }

    /* --- Fractional positions --------------------------------------------- */
    float rpm_span = (float)(IGNMAP_RPM_AXIS[ri + 1u] - IGNMAP_RPM_AXIS[ri]);
    float map_span = (float)(IGNMAP_MAP_AXIS[mi + 1u] - IGNMAP_MAP_AXIS[mi]);

    float rfrac = 0.0f, mfrac = 0.0f;

    if (rpm_span > 0.0f) {
        rfrac = (rpm - (float)IGNMAP_RPM_AXIS[ri]) / rpm_span;
        if (rfrac < 0.0f) rfrac = 0.0f;
        if (rfrac > 1.0f) rfrac = 1.0f;
    }
    if (map_span > 0.0f) {
        mfrac = (map_kpa - (float)IGNMAP_MAP_AXIS[mi]) / map_span;
        if (mfrac < 0.0f) mfrac = 0.0f;
        if (mfrac > 1.0f) mfrac = 1.0f;
    }

    /* --- Bilinear blend --------------------------------------------------- */
    float v00 = (float)s_adv_table[ri    ][mi    ];
    float v10 = (float)s_adv_table[ri + 1u][mi    ];
    float v01 = (float)s_adv_table[ri    ][mi + 1u];
    float v11 = (float)s_adv_table[ri + 1u][mi + 1u];

    float lo = lerp(v00, v01, mfrac);
    float hi = lerp(v10, v11, mfrac);
    return lerp(lo, hi, rfrac);
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t ign_map_init(void)
{
    memset((void *)g_ign_knock, 0, sizeof(g_ign_knock));

    s_rev.state       = IGNMAP_REV_OFF;
    s_rev.mask_toggle = 0u;

    /*
     * IG-3 FIX: Axis monotonicity check runs in ALL builds (not just DEBUG).
     * Same rationale as FM-2 fix in fuel_map.c — Flash ECC corruption of axis
     * data produces wrong interpolation for every spark event, not just edge cases.
     */
    for (uint8_t i = 0u; i < IGNMAP_RPM_POINTS - 1u; i++) {
        if (IGNMAP_RPM_AXIS[i] >= IGNMAP_RPM_AXIS[i + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t j = 0u; j < IGNMAP_MAP_POINTS - 1u; j++) {
        if (IGNMAP_MAP_AXIS[j] >= IGNMAP_MAP_AXIS[j + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }

    return G8BA_OK;
}

float ign_map_get_base_advance(float rpm, float map_kpa)
{
    float adv = adv_bilinear(rpm, map_kpa);
    if (adv < 0.0f) adv = 0.0f;
    return adv;
}

float ign_map_get_advance(uint8_t cyl_index, float rpm, float map_kpa)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;

    float base    = adv_bilinear(rpm, map_kpa);
    float retard  = g_ign_knock[cyl_index].retard_deg;

    float net = base - retard;

    /* Clamp: never retard past TDC (0 °), never advance past hardware max */
    if (net < 0.0f)                  net = 0.0f;
    if (net > G8BA_IGN_ADVANCE_MAX)  net = G8BA_IGN_ADVANCE_MAX;

    return net;
}

void ign_map_knock_event(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    ign_cyl_knock_t *k = &g_ign_knock[cyl_index];

    /*
     * Apply immediate retard.
     * This function may be called from knock_control.c which runs in
     * thd_fast_ctrl (NORMALPRIO+10).  ign_map_get_advance() is called from
     * thd_medium_ctrl (NORMALPRIO).  On Cortex-M7, aligned float stores are
     * single-instruction (32-bit aligned, 4-byte type) — atomic.
     * No mutex needed here; worst case one tick reads a slightly stale value.
     */
    float new_retard = k->retard_deg + IGNMAP_KNOCK_RETARD_STEP_DEG;

    if (new_retard > IGNMAP_KNOCK_MAX_RETARD_DEG) {
        new_retard = IGNMAP_KNOCK_MAX_RETARD_DEG;
    }

    k->retard_deg   = new_retard;
    k->knock_events++;
    k->clean_cycles = 0u;   /* reset recovery counter */
}

void ign_map_clean_cycle(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    ign_cyl_knock_t *k = &g_ign_knock[cyl_index];

    if (k->retard_deg <= 0.0f) {
        k->retard_deg = 0.0f;   /* clamp negative drift */
        return;
    }

    /*
     * IG-1 FIX: Critical section around the retard_deg read-modify-write.
     *
     * Race condition:
     *   This function runs in thd_medium_ctrl (NORMALPRIO).
     *   ign_map_knock_event() runs in thd_fast_ctrl (NORMALPRIO+10) and can
     *   preempt this thread between the load of retard_deg and the store of
     *   new_retard.
     *
     * Failure mode without the fix:
     *   1. clean_cycle reads retard_deg = 5.0
     *   2. thd_fast_ctrl preempts — knock_event stores retard_deg = 7.0
     *   3. clean_cycle resumes, stores new_retard = 4.5 (overwrites 7.0!)
     *   → The knock retard increase is silently lost; engine may run 2.5°
     *     more advanced than intended into a knock-susceptible region.
     *
     * Fix: chSysLock() disables the ChibiOS scheduler (and PendSV) for the
     * duration of the RMW, preventing preemption by thd_fast_ctrl.
     * The critical section is ≤ 4 float instructions on Cortex-M7 FPU —
     * negligible latency impact.
     */
    chSysLock();
    float new_retard = k->retard_deg - IGNMAP_KNOCK_ADVANCE_STEP_DEG;
    if (new_retard < 0.0f) new_retard = 0.0f;
    k->retard_deg = new_retard;
    chSysUnlock();

    k->clean_cycles++;   /* diagnostic counter — outside CS, non-critical */
}

uint8_t ign_map_update_rev_limiter(rpm_t rpm)
{
    uint8_t mask;

    if (rpm >= IGNMAP_HARD_CUT_RPM) {
        /* ── Hard cut — all cylinders disabled ──────────────────────── */
        s_rev.state = IGNMAP_REV_HARD;
        mask = 0x00u;

    } else if (rpm >= IGNMAP_SOFT_CUT_RPM) {
        /* ── Soft cut — alternating mask, changes each call ─────────── */
        s_rev.state = IGNMAP_REV_SOFT;

        s_rev.mask_toggle ^= 1u;
        mask = (s_rev.mask_toggle != 0u) ? IGNMAP_SOFT_CUT_MASK_B
                                          : IGNMAP_SOFT_CUT_MASK_A;

    } else if (rpm < (IGNMAP_SOFT_CUT_RPM - IGNMAP_SOFT_CUT_HYST_RPM)) {
        /* ── Below hysteresis threshold — clear rev limit ────────────── */
        s_rev.state       = IGNMAP_REV_OFF;
        s_rev.mask_toggle = 0u;
        mask = IGNMAP_ALL_CYL_MASK;

    } else {
        /*
         * Inside hysteresis band [SOFT_CUT_RPM - HYST … SOFT_CUT_RPM).
         * Maintain whatever mask was last applied.
         * This prevents rapid oscillation if RPM hovers near the threshold.
         */
        if (s_rev.state == IGNMAP_REV_SOFT) {
            mask = (s_rev.mask_toggle != 0u) ? IGNMAP_SOFT_CUT_MASK_B
                                              : IGNMAP_SOFT_CUT_MASK_A;
        } else {
            mask = IGNMAP_ALL_CYL_MASK;
        }
    }

    return mask;
}

ign_rev_state_t ign_map_get_rev_state(void)
{
    return s_rev.state;
}

float ign_map_get_cyl_retard(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return g_ign_knock[cyl_index].retard_deg;
}
