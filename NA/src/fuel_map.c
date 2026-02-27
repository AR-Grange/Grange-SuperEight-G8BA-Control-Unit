/**
 * @file    fuel_map.c
 * @brief   Fuel Calibration Tables — Hyundai Tau G8BA (Racing Setup)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Calibration baseline (requires dyno verification before use)
 * ──────────────────────────────────────────────────────────────────────────
 *   Engine   : Tau 4.6 G8BA V8, stock block, aggressive cams, cold-air
 *   Fuel     : 98 RON premium pump fuel
 *   Injectors: 650 cc/min high-flow MPI, 300 kPa rail
 *   IAT ref  : 20 °C (293 K) — temperature base for charge-density correction
 *
 * All table values are first-pass estimates from volumetric models and
 * comparative data for similar displacement V8 engines with D-CVVT.
 * MUST be replaced with measured values from wide-band O2 and dyno sweeps.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * VE table convention
 * ──────────────────────────────────────────────────────────────────────────
 *   - Values represent TRUE volumetric efficiency (gas-exchange efficiency)
 *   - Range: 75–102 % (>100 % possible with cam tuning / intake resonance)
 *   - MAP is a SEPARATE multiplicand in the PW formula:
 *       PW = BASE_PW × (VE/100) × (MAP/100) × corr + dead_time
 *   - Do NOT fold MAP into VE table values
 */

#include "fuel_map.h"

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Engine speed axis (RPM) — 16 points, strictly monotone increasing */
const uint16_t FUELMAP_RPM_AXIS[FUELMAP_RPM_POINTS] = {
     600u, 800u, 1000u, 1500u, 2000u, 2500u, 3000u, 3500u,
    4000u, 4500u, 5000u, 5500u, 6000u, 6500u, 7000u, 7800u
};

/** MAP axis (kPa absolute) — 10 points, 20–105 kPa */
const uint8_t FUELMAP_MAP_AXIS[FUELMAP_MAP_POINTS] = {
    20u, 30u, 40u, 50u, 60u, 70u, 80u, 90u, 100u, 105u
};

/** Battery voltage axis (V) for injector dead-time lookup — 9 points */
const float FUELMAP_VBATT_AXIS[FUELMAP_VBATT_POINTS] = {
    8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f
};

/** CLT axis (°C) for warm-up enrichment — 13 points */
const int8_t FUELMAP_CLT_AXIS[FUELMAP_CLT_POINTS] = {
    -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

/** IAT axis (°C) for charge-density correction — 11 points */
const int8_t FUELMAP_IAT_AXIS[FUELMAP_IAT_POINTS] = {
    -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80
};

/* ══════════════════════════════════════════════════════════════════════════
 * VE TABLE  [FUELMAP_RPM_POINTS][FUELMAP_MAP_POINTS]
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Units: percent (whole number).  True volumetric efficiency.
 * MAP axis does NOT scale the values — MAP is applied separately in PW calc.
 * VE variation with MAP captures cam-overlap effects at light vs. heavy load.
 *
 * Columns:  MAP  20  30  40  50  60  70  80  90 100 105 kPa
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint8_t s_ve_table[FUELMAP_RPM_POINTS][FUELMAP_MAP_POINTS] = {
    /* RPM  600 */ { 76, 76, 76, 76, 76, 77, 78, 79, 80, 80 },
    /* RPM  800 */ { 77, 77, 77, 77, 77, 78, 79, 81, 82, 83 },
    /* RPM 1000 */ { 78, 78, 78, 78, 78, 79, 81, 83, 85, 86 },
    /* RPM 1500 */ { 79, 79, 79, 79, 80, 81, 83, 86, 88, 89 },
    /* RPM 2000 */ { 80, 80, 80, 80, 81, 82, 85, 88, 91, 92 },
    /* RPM 2500 */ { 81, 81, 81, 82, 83, 85, 87, 90, 93, 94 },
    /* RPM 3000 */ { 82, 82, 83, 84, 85, 87, 90, 93, 96, 97 },
    /* RPM 3500 */ { 83, 83, 84, 86, 87, 90, 93, 96, 98, 99 },
    /* RPM 4000 */ { 84, 84, 85, 87, 89, 92, 95, 97, 99,100 },
    /* RPM 4500 */ { 85, 85, 86, 88, 91, 93, 96, 98,101,102 },  /* peak VE */
    /* RPM 5000 */ { 85, 85, 86, 88, 91, 93, 96, 98,100,101 },
    /* RPM 5500 */ { 84, 84, 85, 87, 89, 92, 94, 96, 98, 99 },
    /* RPM 6000 */ { 82, 82, 83, 85, 87, 90, 92, 93, 95, 96 },
    /* RPM 6500 */ { 80, 80, 81, 83, 85, 87, 89, 90, 92, 93 },
    /* RPM 7000 */ { 78, 78, 79, 81, 83, 85, 87, 87, 89, 89 },
    /* RPM 7800 */ { 75, 75, 76, 78, 80, 82, 83, 84, 85, 85 },
};

/* ══════════════════════════════════════════════════════════════════════════
 * INJECTOR DEAD-TIME TABLE  [FUELMAP_VBATT_POINTS]
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Injector opening delay vs. battery voltage (µs).
 * 650 cc/min injectors, measured at nominal rail pressure.
 * Decreases with increasing voltage (coil reaches saturation faster).
 * Units: µs (stored as uint16_t, converted to ms in lookup)
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint16_t s_dead_time_us[FUELMAP_VBATT_POINTS] = {
    /*  8.0 V */ 2200u,
    /*  9.0 V */ 1800u,
    /* 10.0 V */ 1400u,
    /* 11.0 V */ 1100u,
    /* 12.0 V */  900u,
    /* 13.0 V */  750u,
    /* 14.0 V */  650u,  /* typical running voltage */
    /* 15.0 V */  580u,
    /* 16.0 V */  520u,
};

/* ══════════════════════════════════════════════════════════════════════════
 * CLT CORRECTION TABLE  [FUELMAP_CLT_POINTS]
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Cold-start / warm-up enrichment multiplier × 100 (integer percent).
 * e.g. 140 → 1.40× fuel (40 % enrichment at -20 °C).
 * Unity (100) at ≥ 80 °C — engine fully warm.
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint8_t s_clt_corr_pct[FUELMAP_CLT_POINTS] = {
    /* -20 °C */ 140u,
    /* -10 °C */ 132u,
    /*   0 °C */ 125u,
    /*  10 °C */ 118u,
    /*  20 °C */ 112u,
    /*  30 °C */ 108u,
    /*  40 °C */ 105u,
    /*  50 °C */ 103u,
    /*  60 °C */ 102u,
    /*  70 °C */ 101u,
    /*  80 °C */ 100u,  /* warm — unity from here */
    /*  90 °C */ 100u,
    /* 100 °C */ 100u,
};

/* ══════════════════════════════════════════════════════════════════════════
 * IAT CORRECTION TABLE  [FUELMAP_IAT_POINTS]
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Charge-density correction multiplier × 100 (integer percent).
 * Reference temperature: 20 °C (293 K) — same as FUELMAP_BASE_PW_MS derivation.
 *   corr = T_ref_K / (IAT_C + 273) × 100 = 293 / (IAT + 273) × 100
 *
 * FM-1 FIX: Previous table had unity (100) at 30 °C, but BASE_PW is derived
 *   at 20 °C (293 K).  Unity MUST be at 20 °C.  Values above 30 °C were also
 *   computed with an inconsistent T_ref, producing systematic over-enrichment
 *   at all temperatures above 20 °C (up to ~6 % at 80 °C).
 *
 * Corrected values: corr(T_IAT) = round(29300 / (T_IAT + 273))
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint8_t s_iat_corr_pct[FUELMAP_IAT_POINTS] = {
    /* -20 °C */ 116u,   /* 293/253 = 1.158 */
    /* -10 °C */ 111u,   /* 293/263 = 1.114 */
    /*   0 °C */ 107u,   /* 293/273 = 1.073 */
    /*  10 °C */ 104u,   /* 293/283 = 1.035 */
    /*  20 °C */ 100u,   /* 293/293 = 1.000 ← REFERENCE POINT (matches BASE_PW) */
    /*  30 °C */  97u,   /* 293/303 = 0.967 */
    /*  40 °C */  94u,   /* 293/313 = 0.936 */
    /*  50 °C */  91u,   /* 293/323 = 0.907 */
    /*  60 °C */  88u,   /* 293/333 = 0.880 */
    /*  70 °C */  85u,   /* 293/343 = 0.854 */
    /*  80 °C */  83u,   /* 293/353 = 0.830 */
};

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  1-D linear interpolation between two float endpoints.
 * @param  y0    Value at the lower bound
 * @param  y1    Value at the upper bound
 * @param  frac  Fractional position in [0.0, 1.0]
 */
static float lerp(float y0, float y1, float frac)
{
    return y0 + frac * (y1 - y0);
}

/**
 * @brief  Bilinear interpolation on the 2-D VE table.
 *
 *         Searches the RPM and MAP axes for the bounding row/column pair,
 *         then performs bilinear (linear in both dimensions) interpolation.
 *         Input values outside the table range are clamped to the edge.
 *
 * @param  rpm      Engine speed (RPM)
 * @param  map_kpa  MAP (kPa)
 * @return Interpolated VE (%).
 */
static float ve_bilinear(float rpm, float map_kpa)
{
    uint8_t ri = 0u, mi = 0u;

    /* --- Find RPM bounding index (ri, ri+1) -------------------------------- */
    for (uint8_t i = 0u; i < FUELMAP_RPM_POINTS - 1u; i++) {
        if (rpm < (float)FUELMAP_RPM_AXIS[i + 1u]) {
            ri = i;
            break;
        }
        ri = FUELMAP_RPM_POINTS - 2u;   /* clamp to last interval */
    }

    /* --- Find MAP bounding index (mi, mi+1) -------------------------------- */
    for (uint8_t j = 0u; j < FUELMAP_MAP_POINTS - 1u; j++) {
        if (map_kpa < (float)FUELMAP_MAP_AXIS[j + 1u]) {
            mi = j;
            break;
        }
        mi = FUELMAP_MAP_POINTS - 2u;
    }

    /* --- Fractional positions in [0, 1] ------------------------------------ */
    float rpm_span = (float)(FUELMAP_RPM_AXIS[ri + 1u] - FUELMAP_RPM_AXIS[ri]);
    float map_span = (float)(FUELMAP_MAP_AXIS[mi + 1u] - FUELMAP_MAP_AXIS[mi]);

    float rfrac = 0.0f, mfrac = 0.0f;

    if (rpm_span > 0.0f) {
        rfrac = (rpm - (float)FUELMAP_RPM_AXIS[ri]) / rpm_span;
        if (rfrac < 0.0f) rfrac = 0.0f;
        if (rfrac > 1.0f) rfrac = 1.0f;
    }
    if (map_span > 0.0f) {
        mfrac = (map_kpa - (float)FUELMAP_MAP_AXIS[mi]) / map_span;
        if (mfrac < 0.0f) mfrac = 0.0f;
        if (mfrac > 1.0f) mfrac = 1.0f;
    }

    /* --- Bilinear blend ---------------------------------------------------- */
    float v00 = (float)s_ve_table[ri    ][mi    ];
    float v10 = (float)s_ve_table[ri + 1u][mi    ];
    float v01 = (float)s_ve_table[ri    ][mi + 1u];
    float v11 = (float)s_ve_table[ri + 1u][mi + 1u];

    float lo = lerp(v00, v01, mfrac);   /* interpolate along MAP at ri   */
    float hi = lerp(v10, v11, mfrac);   /* interpolate along MAP at ri+1 */
    return lerp(lo, hi, rfrac);          /* interpolate along RPM         */
}

/**
 * @brief  1-D linear interpolation on a float-axis / uint16_t-value table.
 *         Used for dead-time lookup.
 */
static float interp1d_u16(const float *x_axis, const uint16_t *y_vals,
                           uint8_t count, float x)
{
    if (x <= x_axis[0]) return (float)y_vals[0];
    if (x >= x_axis[count - 1u]) return (float)y_vals[count - 1u];

    for (uint8_t i = 0u; i < count - 1u; i++) {
        if (x < x_axis[i + 1u]) {
            float span = x_axis[i + 1u] - x_axis[i];
            float frac = (span > 0.0f) ? ((x - x_axis[i]) / span) : 0.0f;
            return lerp((float)y_vals[i], (float)y_vals[i + 1u], frac);
        }
    }
    return (float)y_vals[count - 1u];
}

/**
 * @brief  1-D linear interpolation on a int8_t-axis / uint8_t-percent table.
 *         Used for CLT and IAT corrections.
 * @return Correction factor as a float (e.g. 1.12 for 112 %).
 */
static float interp1d_corr(const int8_t *x_axis, const uint8_t *y_pct,
                             uint8_t count, float x)
{
    if (x <= (float)x_axis[0]) return (float)y_pct[0] * 0.01f;
    if (x >= (float)x_axis[count - 1u]) return (float)y_pct[count - 1u] * 0.01f;

    for (uint8_t i = 0u; i < count - 1u; i++) {
        float x0 = (float)x_axis[i];
        float x1 = (float)x_axis[i + 1u];
        if (x < x1) {
            float span = x1 - x0;
            float frac = (span > 0.0f) ? ((x - x0) / span) : 0.0f;
            return lerp((float)y_pct[i] * 0.01f,
                        (float)y_pct[i + 1u] * 0.01f, frac);
        }
    }
    return (float)y_pct[count - 1u] * 0.01f;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t fuel_map_init(void)
{
    /*
     * FM-2 FIX: Axis monotonicity check runs in ALL builds (not just DEBUG).
     *
     * Rationale: The STM32H743 Flash has ECC protection, but a multi-bit error
     * or a corrupted firmware image can silently flip axis values. A non-monotone
     * axis causes the bilinear bounding-index search to return a wrong interval,
     * producing an incorrect PW for every single operating point — potentially
     * injecting double or half the required fuel across the entire map.
     *
     * This check executes only once at boot (negligible cost) and returns
     * G8BA_ERR_RANGE to g8ba_init(), which halts the system via chSysHalt()
     * before any threads start, preventing an engine run with corrupted tables.
     */
    for (uint8_t i = 0u; i < FUELMAP_RPM_POINTS - 1u; i++) {
        if (FUELMAP_RPM_AXIS[i] >= FUELMAP_RPM_AXIS[i + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t j = 0u; j < FUELMAP_MAP_POINTS - 1u; j++) {
        if (FUELMAP_MAP_AXIS[j] >= FUELMAP_MAP_AXIS[j + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    return G8BA_OK;
}

float fuel_map_get_ve(float rpm, float map_kpa)
{
    return ve_bilinear(rpm, map_kpa);
}

float fuel_map_get_dead_time_ms(float vbatt_v)
{
    float dt_us = interp1d_u16(FUELMAP_VBATT_AXIS, s_dead_time_us,
                                FUELMAP_VBATT_POINTS, vbatt_v);
    return dt_us * 0.001f;   /* µs → ms */
}

float fuel_map_get_clt_correction(float clt_c)
{
    return interp1d_corr(FUELMAP_CLT_AXIS, s_clt_corr_pct,
                          FUELMAP_CLT_POINTS, clt_c);
}

float fuel_map_get_iat_correction(float iat_c)
{
    return interp1d_corr(FUELMAP_IAT_AXIS, s_iat_corr_pct,
                          FUELMAP_IAT_POINTS, iat_c);
}

void fuel_map_calc(const fuel_map_inputs_t *in, fuel_map_result_t *out)
{
    if ((in == NULL) || (out == NULL)) return;

    /* ── 1. VE lookup ─────────────────────────────────────────────────── */
    float ve_pct = ve_bilinear(in->rpm, in->map_kpa);
    out->ve_pct  = ve_pct;

    /* ── 2. Dead-time compensation ────────────────────────────────────── */
    float dt_ms      = fuel_map_get_dead_time_ms(in->vbatt_v);
    out->dead_time_ms = dt_ms;

    /* ── 3. Temperature corrections ───────────────────────────────────── */
    float clt_corr = fuel_map_get_clt_correction(in->clt_c);
    float iat_corr = fuel_map_get_iat_correction(in->iat_c);
    out->clt_corr  = clt_corr;
    out->iat_corr  = iat_corr;

    /* ── 4. Clamp lambda correction to safe band (0.80 … 1.20) ───────── */
    float lambda_corr = in->lambda_corr;
    if (lambda_corr < 0.80f) lambda_corr = 0.80f;
    if (lambda_corr > 1.20f) lambda_corr = 1.20f;

    /* ── 5. Base pulse width (effective open time, without dead-time) ─── */
    /*
     * PW_eff = BASE_PW × (VE/100) × (MAP/MAP_ref) × CLT × IAT × λ_corr
     *
     * Rationale for (MAP/MAP_ref):
     *   VE represents pure gas-exchange efficiency (dimensionless).
     *   Actual air mass ∝ MAP × VE × charge_density.
     *   At MAP_ref = 100 kPa, BASE_PW is calibrated.  At other MAP values
     *   we scale proportionally; charge density (IAT) is handled by iat_corr.
     */
    float pw_eff_ms = FUELMAP_BASE_PW_MS
                    * (ve_pct  * 0.01f)
                    * (in->map_kpa / MAP_REF_KPA)
                    * clt_corr
                    * iat_corr
                    * lambda_corr;

    /* Enforce minimum effective open time */
    if (pw_eff_ms < FUELMAP_MIN_OPEN_MS) {
        pw_eff_ms = FUELMAP_MIN_OPEN_MS;
    }
    out->pw_effective_ms = pw_eff_ms;

    /* ── 6. Add dead-time → total commanded pulse width ──────────────── */
    float pw_cmd = pw_eff_ms + dt_ms;

    /* Clamp to hardware-safe maximum */
    if (pw_cmd > FUELMAP_MAX_PW_MS) {
        pw_cmd = FUELMAP_MAX_PW_MS;
    }
    out->pw_cmd_ms = pw_cmd;
}
