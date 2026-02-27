/**
 * @file    fuel_map.h
 * @brief   Fuel Calibration Tables — Hyundai Tau G8BA (Racing Setup)
 *
 * This module owns all fuel calibration data and lookup functions.
 * fuel_injection.c calls into this module for VE lookups and corrections.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Hardware baseline
 * ──────────────────────────────────────────────────────────────────────────
 *   Injectors  : 650 cc/min @ 300 kPa (Siemens/Continental high-flow MPI)
 *   Fuel press : 300 kPa gauge (rail)
 *   Fuel type  : 98 RON unleaded (stoich 14.7:1)
 *   Target λ   : 1.00 (cruise/idle), 0.88 (WOT above 4500 RPM)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Base pulse-width derivation (FUELMAP_BASE_PW_MS)
 * ──────────────────────────────────────────────────────────────────────────
 *   At: 100 % VE, 100 kPa MAP, 20 °C IAT, λ=1.00, 650 cc/min injectors
 *
 *   Vcyl       = G8BA_DISPLACEMENT_CC / G8BA_CYLINDERS
 *              = 4627 / 8 = 578.375 cc
 *
 *   air_mass   = Vcyl × ρ_air_ref = 578.375 × 0.001204 = 0.6965 g
 *   fuel_mass  = air_mass / AFR   = 0.6965 / 14.7     = 0.04737 g
 *   fuel_vol   = fuel_mass / ρ_fuel= 0.04737 / 0.720  = 0.06580 cc
 *
 *   inj_flow   = 650 cc/min = 650/60000 cc/ms = 0.010833 cc/ms
 *   BASE_PW    = fuel_vol / inj_flow = 0.06580 / 0.010833 ≈ 6.074 ms
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Pulse-width formula (in fuel_map_calc)
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   PW = BASE_PW × (VE/100) × (MAP/MAP_ref) × CLT_corr × IAT_corr
 *                × λ_corr   + dead_time_ms
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Table axes
 * ──────────────────────────────────────────────────────────────────────────
 *   RPM  : 16 points — 600 … 7800  (FUELMAP_RPM_AXIS[])
 *   MAP  : 10 points — 20  … 105 kPa (FUELMAP_MAP_AXIS[])
 *   VBatt: 9 points  — 8.0 … 16.0 V  (injector dead-time)
 *   CLT  : 13 points — −20 … 100 °C  (warm-up enrichment)
 *   IAT  : 11 points — −20 … 80 °C   (charge-density correction)
 */

#ifndef FUEL_MAP_H
#define FUEL_MAP_H

#include "g8ba_config.h"   /* rpm_t, floatdeg_t, g8ba_status_t, constants */

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE DIMENSIONS
 * ══════════════════════════════════════════════════════════════════════════ */

#define FUELMAP_RPM_POINTS     16u
#define FUELMAP_MAP_POINTS     10u
#define FUELMAP_VBATT_POINTS    9u
#define FUELMAP_CLT_POINTS     13u
#define FUELMAP_IAT_POINTS     11u

/* ══════════════════════════════════════════════════════════════════════════
 * INJECTOR CONSTANTS (650 cc/min high-flow racing injectors)
 * ══════════════════════════════════════════════════════════════════════════ */

/** Static injector flow rate at rated pressure (cc/min) */
#define FUELMAP_INJ_FLOW_CC_MIN     650.0f

/** Injector flow rate in cc per millisecond */
#define FUELMAP_INJ_FLOW_CC_MS      (FUELMAP_INJ_FLOW_CC_MIN / 60000.0f)

/**
 * Base pulse width (ms) at 100 % VE, 100 kPa MAP, 20 °C IAT, λ=1.00.
 * Derivation: see file header. Verify on dyno before production.
 */
#define FUELMAP_BASE_PW_MS          6.074f

/**
 * Minimum effective open time (ms) — below this the injector does not
 * open fully; minimum commanded PW = dead_time_ms + FUELMAP_MIN_OPEN_MS.
 * Must be consistent with G8BA_INJ_MIN_PW_US (800 µs = 0.8 ms):
 *   FUELMAP_MIN_OPEN_MS(0.5) + max_dead_time(2.2 ms) = 2.7 ms > 0.8 ms ✓
 */
#define FUELMAP_MIN_OPEN_MS         0.5f

/**
 * Maximum pulse width cap (ms) — prevents injector-wetting at high load
 * with a stuck fuel-cut recovery or transient spike.
 * Must be ≤ G8BA_INJ_MAX_PW_US (25000 µs = 25 ms).
 */
#define FUELMAP_MAX_PW_MS           24.0f

/*
 * FM-6: Compile-time guard — ensure FUELMAP_MAX_PW_MS stays within the
 * hardware injector limit defined in g8ba_config.h.
 * Convert both to µs integer for the comparison (avoids float in #if).
 */
#if ((uint32_t)(FUELMAP_MAX_PW_MS * 1000.0f + 0.5f)) > G8BA_INJ_MAX_PW_US
#  error "FUELMAP_MAX_PW_MS exceeds G8BA_INJ_MAX_PW_US — update one of them"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * Inputs required for a fuel pulse-width calculation.
 * All values must be validated by the caller (non-NaN, in-range).
 */
typedef struct {
    float  rpm;           /**< Engine speed (RPM), must be > 0          */
    float  map_kpa;       /**< Manifold absolute pressure (kPa)         */
    float  vbatt_v;       /**< Battery voltage (V) for dead-time comp.  */
    float  clt_c;         /**< Coolant temperature (°C)                 */
    float  iat_c;         /**< Intake air temperature (°C)              */
    float  lambda_corr;   /**< Closed-loop λ correction (1.0 = none)    */
} fuel_map_inputs_t;

/**
 * Calculated fuel pulse-width result.
 */
typedef struct {
    float  ve_pct;           /**< Volumetric efficiency from table (%)   */
    float  dead_time_ms;     /**< Battery-compensated injector dead time */
    float  pw_cmd_ms;        /**< Total commanded pulse width (ms)       */
    float  pw_effective_ms;  /**< Effective open time (pw_cmd - dead_time) */
    float  clt_corr;         /**< Applied CLT correction factor          */
    float  iat_corr;         /**< Applied IAT correction factor          */
} fuel_map_result_t;

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXIS DECLARATIONS (defined in fuel_map.c)
 * ══════════════════════════════════════════════════════════════════════════ */

extern const uint16_t FUELMAP_RPM_AXIS[FUELMAP_RPM_POINTS];
extern const uint8_t  FUELMAP_MAP_AXIS[FUELMAP_MAP_POINTS];
extern const float    FUELMAP_VBATT_AXIS[FUELMAP_VBATT_POINTS];
extern const int8_t   FUELMAP_CLT_AXIS[FUELMAP_CLT_POINTS];
extern const int8_t   FUELMAP_IAT_AXIS[FUELMAP_IAT_POINTS];

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise fuel map module.
 *         Validates table monotonicity at startup (debug builds).
 * @return G8BA_OK always; G8BA_ERR_RANGE if table sanity fails (debug only).
 */
g8ba_status_t fuel_map_init(void);

/**
 * @brief  Compute complete fuel pulse width from sensor inputs.
 *
 *         Applies:
 *           1. 2D bilinear VE lookup (RPM × MAP)
 *           2. Battery dead-time compensation
 *           3. CLT warm-up enrichment
 *           4. IAT charge-density correction
 *           5. Lambda closed-loop correction
 *           6. Minimum and maximum PW clamps
 *
 * @param  in   Validated sensor snapshot (caller must ensure non-NULL)
 * @param  out  Result struct populated by this function (non-NULL)
 */
void fuel_map_calc(const fuel_map_inputs_t *in, fuel_map_result_t *out);

/**
 * @brief  Look up volumetric efficiency (%) from RPM × MAP table.
 * @param  rpm      Engine speed (RPM)
 * @param  map_kpa  Manifold pressure (kPa)
 * @return VE in percent (e.g. 96.5 %).  Clamped to table range.
 */
float fuel_map_get_ve(float rpm, float map_kpa);

/**
 * @brief  Look up injector dead time from battery voltage table.
 * @param  vbatt_v  Measured battery voltage (V)
 * @return Dead time in milliseconds.
 */
float fuel_map_get_dead_time_ms(float vbatt_v);

/**
 * @brief  CLT warm-up enrichment correction factor.
 * @param  clt_c  Coolant temperature (°C)
 * @return Multiplicative correction (1.00 when warm, up to 1.40 when cold).
 */
float fuel_map_get_clt_correction(float clt_c);

/**
 * @brief  IAT charge-density correction factor.
 *         Compensates for air density change vs reference temperature (20 °C).
 * @param  iat_c  Intake air temperature (°C)
 * @return Multiplicative correction (1.00 at 30 °C, <1 when hot, >1 when cold).
 */
float fuel_map_get_iat_correction(float iat_c);

#endif /* FUEL_MAP_H */
