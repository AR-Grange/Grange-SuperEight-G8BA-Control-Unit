/**
 * @file    engine_protection.h
 * @brief   Module 6 — Engine Protection (Overheat / Low Oil / Rev Limit)
 *
 * Monitors critical engine parameters and enforces protection strategies:
 *
 *   Protection Type   │ Strategy
 *   ──────────────────┼──────────────────────────────────────────────
 *   Overheat (CLT)    │ Warning → power reduction → fuel cut
 *   Low oil pressure  │ Warning → power reduction → fuel cut
 *   High oil temp     │ Warning → power reduction
 *   Rev limit         │ Soft cut (alternating) → Hard cut (all)
 *   Boost overrun     │ Partial fuel cut (n/a NA engine — reserved)
 *   Sync lost         │ Immediate fuel + ignition cut
 *
 * All protection actions are communicated to fuel_injection and
 * ignition_control modules via their respective cut/mode APIs.
 */

#ifndef ENGINE_PROTECTION_H
#define ENGINE_PROTECTION_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Protection event levels */
typedef enum {
    PROT_LEVEL_OK      = 0,   /* nominal, no action                        */
    PROT_LEVEL_WARN    = 1,   /* warning — operator notification only      */
    PROT_LEVEL_REDUCE  = 2,   /* power reduction (fuel/ign trim)           */
    PROT_LEVEL_CUT     = 3,   /* fuel or ignition cut                      */
    PROT_LEVEL_EMERG   = 4,   /* immediate shutdown / hard cut             */
} prot_level_t;

/** Bitmask of active protection events */
typedef enum {
    PROT_EVENT_NONE         = 0x0000,
    PROT_EVENT_CLT_WARN     = 0x0001,
    PROT_EVENT_CLT_REDUCE   = 0x0002,
    PROT_EVENT_CLT_CUT      = 0x0004,
    PROT_EVENT_OIL_PRESS_W  = 0x0008,
    PROT_EVENT_OIL_PRESS_CUT= 0x0010,
    PROT_EVENT_OIL_TEMP_W   = 0x0020,
    PROT_EVENT_OIL_TEMP_CUT = 0x0040,
    PROT_EVENT_REV_SOFT     = 0x0080,
    PROT_EVENT_REV_HARD     = 0x0100,
    PROT_EVENT_SYNC_LOST    = 0x0200,
    PROT_EVENT_SENSOR_FAULT = 0x0400,
} prot_event_flags_t;

/** Input snapshot for protection evaluation */
typedef struct {
    float       clt_c;             /* coolant temperature (°C)             */
    float       oil_pressure_kpa;  /* oil pressure (kPa)                   */
    float       oil_temp_c;        /* oil temperature (°C)                 */
    float       map_kpa;           /* manifold pressure (kPa abs)          */
    rpm_t       rpm;
    bool        sync_ok;           /* crank/cam sync healthy               */
    float       vbatt;             /* battery voltage (V)                  */
} prot_inputs_t;

/** Rev limiter state */
typedef struct {
    bool        soft_active;       /* soft cut currently engaged           */
    bool        hard_active;       /* hard cut currently engaged           */
    uint8_t     soft_cut_mask;     /* current alternating cylinder mask    */
    uint8_t     soft_cut_toggle;   /* toggles each cycle for alternation   */
    rpm_t       hysteresis_rpm;    /* re-enable RPM (soft_cut - hysteresis)*/
} rev_limiter_state_t;

/** Thermal protection state */
typedef struct {
    prot_level_t    clt_level;
    prot_level_t    oil_temp_level;
    prot_level_t    oil_press_level;
    float           power_reduction_pct;  /* 0 = full power, 100 = zero     */
} thermal_state_t;

/** Module-level protection status */
typedef struct {
    prot_inputs_t       inputs;
    prot_event_flags_t  active_events;    /* bitmask of current events      */
    prot_level_t        overall_level;    /* worst active level             */
    rev_limiter_state_t rev_limiter;
    thermal_state_t     thermal;
    uint32_t            clt_warn_count;   /* CLT warning event counter      */
    uint32_t            oil_warn_count;   /* Oil pressure warning counter   */
    us_t                last_update_us;
} prot_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile prot_status_t g_protection;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise engine protection module.
 *         Sets thresholds, clears event flags, configures warning outputs.
 * @return G8BA_OK on success.
 */
g8ba_status_t engine_protection_init(void);

/**
 * @brief  Main protection evaluation — call every TASK_PERIOD_SLOW_MS (50ms).
 *         Reads all sensors, evaluates thresholds, applies fuel/ign cuts.
 * @param  inputs  Current sensor snapshot
 */
void engine_protection_update(const prot_inputs_t *inputs);

/**
 * @brief  Rev limiter evaluation — call every TASK_PERIOD_MEDIUM_MS (10ms).
 *         Must be called more frequently than protection_update for smooth cut.
 * @param  rpm  Current RPM
 */
void rev_limiter_update(rpm_t rpm);

/**
 * @brief  Return true if any hard fuel/ignition cut is currently active.
 */
bool prot_is_hard_cut_active(void);

/**
 * @brief  Return current power reduction factor (0.0 = full, 1.0 = zero power).
 */
float prot_get_power_reduction(void);

/**
 * @brief  Return current overall protection level.
 */
prot_level_t prot_get_level(void);

/**
 * @brief  Return active event flags bitmask.
 */
prot_event_flags_t prot_get_events(void);

/**
 * @brief  Force immediate fuel + ignition cut (sync lost / emergency).
 *         Notifies both fuel_injection and ignition_control modules.
 *         Sets PROT_EVENT_SYNC_LOST flag.
 */
void prot_emergency_cut(void);

/**
 * @brief  Clear emergency cut — only callable when sync is restored.
 */
void prot_emergency_clear(void);

/**
 * @brief  Send warning indicator to CAN bus / dashboard output.
 * @param  event  Event that triggered the warning
 */
void prot_send_warning(prot_event_flags_t event);

/**
 * @brief  Log a protection event to fault memory (non-volatile).
 * @param  event   Event type
 * @param  level   Severity level
 * @param  value   Sensor value that triggered the event
 */
void prot_log_fault(prot_event_flags_t event, prot_level_t level, float value);

#endif /* ENGINE_PROTECTION_H */
